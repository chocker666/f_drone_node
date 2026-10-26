#include "drone_node/exec/motion_executor.hpp"

#include <cmath>
#include <string>
#include <algorithm>
#include <atomic>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drone_node::exec
{

MotionExecutor::MotionExecutor(const rclcpp::NodeOptions& options)
    : rclcpp::Node("motion_executor", options)
{
    const auto topic = [this](
        const std::string& parameter,
        const std::string& default_value)
    {
        return declare_parameter<std::string>(
            parameter, default_value);
    };

    trajectory_pub_ = create_publisher<ControlTrajectory>(
        topic(
            "topics.trajectory",
            "/flight_controller/in/trajectory"),
        rclcpp::QoS(1).reliable());

    command_pub_ = create_publisher<ControlCommand>(
        topic(
            "topics.command",
            "/flight_controller/in/command"),
        rclcpp::QoS(10).reliable());

    control_status_sub_ = create_subscription<ControlStatus>(
        topic(
            "topics.control_status",
            "/flight_controller/out/control_status"),
        rclcpp::QoS(10).reliable(),
        [this](ControlStatus::ConstSharedPtr message)
        {
            // position_error/yaw_error 在部分状态下允许为 NaN。
            // FAULT 是有效状态报告，不应因为处于 FAULT 而丢弃。
            const bool valid = message->state <= ControlStatus::FAULT;
            updateCache(state_.control_status, *message, valid);
        });

    const auto state_qos = rclcpp::QoS(1).reliable();

    position_sub_ =
        create_subscription<geometry_msgs::msg::PointStamped>(
            topic(
                "topics.position",
                "/px4_interface/out/position"),
            state_qos,
            [this](
                geometry_msgs::msg::PointStamped::ConstSharedPtr message)
            {
                const auto& position = message->point;
                const bool valid =
                    std::isfinite(position.x) &&
                    std::isfinite(position.y) &&
                    std::isfinite(position.z);

                updateCache(state_.position, *message, valid);
            });

    velocity_sub_ =
        create_subscription<geometry_msgs::msg::Vector3Stamped>(
            topic(
                "topics.velocity",
                "/px4_interface/out/velocity"),
            state_qos,
            [this](
                geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message)
            {
                const auto& velocity = message->vector;
                const bool valid =
                    std::isfinite(velocity.x) &&
                    std::isfinite(velocity.y) &&
                    std::isfinite(velocity.z);

                updateCache(state_.velocity, *message, valid);
            });

    attitude_sub_ =
        create_subscription<geometry_msgs::msg::QuaternionStamped>(
            topic(
                "topics.attitude",
                "/px4_interface/out/attitude"),
            state_qos,
            [this](
                geometry_msgs::msg::QuaternionStamped::ConstSharedPtr message)
            {
                const auto& attitude = message->quaternion;
                const double norm = std::hypot(
                    std::hypot(attitude.x, attitude.y),
                    std::hypot(attitude.z, attitude.w));

                const bool valid =
                    std::isfinite(attitude.x) &&
                    std::isfinite(attitude.y) &&
                    std::isfinite(attitude.z) &&
                    std::isfinite(attitude.w) &&
                    std::isfinite(norm) &&
                    std::abs(norm - 1.0) <= 1.0e-3;

                updateCache(state_.attitude, *message, valid);
            });

    armed_sub_ = create_subscription<std_msgs::msg::Bool>(
        topic("topics.armed", "/px4_interface/out/arm"),
        state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr message)
        {
            updateCache(state_.armed, *message, true);
        });

    landed_sub_ = create_subscription<std_msgs::msg::Bool>(
        topic("topics.landed", "/px4_interface/out/landed"),
        state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr message)
        {
            updateCache(state_.landed, *message, true);
        });

    connected_sub_ = create_subscription<std_msgs::msg::Bool>(
        topic(
            "topics.connected",
            "/px4_interface/out/connected"),
        state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr message)
        {
            updateCache(state_.connected, *message, true);
        });

    arm_client_ = create_client<SetArming>(
        topic("services.arm", "/px4_interface/in/arm"));

            const auto positive_parameter = [this](
        const std::string& name,
        double default_value)
    {
        const double value =
            declare_parameter<double>(name, default_value);

        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                name + " must be finite and positive");
        }

        return value;
    };

        status_timeout_ = positive_parameter(
        "execution.status_timeout", 1.5);

    hover_position_tolerance_ = positive_parameter(
        "observation.hover_position_tolerance", 0.15);

    hover_yaw_tolerance_ = positive_parameter(
        "observation.hover_yaw_tolerance", 0.10);

    hover_velocity_tolerance_ = positive_parameter(
        "observation.hover_velocity_tolerance", 0.15);

    hover_settle_duration_ = positive_parameter(
        "observation.hover_settle_duration", 0.5);

    if (hover_yaw_tolerance_ > std::acos(-1.0)) {
        throw std::invalid_argument(
            "observation.hover_yaw_tolerance must be <= pi");
    }

    const double check_frequency = positive_parameter(
        "execution.check_frequency", 20.0);

    state_timeout_ = positive_parameter(
        "execution.state_timeout", 0.3);

    tracking_ack_timeout_ = positive_parameter(
        "execution.tracking_ack_timeout", 1.0);

    hover_ack_timeout_ = positive_parameter(
        "execution.hover_ack_timeout", 1.0);

    stop_timeout_ = positive_parameter(
        "execution.stop_timeout", 5.0);

    if (stop_timeout_ <= hover_ack_timeout_) {
        throw std::invalid_argument(
            "execution.stop_timeout must exceed hover_ack_timeout");
    }

    const auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / check_frequency));

    if (period.count() <= 0) {
        throw std::invalid_argument(
            "execution.check_frequency is too large");
    }

    execution_timer_ = create_wall_timer(
        period, [this] { updateExecution(); });

    RCLCPP_INFO(
        get_logger(),
        "MotionExecutor communication, execution and "
        "real-state observation initialized; "
        "Action servers are not enabled");
}

void MotionExecutor::sendTrajectory(
    const ControlTrajectory& trajectory)
{
    trajectory_pub_->publish(trajectory);
}

void MotionExecutor::sendCommand(
    const ControlCommand& command)
{
    command_pub_->publish(command);
}

MotionExecutor::StateSnapshot
MotionExecutor::getStateSnapshot() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

bool MotionExecutor::armServiceReady() const
{
    return arm_client_->service_is_ready();
}

std::optional<MotionExecutor::ArmRequest>
MotionExecutor::requestArm(bool arm)
{
    if (!armServiceReady()) {
        return std::nullopt;
    }

    auto request = std::make_shared<SetArming::Request>();
    request->arm = arm;

    return arm_client_->async_send_request(request);
}

bool MotionExecutor::removePendingArmRequest(
    std::int64_t request_id)
{
    return arm_client_->remove_pending_request(request_id);
}

std::int64_t MotionExecutor::statusStamp(
    const ControlStatus& status)
{
    return static_cast<std::int64_t>(status.stamp.sec) *
               1000000000LL +
           static_cast<std::int64_t>(status.stamp.nanosec);
}

double MotionExecutor::yawOf(
    const geometry_msgs::msg::QuaternionStamped& attitude)
{
    const auto& q = attitude.quaternion;

    // 缓存已经检查四元数有效性；这里归一化后求 Yaw。
    const double norm = std::hypot(
        std::hypot(q.x, q.y),
        std::hypot(q.z, q.w));

    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;

    return std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
}

double MotionExecutor::yawDistance(
    double first,
    double second)
{
    const double difference = first - second;

    return std::abs(std::atan2(
        std::sin(difference),
        std::cos(difference)));
}

bool MotionExecutor::validConstraints(
    const ExecutionConstraints& constraints)
{
    const auto positive = [](double value)
    {
        return std::isfinite(value) && value > 0.0;
    };

    return positive(constraints.position_tolerance) &&
           positive(constraints.yaw_tolerance) &&
           constraints.yaw_tolerance <= std::acos(-1.0) &&
           positive(constraints.velocity_tolerance) &&
           positive(constraints.settle_duration) &&
           positive(constraints.timeout) &&
           constraints.timeout > constraints.settle_duration;
}

std::string MotionExecutor::stateProblem(
    const StateSnapshot& state,
    Clock::time_point time) const
{
    if (!fresh(state.position, time))
        return "Position unavailable, invalid or stale";

    if (!fresh(state.velocity, time))
        return "Velocity unavailable, invalid or stale";

    if (!fresh(state.attitude, time))
        return "Attitude unavailable, invalid or stale";

    if (!fresh(state.control_status, time))
        return "ControlStatus unavailable, invalid or stale";

    if (state.position.message.header.frame_id != "map" ||
        state.velocity.message.header.frame_id != "map" ||
        state.attitude.message.header.frame_id != "map") {
        return "Inter feedback frame must be map";
    }

    if (!statusFresh(state.connected, time) ||
        !state.connected.message.data) {
        return "Vehicle connection unavailable or disconnected";
    }

    if (!statusFresh(state.armed, time) ||
        !state.armed.message.data) {
        return "Vehicle armed state unavailable or disarmed";
    }

    return {};
}

bool MotionExecutor::statusAfter(
    const StateSnapshot& state,
    const ExecutionContext& context) const
{
    return state.control_status.received_at >
               context.phase_started_at &&
           statusStamp(state.control_status.message) >
               context.status_stamp_floor;
}

std::optional<std::uint64_t> MotionExecutor::startExecution(
    ControlTrajectory trajectory,
    const ExecutionConstraints& constraints,
    std::string& rejection_reason)
{
    std::lock_guard<std::mutex> lock(execution_mutex_);
    rejection_reason.clear();

    if (execution_) {
        rejection_reason = "Another trajectory is active";
        return std::nullopt;
    }

    if (!validConstraints(constraints)) {
        rejection_reason = "Invalid execution constraints";
        return std::nullopt;
    }

    if (stop_timeout_ <=
        hover_ack_timeout_ + constraints.settle_duration) {
        rejection_reason =
            "stop_timeout must exceed hover_ack_timeout "
            "plus settle_duration";
        return std::nullopt;
    }

    const auto& target = trajectory.point_params;

    if (trajectory.frame_id != "map" ||
        trajectory.trajectory_type != ControlTrajectory::POINT ||
        !std::isfinite(target.position.x) ||
        !std::isfinite(target.position.y) ||
        !std::isfinite(target.position.z) ||
        !std::isfinite(target.yaw) ||
        target.yaw_direction > 2 ||
        trajectory.controller_id.empty() ||
        trajectory.config_name.empty()) {
        rejection_reason = "Invalid map POINT trajectory";
        return std::nullopt;
    }

    const auto state = getStateSnapshot();
    const auto time = Clock::now();

    rejection_reason = stateProblem(state, time);
    if (!rejection_reason.empty())
        return std::nullopt;

    const auto& status = state.control_status.message;

    if (status.state != ControlStatus::IDLE &&
        status.state != ControlStatus::HOVER) {
        rejection_reason = "Ctrl must be IDLE or HOVER";
        return std::nullopt;
    }

    if (status.fault_code != ControlStatus::FAULT_NONE) {
        rejection_reason = "Ctrl reports a fault";
        return std::nullopt;
    }

    // 进程内所有 MotionExecutor 实例共用分配器。
    // 实时时钟纳秒作为初值，之后只递增，避免进程内重复。
    static std::atomic<std::uint64_t> next_id{
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now()
                    .time_since_epoch()).count())};

    auto previous = next_id.load(std::memory_order_relaxed);

    for (;;) {
        if (previous == std::numeric_limits<std::uint64_t>::max()) {
            rejection_reason = "Trajectory ID space exhausted";
            return std::nullopt;
        }

        if (next_id.compare_exchange_weak(
                previous,
                previous + 1,
                std::memory_order_relaxed)) {
            trajectory.trajectory_id = previous + 1;
            break;
        }
    }

    ExecutionContext context;
    context.trajectory = std::move(trajectory);
    context.constraints = constraints;
    context.started_at = time;
    context.phase_started_at = time;
    context.status_stamp_floor = statusStamp(status);

    execution_ = std::move(context);

    try {
        sendTrajectory(execution_->trajectory);
    }
    catch (const std::exception& error) {
        execution_.reset();
        rejection_reason =
            std::string("Trajectory publish failed: ") + error.what();
        return std::nullopt;
    }

    return execution_->trajectory.trajectory_id;
}

bool MotionExecutor::requestStop(
    std::uint64_t trajectory_id,
    StopReason reason)
{
    if (reason != StopReason::STOP &&
        reason != StopReason::CLIENT_CANCEL) {
        return false;
    }

    std::lock_guard<std::mutex> lock(execution_mutex_);

    if (!execution_ ||
        execution_->trajectory.trajectory_id != trajectory_id) {
        return false;
    }

    auto& context = *execution_;

    if (context.phase == ExecutionPhase::WAITING_HOVER) {
        // 故障结论优先；客户端放弃优先于可恢复的 Stop。
        // 不重置停止期限，也不重复发送 CANCEL。
        if (context.stop_reason != StopReason::FAILURE &&
            (reason == StopReason::CLIENT_CANCEL ||
             context.stop_reason == StopReason::COMPLETED)) {
            context.stop_reason = reason;
        }

        return true;
    }

    const auto state = getStateSnapshot();

    beginStopping(
        context, state, Clock::now(), reason);

    return true;
}

MotionExecutor::ExecutionSnapshot
MotionExecutor::getExecutionSnapshot() const
{
    std::lock_guard<std::mutex> lock(execution_mutex_);

    ExecutionSnapshot snapshot;

    if (execution_) {
        snapshot.phase = execution_->phase;
        snapshot.trajectory_id =
            execution_->trajectory.trajectory_id;
        snapshot.stop_reason = execution_->stop_reason;
        snapshot.failure = execution_->failure;
    }

    return snapshot;
}

void MotionExecutor::setExecutionCallback(
    ExecutionCallback callback)
{
    std::lock_guard<std::mutex> lock(execution_mutex_);
    execution_callback_ = std::move(callback);
}

void MotionExecutor::beginStopping(
    ExecutionContext& context,
    const StateSnapshot& state,
    Clock::time_point time,
    StopReason reason,
    const std::string& failure)
{
    context.phase = ExecutionPhase::WAITING_HOVER;
    context.phase_started_at = time;
    context.status_stamp_floor =
        statusStamp(state.control_status.message);
    context.stop_reason = reason;
    context.failure = failure;
    context.stability_started = false;
    context.hover_confirmed = false;

    const bool already_hovering =
        fresh(state.control_status, time) &&
        state.control_status.message.state == ControlStatus::HOVER &&
        state.control_status.message.trajectory_id == 0;

    // 即使已经 HOVER，也要等待本次停止请求之后的新鲜状态。
    // 此时不重复发送没有意义的 CANCEL。
    if (already_hovering)
        return;

    ControlCommand command;
    command.command = ControlCommand::CANCEL;

    try {
        sendCommand(command);
    }
    catch (const std::exception& error) {
        context.stop_reason = StopReason::FAILURE;
        context.failure =
            std::string("CANCEL publish failed: ") + error.what();
    }
}

bool MotionExecutor::settled(
    ExecutionContext& context,
    const StateSnapshot& state,
    Clock::time_point time,
    bool hover)
{
    const auto& position = state.position.message.point;
    const auto& velocity = state.velocity.message.vector;
    const double yaw = yawOf(state.attitude.message);

    const auto& target_position = hover
        ? context.hover_position
        : context.trajectory.point_params.position;

    const double target_yaw = hover
        ? context.hover_yaw
        : context.trajectory.point_params.yaw;

    const double position_error = std::hypot(
        position.x - target_position.x,
        position.y - target_position.y,
        position.z - target_position.z);

    const double speed =
        std::hypot(velocity.x, velocity.y, velocity.z);

    const bool within_tolerance =
        position_error <= context.constraints.position_tolerance &&
        yawDistance(yaw, target_yaw) <=
            context.constraints.yaw_tolerance &&
        speed <= context.constraints.velocity_tolerance;

    if (!within_tolerance) {
        context.stability_started = false;

        // Stop/Cancel 的悬停稳定窗口从当前位置重新建立。
        // 正常完成必须继续保持原任务目标。
        if (hover &&
            context.stop_reason != StopReason::COMPLETED) {
            context.hover_position = position;
            context.hover_yaw = yaw;
        }

        return false;
    }

    if (!context.stability_started) {
        context.stability_started = true;
        context.stable_since = time;
        return false;
    }

    // 稳定窗口内必须收到新的真实位置、速度和姿态数据。
    return std::chrono::duration<double>(
               time - context.stable_since).count() >=
               context.constraints.settle_duration &&
           state.position.received_at > context.stable_since &&
           state.velocity.received_at > context.stable_since &&
           state.attitude.received_at > context.stable_since;
}

MotionExecutor::ExecutionEvent
MotionExecutor::finishExecution(bool safe_stop_confirmed)
{
    const auto& context = *execution_;

    ExecutionEvent event;
    event.trajectory_id = context.trajectory.trajectory_id;
    event.stop_reason = context.stop_reason;
    event.safe_stop_confirmed = safe_stop_confirmed;

    switch (context.stop_reason) {
    case StopReason::COMPLETED:
        event.outcome = ExecutionOutcome::COMPLETED;
        event.message = "Target reached; stable HOVER confirmed";
        break;

    case StopReason::STOP:
        event.outcome = ExecutionOutcome::STOPPED;
        event.message = "Stop completed; stable HOVER confirmed";
        break;

    case StopReason::CLIENT_CANCEL:
        event.outcome = ExecutionOutcome::CANCELED;
        event.message =
            "Client cancellation completed; stable HOVER confirmed";
        break;

    case StopReason::FAILURE:
        event.outcome = ExecutionOutcome::FAILED;
        event.message = context.failure;
        break;
    }

    execution_.reset();
    return event;
}

void MotionExecutor::updateExecution()
{
    std::optional<ExecutionEvent> event;
    ExecutionCallback callback;

    {
        std::lock_guard<std::mutex> lock(execution_mutex_);

        const auto state = getStateSnapshot();
        const auto time = Clock::now();

        // 即使当前没有任务，也持续建立真实悬停观测。
        updateHoverObservation(state, time);

        if (!execution_)
            return;
        auto& context = *execution_;
        const auto& status = state.control_status.message;

        const auto age = [time](Clock::time_point start)
        {
            return std::chrono::duration<double>(
                time - start).count();
        };

        const auto fail_now = [&](
            const std::string& message)
        {
            // 故障/失效时只尽力请求停止，不宣称安全停止成功。
            if (context.phase != ExecutionPhase::WAITING_HOVER) {
                beginStopping(
                    context, state, time,
                    StopReason::FAILURE, message);
            }

            context.stop_reason = StopReason::FAILURE;
            context.failure = message;
            event = finishExecution(false);
        };

        const std::string problem = stateProblem(state, time);

        // 异常优先于到达与普通完成。
        if (!problem.empty()) {
            fail_now(problem);
        }
        else if (
            status.state == ControlStatus::FAULT ||
            status.fault_code != ControlStatus::FAULT_NONE) {
            fail_now(
                "Ctrl FAULT, code=" +
                std::to_string(status.fault_code));
        }
        else if (
            context.constraints.require_airborne &&
            !statusFresh(state.landed, time)) {
            fail_now("Landed observation unavailable or stale");
        }
        else if (
            context.constraints.require_airborne &&
            context.phase == ExecutionPhase::WAITING_HOVER &&
            context.stop_reason == StopReason::COMPLETED &&
            state.landed.message.data) {
            fail_now("Unexpected ground contact during motion completion");
        }
        else {
            const bool later_status = statusAfter(state, context);

            switch (context.phase) {
            case ExecutionPhase::WAITING_TRACKING:
                if (age(context.started_at) >=
                    context.constraints.timeout) {
                    beginStopping(
                        context, state, time,
                        StopReason::FAILURE,
                        "Execution timeout before TRACKING");
                }
                else if (
                    later_status &&
                    status.state == ControlStatus::TRACKING &&
                    status.trajectory_id ==
                        context.trajectory.trajectory_id) {
                    context.phase = ExecutionPhase::TRACKING;
                    context.stability_started = false;
                }
                else if (
                    later_status &&
                    status.state == ControlStatus::TRACKING &&
                    status.trajectory_id !=
                        context.trajectory.trajectory_id) {
                    fail_now("Ctrl is tracking another trajectory");
                }
                else if (
                    age(context.phase_started_at) >=
                    tracking_ack_timeout_) {
                    beginStopping(
                        context, state, time,
                        StopReason::FAILURE,
                        "TRACKING confirmation timeout");
                }
                break;

            case ExecutionPhase::TRACKING:
                if (
                    status.state != ControlStatus::TRACKING ||
                    status.trajectory_id !=
                        context.trajectory.trajectory_id) {
                    beginStopping(
                        context, state, time,
                        StopReason::FAILURE,
                        "Ctrl left the expected TRACKING state");
                }
                else if (
                    age(context.started_at) >=
                    context.constraints.timeout) {
                    beginStopping(
                        context, state, time,
                        StopReason::FAILURE,
                        "Execution timeout");
                }
                else if (
                    context.constraints.require_airborne &&
                    state.landed.message.data) {
                    // 尚未离地不能累计到达稳定时间。
                    context.stability_started = false;
                }
                else if (settled(context, state, time, false)) {
                    beginStopping(
                        context, state, time,
                        StopReason::COMPLETED);
                }
                break;

            case ExecutionPhase::WAITING_HOVER:
                if (age(context.phase_started_at) >= stop_timeout_) {
                    fail_now(
                        "Safe stop timeout; stable HOVER unconfirmed");
                }
                else if (!context.hover_confirmed) {
                    if (
                        later_status &&
                        status.state == ControlStatus::HOVER &&
                        status.trajectory_id == 0) {
                        context.hover_confirmed = true;
                        context.stability_started = false;

                        if (context.stop_reason ==
                            StopReason::COMPLETED) {
                            context.hover_position =
                                context.trajectory.point_params.position;
                            context.hover_yaw =
                                context.trajectory.point_params.yaw;
                        }
                        else {
                            context.hover_position =
                                state.position.message.point;
                            context.hover_yaw =
                                yawOf(state.attitude.message);
                        }
                    }
                    else if (
                        age(context.phase_started_at) >=
                        hover_ack_timeout_) {
                        fail_now("HOVER confirmation timeout");
                    }
                }
                else if (
                    !later_status ||
                    status.state != ControlStatus::HOVER ||
                    status.trajectory_id != 0) {
                    fail_now("Ctrl left HOVER during stop confirmation");
                }
                else if (settled(context, state, time, true)) {
                    event = finishExecution(true);
                }
                break;

            case ExecutionPhase::IDLE:
                break;
            }
        }

        callback = execution_callback_;
    }

    // 不能在持锁期间调用上层模块。
    if (event) {
        RCLCPP_INFO(
            get_logger(),
            "Trajectory %llu outcome=%d safe_stop=%s: %s",
            static_cast<unsigned long long>(event->trajectory_id),
            static_cast<int>(event->outcome),
            event->safe_stop_confirmed ? "true" : "false",
            event->message.c_str());

        if (callback) {
            try {
                callback(*event);
            }
            catch (const std::exception& error) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Execution callback failed: %s",
                    error.what());
            }
            catch (...) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Execution callback failed with unknown exception");
            }
        }
    }
}

std::optional<std::uint64_t> MotionExecutor::startExecution(
    const PlannedMotion& motion,
    const ExecutionConstraints& constraints,
    std::string& rejection_reason)
{
    rejection_reason.clear();

    if (motion.planner_type != PlannerType::FIXED ||
        motion.motion_type != PlannedMotionType::POINT) {
        rejection_reason = "Only FIXED POINT motion is supported";
        return std::nullopt;
    }

    // 同样校验直接构造的 PlannedMotion，
    // 不假定所有调用方都经过了 FixedPlanner。
    if (motion.frame_id != "map" ||
        !motion.target_position.allFinite() ||
        !std::isfinite(motion.target_yaw) ||
        motion.yaw_direction > 2 ||
        motion.controller_id != "position_yaw_pid" ||
        motion.config_name.empty()) {
        rejection_reason = "Invalid planned FIXED POINT motion";
        return std::nullopt;
    }

    ControlTrajectory trajectory;

    // 轨迹 ID 由阶段 2 的执行入口统一分配。
    trajectory.frame_id = motion.frame_id;
    trajectory.trajectory_type = ControlTrajectory::POINT;

    trajectory.point_params.position.x =
        motion.target_position.x();
    trajectory.point_params.position.y =
        motion.target_position.y();
    trajectory.point_params.position.z =
        motion.target_position.z();

    trajectory.point_params.yaw = motion.target_yaw;
    trajectory.point_params.yaw_direction =
        motion.yaw_direction;

    trajectory.controller_id = motion.controller_id;
    trajectory.config_name = motion.config_name;

    return startExecution(
        std::move(trajectory),
        constraints,
        rejection_reason);
}

VehicleSnapshot MotionExecutor::buildVehicleSnapshot(
    const StateSnapshot& state,
    Clock::time_point time) const
{
    VehicleSnapshot result;
    result.captured_at = time;

    result.connected_valid = statusFresh(state.connected, time);
    result.connected = state.connected.message.data;

    result.armed_valid = statusFresh(state.armed, time);
    result.armed = state.armed.message.data;

    result.landed_valid = statusFresh(state.landed, time);
    result.landed = state.landed.message.data;

    result.position_valid =
        fresh(state.position, time) &&
        state.position.message.header.frame_id == "map";

    result.velocity_valid =
        fresh(state.velocity, time) &&
        state.velocity.message.header.frame_id == "map";

    result.attitude_valid =
        fresh(state.attitude, time) &&
        state.attitude.message.header.frame_id == "map";

    result.control_status_valid =
        fresh(state.control_status, time);

    const auto& status = state.control_status.message;

    result.control_fault =
        result.control_status_valid &&
        (status.state == ControlStatus::FAULT ||
         status.fault_code != ControlStatus::FAULT_NONE);

    result.control_available =
        result.control_status_valid &&
        !result.control_fault;

    result.control_tracking =
        result.control_available &&
        status.state == ControlStatus::TRACKING;

    result.control_hover =
        result.control_available &&
        status.state == ControlStatus::HOVER &&
        status.trajectory_id == 0;

    result.execution_active = execution_.has_value();

    result.hover_confirmed =
        observed_hover_confirmed_ &&
        result.connected_valid &&
        result.connected &&
        result.armed_valid &&
        result.armed &&
        result.landed_valid &&
        !result.landed &&
        result.position_valid &&
        result.velocity_valid &&
        result.attitude_valid &&
        result.control_hover;

    return result;
}

VehicleSnapshot MotionExecutor::getVehicleSnapshot() const
{
    // 与执行循环保持相同锁顺序：execution → state。
    std::lock_guard<std::mutex> lock(execution_mutex_);

    const auto state = getStateSnapshot();
    return buildVehicleSnapshot(state, Clock::now());
}

void MotionExecutor::updateHoverObservation(
    const StateSnapshot& state,
    Clock::time_point time)
{
    const auto observation = buildVehicleSnapshot(state, time);

    const bool ready =
        observation.connected_valid &&
        observation.connected &&
        observation.armed_valid &&
        observation.armed &&
        observation.landed_valid &&
        !observation.landed &&
        observation.position_valid &&
        observation.velocity_valid &&
        observation.attitude_valid &&
        observation.control_hover;

    observed_hover_confirmed_ = false;

    if (!ready) {
        hover_stable_since_.reset();
        return;
    }

    const auto& position = state.position.message.point;
    const auto& velocity = state.velocity.message.vector;
    const double yaw = yawOf(state.attitude.message);

    const double speed =
        std::hypot(velocity.x, velocity.y, velocity.z);

    if (speed > hover_velocity_tolerance_) {
        hover_stable_since_.reset();
        return;
    }

    if (!hover_stable_since_) {
        hover_stable_since_ = time;
        hover_anchor_position_ = position;
        hover_anchor_yaw_ = yaw;
        return;
    }

    const double displacement = std::hypot(
        position.x - hover_anchor_position_.x,
        position.y - hover_anchor_position_.y,
        position.z - hover_anchor_position_.z);

    if (displacement > hover_position_tolerance_ ||
        yawDistance(yaw, hover_anchor_yaw_) >
            hover_yaw_tolerance_) {
        hover_stable_since_ = time;
        hover_anchor_position_ = position;
        hover_anchor_yaw_ = yaw;
        return;
    }

    observed_hover_confirmed_ =
        std::chrono::duration<double>(
            time - *hover_stable_since_).count() >=
                hover_settle_duration_ &&
        state.position.received_at > *hover_stable_since_ &&
        state.velocity.received_at > *hover_stable_since_ &&
        state.attitude.received_at > *hover_stable_since_ &&
        state.control_status.received_at > *hover_stable_since_;
}

}  // namespace drone_node::exec