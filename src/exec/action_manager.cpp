#include "drone_node/exec/action_manager.hpp"

#include <cmath>
#include <future>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <algorithm>

namespace drone_node::exec
{

ActionManager::ActionManager(
    std::shared_ptr<MotionExecutor> motion_executor,
    std::shared_ptr<FlightStateMachine> flight_state_machine,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("action_manager", options),
      motion_executor_(std::move(motion_executor)),
      fsm_(std::move(flight_state_machine))
{
    if (!motion_executor_ || !fsm_) {
        throw std::invalid_argument(
            "ActionManager requires MotionExecutor and FSM");
    }

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

    takeoff_default_height_ = positive_parameter(
        "takeoff.default_height", 1.5);

    takeoff_constraints_.position_tolerance = positive_parameter(
        "takeoff.position_tolerance", 0.15);

    takeoff_constraints_.yaw_tolerance = positive_parameter(
        "takeoff.yaw_tolerance", 0.10);

    takeoff_constraints_.velocity_tolerance = positive_parameter(
        "takeoff.velocity_tolerance", 0.15);

    takeoff_constraints_.settle_duration = positive_parameter(
        "takeoff.settle_duration", 0.5);

    takeoff_constraints_.timeout = positive_parameter(
        "takeoff.timeout", 15.0);

    takeoff_confirmation_timeout_ = positive_parameter(
        "takeoff.confirmation_timeout", 2.0);

    takeoff_constraints_.require_airborne = true;

    if (takeoff_constraints_.yaw_tolerance > std::acos(-1.0) ||
        takeoff_constraints_.timeout <=
            takeoff_constraints_.settle_duration) {
        throw std::invalid_argument(
            "Invalid Takeoff yaw tolerance or timeout");
    }

    const double check_frequency = positive_parameter(
        "check_frequency", 20.0);

    const double feedback_frequency = positive_parameter(
        "feedback_frequency", 5.0);

    feedback_period_ = 1.0 / feedback_frequency;

    service_wait_timeout_ = positive_parameter(
        "arm.service_wait_timeout", 3.0);

    response_timeout_ = positive_parameter(
        "arm.response_timeout", 6.0);

    observation_timeout_ = positive_parameter(
        "arm.observation_timeout", 2.5);

    action_timeout_ = positive_parameter(
        "arm.action_timeout", 15.0);

    if (action_timeout_ <=
        service_wait_timeout_ +
        response_timeout_ +
        observation_timeout_) {
        throw std::invalid_argument(
            "arm.action_timeout must exceed the sum of "
            "service_wait_timeout, response_timeout and "
            "observation_timeout");
    }

    const auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / check_frequency));

    if (period.count() <= 0) {
        throw std::invalid_argument(
            "check_frequency is too large");
    }

    arm_server_ = createServer<Arm>("/drone_node/arm");
    disarm_server_ = createServer<Disarm>("/drone_node/disarm");

    takeoff_server_ = createServer<Takeoff>("/drone_node/takeoff");
    goto_server_ = createServer<GoTo>("/drone_node/goto");
    land_server_ = createServer<Land>("/drone_node/land");
    stop_server_ = createServer<Stop>("/drone_node/stop");

    timer_ = create_wall_timer(period, [this] { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "ActionManager initialized: Arm/Disarm/Takeoff enabled; "
        "GoTo/Land/Stop goals are rejected");
}

ActionManager::~ActionManager()
{
    try {
        clearPendingRequest();
    }
    catch (...) {
        // 析构期间不抛出异常。
    }
}

template<typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr
ActionManager::createServer(const std::string& name)
{
    return rclcpp_action::create_server<ActionT>(
        this,
        name,
        [this, name](
            const GoalUUID& uuid,
            std::shared_ptr<const typename ActionT::Goal> goal)
        {
            try {
                if constexpr (
                    std::is_same_v<ActionT, Arm> ||
                    std::is_same_v<ActionT, Disarm>) {
                    return handleArmGoal(
                        uuid, std::is_same_v<ActionT, Arm>);
                }
                else if constexpr (
                    std::is_same_v<ActionT, Takeoff>) {
                    return handleTakeoffGoal(uuid, *goal);
                }
                else {
                    RCLCPP_WARN(
                        get_logger(),
                        "%s rejected: not implemented in this stage",
                        name.c_str());

                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
            catch (const std::exception& error) {
                RCLCPP_ERROR(
                    get_logger(),
                    "%s goal rejected after exception: %s",
                    name.c_str(),
                    error.what());

                return rclcpp_action::GoalResponse::REJECT;
            }
        },
        [this](
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<ActionT>> handle)
        {
            if (!handle->is_active())
                return rclcpp_action::CancelResponse::REJECT;

            if constexpr (
                std::is_same_v<ActionT, Arm> ||
                std::is_same_v<ActionT, Disarm>) {
                return handleArmCancel(handle->get_goal_id());
            }
            else if constexpr (
                std::is_same_v<ActionT, Takeoff>) {
                return handleTakeoffCancel(handle->get_goal_id());
            }
            else {
                return rclcpp_action::CancelResponse::REJECT;
            }
        },
        [this](
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<ActionT>> handle)
        {
            if constexpr (
                std::is_same_v<ActionT, Arm> ||
                std::is_same_v<ActionT, Disarm>) {
                attachArmGoal<ActionT>(handle);
            }
            else if constexpr (
                std::is_same_v<ActionT, Takeoff>) {
                attachTakeoffGoal(handle);
            }
        });
}

rclcpp_action::GoalResponse ActionManager::handleArmGoal(
    const GoalUUID& uuid,
    bool desired_arm)
{
    refreshSnapshot();

    // 在 goal 回调中预留执行槽，防止多个 Goal 在
    // accepted 回调到达前同时通过准入。
    if (active_ || takeoff_) {
        RCLCPP_WARN(
            get_logger(),
            "Arm/Disarm rejected: another action is active");

        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto type = desired_arm
        ? ActionType::ARM
        : ActionType::DISARM;

    const auto admission = fsm_->check_action(type);

    if (!admission.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "%s rejected: %s",
            desired_arm ? "Arm" : "Disarm",
            admission.reason.c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    ArmOperation operation;
    operation.uuid = uuid;
    operation.desired_arm = desired_arm;
    operation.started_at = Clock::now();

    active_.emplace(std::move(operation));

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionManager::handleArmCancel(
    const GoalUUID& uuid)
{
    if (!active_ || active_->uuid != uuid)
        return rclcpp_action::CancelResponse::REJECT;

    // SetArming 无撤回接口。发送后不能把取消请求
    // 伪装为已经撤销，也不自动发送反向命令。
    if (active_->request_sent) {
        RCLCPP_WARN(
            get_logger(),
            "Arm/Disarm cancel rejected: "
            "SetArming request has already been sent");

        return rclcpp_action::CancelResponse::REJECT;
    }

    active_->cancel_requested = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

template<typename ActionT>
void ActionManager::attachArmGoal(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<ActionT>>& handle)
{
    if (!active_ || active_->uuid != handle->get_goal_id()) {
        auto result = std::make_shared<typename ActionT::Result>();
        result->success = false;
        result->error_code =
            static_cast<std::uint16_t>(ErrorCode::INTERNAL_ERROR);
        result->message = "Accepted goal has no matching execution slot";

        handle->abort(result);
        latchFault(result->message);
        return;
    }

    active_->feedback =
        [handle](std::uint8_t stage, const std::string& message)
        {
            if (!handle->is_executing())
                return;

            auto feedback =
                std::make_shared<typename ActionT::Feedback>();

            feedback->stage = stage;
            feedback->message = message;

            handle->publish_feedback(feedback);
        };

    active_->finish =
        [handle](
            ActionState state,
            ErrorCode error,
            const std::string& message)
        {
            auto result =
                std::make_shared<typename ActionT::Result>();

            result->success = state == ActionState::SUCCEEDED;
            result->error_code =
                static_cast<std::uint16_t>(error);
            result->message = message;

            if (state == ActionState::SUCCEEDED) {
                handle->succeed(result);
            }
            else if (state == ActionState::CANCELED) {
                if (!handle->is_canceling()) {
                    throw std::logic_error(
                        "Goal is not in CANCELING state");
                }

                handle->canceled(result);
            }
            else {
                handle->abort(result);
            }
        };

    active_->attached = true;

    active_->feedback(
        0, "Goal accepted; no SetArming request sent yet");
}

void ActionManager::enqueueExecutionEvent(
    const MotionExecutor::ExecutionEvent& event)
{
    std::lock_guard<std::mutex> lock(event_mutex_);
    execution_events_.push_back(event);
}

void ActionManager::processExecutionEvents()
{
    std::deque<MotionExecutor::ExecutionEvent> events;

    {
        std::lock_guard<std::mutex> lock(event_mutex_);
        events.swap(execution_events_);
    }

    for (const auto& event : events) {
        // 所有事件都必须匹配当前任务的轨迹 ID。
        // 迟到的旧事件，包括失败事件，不得终止新任务。
        if (!takeoff_ ||
            !takeoff_->trajectory_id ||
            *takeoff_->trajectory_id != event.trajectory_id) {
            RCLCPP_WARN(
                get_logger(),
                "Unbound motion event ignored: trajectory_id=%llu",
                static_cast<unsigned long long>(
                    event.trajectory_id));
            continue;
        }

        // 重复事件不覆盖已有结果；异常结果优先。
        if (!takeoff_->terminal_event ||
            event.outcome ==
                MotionExecutor::ExecutionOutcome::FAILED) {
            takeoff_->terminal_event = event;
            takeoff_->event_received_at = Clock::now();
        }

        if (event.outcome ==
            MotionExecutor::ExecutionOutcome::FAILED) {
            takeoff_->failure = event.message;
            latchFault(event.message);
        }
    }
}

void ActionManager::refreshSnapshot()
{
    fsm_->update_snapshot(
        motion_executor_->getVehicleSnapshot());

    const auto phase = fsm_->phase();

    if (phase == logged_phase_)
        return;

    RCLCPP_INFO(
        get_logger(),
        "Flight phase: %s -> %s",
        to_string(logged_phase_),
        to_string(phase));

    if (phase == FlightPhase::FAULT) {
        RCLCPP_ERROR(
            get_logger(),
            "FSM fault: %s",
            fsm_->fault_reason().c_str());
    }

    logged_phase_ = phase;
}

void ActionManager::latchFault(const std::string& reason)
{
    fsm_->process_event(FlightEvent::FAULT_DETECTED);

    RCLCPP_ERROR(
        get_logger(),
        "Fault latched: %s",
        reason.c_str());
}

void ActionManager::clearPendingRequest()
{
    if (!active_ || !active_->request)
        return;

    const auto request_id = active_->request->request_id;

    // 只清理本地 future 关联，不撤销已经发送的服务请求。
    motion_executor_->removePendingArmRequest(request_id);
    active_->request.reset();
}

void ActionManager::finishActive(
    ActionState state,
    ErrorCode error,
    const std::string& message)
{
    if (!active_ || !active_->attached)
        return;

    clearPendingRequest();

    auto finish = std::move(active_->finish);
    active_.reset();

    try {
        finish(state, error, message);
    }
    catch (const std::exception& exception) {
        latchFault(
            std::string("Action terminal delivery failed: ") +
            exception.what());
    }

    RCLCPP_INFO(
        get_logger(),
        "Action terminal state=%d error=%u: %s",
        static_cast<int>(state),
        static_cast<unsigned>(error),
        message.c_str());
}

void ActionManager::publishFeedback()
{
    if (!active_ || !active_->attached)
        return;

    const auto now = Clock::now();

    if (now < active_->next_feedback_at)
        return;

    active_->next_feedback_at =
        now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(feedback_period_));

    const char* message = "";

    switch (active_->phase) {
    case ArmPhase::WAIT_SERVICE:
        message = "Waiting for SetArming service; request not sent";
        break;

    case ArmPhase::WAIT_RESPONSE:
        message = "SetArming request sent; waiting for service result";
        break;

    case ArmPhase::WAIT_OBSERVATION:
        message = "Service succeeded; waiting for actual armed state";
        break;
    }

    active_->feedback(
        static_cast<std::uint8_t>(active_->phase),
        message);
}

void ActionManager::tick()
{
    try {
        refreshSnapshot();
        processExecutionEvents();

        if (takeoff_) {
            if (!takeoff_->attached)
                return;

            if (fsm_->phase() == FlightPhase::FAULT &&
                takeoff_->failure.empty()) {
                takeoff_->failure =
                    "FSM is in FAULT: " + fsm_->fault_reason();
            }

            tickTakeoff();
            return;
        }

        if (!active_ || !active_->attached)
            return;

        if (fsm_->phase() == FlightPhase::FAULT) {
            finishActive(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                "FSM is in FAULT: " + fsm_->fault_reason());
            return;
        }

        tickArm();
    }
    catch (const std::exception& error) {
        if (takeoff_ && takeoff_->attached) {
            if (takeoff_->trajectory_id) {
                try {
                    motion_executor_->requestStop(
                        *takeoff_->trajectory_id,
                        MotionExecutor::StopReason::CLIENT_CANCEL);
                }
                catch (...) {
                    // 继续锁存故障，不宣称已经安全停止。
                }
            }

            if (takeoff_->fsm_started) {
                latchFault(
                    std::string("Takeoff exception: ") +
                    error.what());
            }

            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::INTERNAL_ERROR,
                error.what());
            return;
        }

        const bool request_sent =
            active_ && active_->request_sent;

        if (request_sent) {
            latchFault(
                std::string("Arm/Disarm exception after dispatch: ") +
                error.what());
        }

        if (active_ && active_->attached) {
            finishActive(
                ActionState::FAILED,
                ErrorCode::INTERNAL_ERROR,
                error.what());
        }
        else {
            RCLCPP_ERROR(
                get_logger(),
                "ActionManager tick failed: %s",
                error.what());
        }
    }
}

void ActionManager::tickArm()
{
    if (!active_)
        return;

    const auto now = Clock::now();

    const auto elapsed = [now](Clock::time_point since)
    {
        return std::chrono::duration<double>(
            now - since).count();
    };

    if (active_->cancel_requested) {
        finishActive(
            ActionState::CANCELED,
            ErrorCode::CANCELED,
            "Canceled before SetArming request was sent");
        return;
    }

    if (elapsed(active_->started_at) >= action_timeout_) {
        if (active_->request_sent)
            latchFault("Arm/Disarm total timeout after dispatch");

        finishActive(
            ActionState::FAILED,
            ErrorCode::TIMEOUT,
            "Arm/Disarm total timeout");
        return;
    }

    publishFeedback();

    if (active_->phase == ArmPhase::WAIT_SERVICE) {
        const auto type = active_->desired_arm
            ? ActionType::ARM
            : ActionType::DISARM;

        // 等待服务期间状态可能发生变化，发送前重新准入。
        const auto admission = fsm_->check_action(type);

        if (!admission.accepted) {
            finishActive(
                ActionState::FAILED,
                ErrorCode::STATE_REJECTED,
                "State changed before dispatch: " + admission.reason);
            return;
        }

        if (elapsed(active_->started_at) >= service_wait_timeout_) {
            finishActive(
                ActionState::FAILED,
                ErrorCode::SERVICE_UNAVAILABLE,
                "SetArming service availability timeout");
            return;
        }

        if (!motion_executor_->armServiceReady())
            return;

        active_->sent_at = Clock::now();

        // 若发送过程抛出异常，按可能已发送处理。
        active_->request_sent = true;

        auto request =
            motion_executor_->requestArm(active_->desired_arm);

        if (!request) {
            active_->request_sent = false;
            return;
        }

        active_->request.emplace(std::move(*request));
        active_->phase = ArmPhase::WAIT_RESPONSE;

        RCLCPP_INFO(
            get_logger(),
            "SetArming dispatched: arm=%s request_id=%lld",
            active_->desired_arm ? "true" : "false",
            static_cast<long long>(
                active_->request->request_id));

        return;
    }

    if (active_->phase == ArmPhase::WAIT_RESPONSE) {
        if (elapsed(active_->sent_at) >= response_timeout_) {
            latchFault(
                "SetArming response timeout; physical result uncertain");

            finishActive(
                ActionState::FAILED,
                ErrorCode::TIMEOUT,
                "SetArming response timeout");
            return;
        }

        if (active_->request->future.wait_for(
                std::chrono::seconds(0)) !=
            std::future_status::ready) {
            return;
        }

        const auto response = active_->request->future.get();
        active_->request.reset();

        if (!response || !response->success) {
            const std::string reason = response
                ? response->message
                : "Null SetArming response";

            // 发送后的失败统一采取保守策略，
            // 不立即接纳可能相反的 Arm/Disarm 请求。
            latchFault("SetArming failed: " + reason);

            finishActive(
                ActionState::FAILED,
                ErrorCode::SERVICE_FAILED,
                reason);
            return;
        }

        active_->response_at = Clock::now();
        active_->phase = ArmPhase::WAIT_OBSERVATION;

        RCLCPP_INFO(
            get_logger(),
            "SetArming service succeeded: %s; "
            "checking actual armed observation",
            response->message.c_str());
    }

    if (active_->phase != ArmPhase::WAIT_OBSERVATION)
        return;

    if (elapsed(active_->response_at) >= observation_timeout_) {
        latchFault(
            "Actual armed state confirmation timeout");

        finishActive(
            ActionState::FAILED,
            ErrorCode::TIMEOUT,
            "Actual armed state confirmation timeout");
        return;
    }

    // 不只依赖服务响应：必须有请求发出后的真实 armed 消息。
    const auto raw = motion_executor_->getStateSnapshot();

    refreshSnapshot();

    if (fsm_->phase() == FlightPhase::FAULT) {
        finishActive(
            ActionState::FAILED,
            ErrorCode::VEHICLE_FAULT,
            "FSM entered FAULT during armed confirmation");
        return;
    }

    const auto observation = fsm_->snapshot();

    const bool actual_state_confirmed =
        raw.armed.received &&
        raw.armed.valid &&
        raw.armed.received_at > active_->sent_at &&
        raw.armed.message.data == active_->desired_arm &&
        observation.armed_valid &&
        observation.armed == active_->desired_arm;

    if (!actual_state_confirmed)
        return;

    const auto event = active_->desired_arm
        ? FlightEvent::ARM_CONFIRMED
        : FlightEvent::DISARM_CONFIRMED;

    const auto transition = fsm_->process_event(event);

    if (!transition.accepted) {
        latchFault(
            "Armed observation could not confirm FSM transition");

        finishActive(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            transition.reason);
        return;
    }

    active_->feedback(
        4, "Service succeeded and actual armed state confirmed");

    finishActive(
        ActionState::SUCCEEDED,
        ErrorCode::NONE,
        "SetArming succeeded and actual armed state confirmed");
}


double ActionManager::measuredYaw(
    const geometry_msgs::msg::QuaternionStamped& attitude)
{
    const auto& q = attitude.quaternion;

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

rclcpp_action::GoalResponse ActionManager::handleTakeoffGoal(
    const GoalUUID& uuid,
    const Takeoff::Goal& goal)
{
    refreshSnapshot();

    if (active_ || takeoff_) {
        RCLCPP_WARN(get_logger(), "Takeoff rejected: another task is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!std::isfinite(goal.height) || goal.height < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "Takeoff rejected: height must be finite and >= 0");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto admission =
        fsm_->check_action(ActionType::TAKEOFF);

    if (!admission.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "Takeoff rejected: %s",
            admission.reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    TakeoffOperation operation;
    operation.uuid = uuid;
    operation.height = goal.height == 0.0
        ? takeoff_default_height_
        : goal.height;
    operation.constraints = takeoff_constraints_;

    takeoff_.emplace(std::move(operation));

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionManager::handleTakeoffCancel(
    const GoalUUID& uuid)
{
    if (!takeoff_ || takeoff_->uuid != uuid)
        return rclcpp_action::CancelResponse::REJECT;

    // 只记录终止意图，由定时器请求真实停止并等待确认。
    takeoff_->cancel_requested = true;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionManager::attachTakeoffGoal(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<Takeoff>>& handle)
{
    if (!takeoff_ || takeoff_->uuid != handle->get_goal_id()) {
        auto result = std::make_shared<Takeoff::Result>();
        result->success = false;
        result->error_code =
            static_cast<std::uint16_t>(ErrorCode::INTERNAL_ERROR);
        result->message = "Accepted Takeoff has no matching slot";

        handle->abort(result);
        latchFault(result->message);
        return;
    }

    takeoff_->feedback =
        [handle](const Takeoff::Feedback& feedback)
        {
            if (!handle->is_executing())
                return;

            handle->publish_feedback(
                std::make_shared<Takeoff::Feedback>(feedback));
        };

    takeoff_->finish =
        [handle](
            ActionState state,
            ErrorCode error,
            const std::string& message)
        {
            auto result = std::make_shared<Takeoff::Result>();
            result->success = state == ActionState::SUCCEEDED;
            result->error_code = static_cast<std::uint16_t>(error);
            result->message = message;

            if (state == ActionState::SUCCEEDED) {
                handle->succeed(result);
            }
            else if (state == ActionState::CANCELED) {
                if (!handle->is_canceling())
                    throw std::logic_error("Takeoff is not CANCELING");

                handle->canceled(result);
            }
            else {
                handle->abort(result);
            }
        };

    takeoff_->attached = true;
}

void ActionManager::startTakeoff()
{
    refreshSnapshot();

    const auto admission =
        fsm_->check_action(ActionType::TAKEOFF);

    if (!admission.accepted) {
        finishTakeoff(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            admission.reason);
        return;
    }

    const auto raw = motion_executor_->getStateSnapshot();
    const auto now = Clock::now();

    const double state_timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    const auto fresh = [now, state_timeout](const auto& cache)
    {
        return cache.received &&
               cache.valid &&
               now >= cache.received_at &&
               std::chrono::duration<double>(
                   now - cache.received_at).count() <= state_timeout;
    };

    if (!fresh(raw.position) ||
        !fresh(raw.velocity) ||
        !fresh(raw.attitude) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        finishTakeoff(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Takeoff start feedback is invalid or stale");
        return;
    }

    const auto& position = raw.position.message.point;

    PlanningRequest request;
    request.planner_type = PlannerType::FIXED;
    request.frame_id = "map";

    request.target_position = Eigen::Vector3d(
        position.x,
        position.y,
        position.z + takeoff_->height);

    request.target_yaw = measuredYaw(raw.attitude.message);
    request.yaw_direction = 0;
    request.controller_id = "position_yaw_pid";
    request.config_name = "default";

    const auto planning = planner_.plan(request);

    if (!planning.success()) {
        finishTakeoff(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Takeoff planning failed: " + planning.reason);
        return;
    }

    takeoff_->start_z = position.z;
    takeoff_->motion = planning.motion;

    // 必须先进入 TAKING_OFF，再发布轨迹，
    // 防止真实离地被 FSM 当成 GROUNDED 下的意外离地。
    const auto transition =
        fsm_->process_event(FlightEvent::TAKEOFF_STARTED);

    if (!transition.accepted) {
        finishTakeoff(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            transition.reason);
        return;
    }

    takeoff_->fsm_started = true;

    std::string rejection;

    const auto id = motion_executor_->startExecution(
        takeoff_->motion,
        takeoff_->constraints,
        rejection);

    if (!id) {
        // FSM 已经开始任务；执行却未建立，不能留在 TAKING_OFF。
        latchFault("Takeoff dispatch failed: " + rejection);

        finishTakeoff(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            rejection);
        return;
    }

    takeoff_->trajectory_id = *id;
    takeoff_->dispatched_at = Clock::now();

    RCLCPP_INFO(
        get_logger(),
        "Takeoff dispatched: id=%llu relative_height=%.3f "
        "target=(%.3f, %.3f, %.3f) yaw=%.3f",
        static_cast<unsigned long long>(*id),
        takeoff_->height,
        request.target_position.x(),
        request.target_position.y(),
        request.target_position.z(),
        request.target_yaw);
}

bool ActionManager::takeoffAtTarget() const
{
    if (!takeoff_)
        return false;

    const auto observation = fsm_->snapshot();

    if (!observation.position_valid ||
        !observation.velocity_valid ||
        !observation.attitude_valid ||
        !observation.armed_valid ||
        !observation.armed ||
        !observation.landed_valid ||
        observation.landed ||
        !observation.hover_confirmed) {
        return false;
    }

    const auto raw = motion_executor_->getStateSnapshot();
    const auto now = Clock::now();

    const double state_timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    const auto fresh = [now, state_timeout](const auto& cache)
    {
        return cache.received &&
               cache.valid &&
               now >= cache.received_at &&
               std::chrono::duration<double>(
                   now - cache.received_at).count() <= state_timeout;
    };

    if (!fresh(raw.position) ||
        !fresh(raw.velocity) ||
        !fresh(raw.attitude)) {
        return false;
    }

    const auto& position = raw.position.message.point;
    const auto& velocity = raw.velocity.message.vector;
    const auto& target = takeoff_->motion.target_position;

    const double position_error = std::hypot(
        position.x - target.x(),
        position.y - target.y(),
        position.z - target.z());

    const double speed =
        std::hypot(velocity.x, velocity.y, velocity.z);

    const double yaw_delta =
        measuredYaw(raw.attitude.message) -
        takeoff_->motion.target_yaw;

    const double yaw_error = std::abs(std::atan2(
        std::sin(yaw_delta), std::cos(yaw_delta)));

    return
        position_error <= takeoff_->constraints.position_tolerance &&
        speed <= takeoff_->constraints.velocity_tolerance &&
        yaw_error <= takeoff_->constraints.yaw_tolerance;
}

void ActionManager::publishTakeoffFeedback()
{
    if (!takeoff_ || !takeoff_->trajectory_id)
        return;

    const auto now = Clock::now();

    if (now < takeoff_->next_feedback_at)
        return;

    takeoff_->next_feedback_at =
        now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(feedback_period_));

    const auto raw = motion_executor_->getStateSnapshot();

    const double state_timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!raw.position.received ||
        !raw.position.valid ||
        now < raw.position.received_at ||
        std::chrono::duration<double>(
            now - raw.position.received_at).count() > state_timeout) {
        return;
    }

    Takeoff::Feedback feedback;

    feedback.current_height =
        raw.position.message.point.z - takeoff_->start_z;

    feedback.progress = static_cast<float>(std::clamp(
        feedback.current_height / takeoff_->height, 0.0, 1.0));

    if (takeoff_->terminal_event) {
        feedback.phase = "WAITING_FSM_CONFIRMATION";
    }
    else {
        switch (motion_executor_->getExecutionSnapshot().phase) {
        case MotionExecutor::ExecutionPhase::WAITING_TRACKING:
            feedback.phase = "WAITING_TRACKING";
            break;
        case MotionExecutor::ExecutionPhase::TRACKING:
            feedback.phase = "TRACKING";
            break;
        case MotionExecutor::ExecutionPhase::WAITING_HOVER:
            feedback.phase = "WAITING_HOVER";
            break;
        case MotionExecutor::ExecutionPhase::IDLE:
            feedback.phase = "WAITING_EXECUTION_EVENT";
            break;
        }
    }

    takeoff_->feedback(feedback);
}

void ActionManager::tickTakeoff()
{
    if (!takeoff_ || !takeoff_->attached)
        return;

    if (!takeoff_->trajectory_id) {
        if (!takeoff_->failure.empty()) {
            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                takeoff_->failure);
        }
        else if (takeoff_->cancel_requested) {
            finishTakeoff(
                ActionState::CANCELED,
                ErrorCode::CANCELED,
                "Takeoff canceled before trajectory dispatch");
        }
        else {
            startTakeoff();
        }

        return;
    }

    const auto now = Clock::now();

    if (takeoff_->terminal_event) {
        const auto event = *takeoff_->terminal_event;

        if (event.outcome ==
                MotionExecutor::ExecutionOutcome::FAILED ||
            !event.safe_stop_confirmed) {
            latchFault(event.message);

            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::MOTION_FAILED,
                event.message);
            return;
        }

        if (!takeoff_->failure.empty()) {
            latchFault(takeoff_->failure);

            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                takeoff_->failure);
            return;
        }

        const bool completed =
            event.outcome ==
                MotionExecutor::ExecutionOutcome::COMPLETED &&
            event.stop_reason ==
                MotionExecutor::StopReason::COMPLETED;

        const bool canceled =
            event.outcome ==
                MotionExecutor::ExecutionOutcome::CANCELED &&
            event.stop_reason ==
                MotionExecutor::StopReason::CLIENT_CANCEL;

        if (!completed &&
            !(takeoff_->cancel_requested && canceled)) {
            latchFault("Unexpected Takeoff execution outcome");

            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::MOTION_FAILED,
                "Unexpected Takeoff execution outcome");
            return;
        }

        if (takeoff_->cancel_requested &&
            !takeoff_->stop_requested) {
            // Cancel 可能与正常完成事件交叉到达。
            // 已有安全停止证明时不重复发送 CANCEL。
            const auto transition =
                fsm_->process_event(FlightEvent::STOP_STARTED);

            if (!transition.accepted) {
                latchFault(transition.reason);
                finishTakeoff(
                    ActionState::FAILED,
                    ErrorCode::STATE_REJECTED,
                    transition.reason);
                return;
            }

            takeoff_->stop_requested = true;
        }

        refreshSnapshot();

        if (fsm_->phase() == FlightPhase::FAULT) {
            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                fsm_->fault_reason());
            return;
        }

        const auto observation = fsm_->snapshot();

        if (takeoff_->cancel_requested) {
            std::optional<FlightEvent> confirmation;

            if (observation.connected_valid &&
                observation.connected &&
                observation.armed_valid &&
                observation.landed_valid &&
                observation.landed &&
                !observation.execution_active) {
                confirmation =
                    FlightEvent::STOP_GROUNDED_CONFIRMED;
            }
            else if (observation.hover_confirmed &&
                     !observation.execution_active) {
                confirmation =
                    FlightEvent::STOP_HOVER_CONFIRMED;
            }

            if (confirmation) {
                const auto transition =
                    fsm_->process_event(*confirmation);

                if (transition.accepted) {
                    finishTakeoff(
                        ActionState::CANCELED,
                        ErrorCode::CANCELED,
                        "Takeoff canceled after confirmed safe stop");
                    return;
                }
            }
        }
        else if (takeoffAtTarget()) {
            const auto transition =
                fsm_->process_event(FlightEvent::TAKEOFF_COMPLETED);

            if (transition.accepted) {
                finishTakeoff(
                    ActionState::SUCCEEDED,
                    ErrorCode::NONE,
                    "Takeoff target reached; stable HOVER confirmed");
                return;
            }
        }

        // 执行完成事件与独立悬停观测可能相差几个周期。
        // 不在第一个周期条件尚未齐全时立即误报失败。
        if (std::chrono::duration<double>(
                now - takeoff_->event_received_at).count() >=
            takeoff_confirmation_timeout_) {
            latchFault("Takeoff FSM confirmation timeout");

            finishTakeoff(
                ActionState::FAILED,
                ErrorCode::TIMEOUT,
                "Takeoff FSM confirmation timeout");
            return;
        }

        publishTakeoffFeedback();
        return;
    }

    if ((takeoff_->cancel_requested ||
         !takeoff_->failure.empty()) &&
        !takeoff_->stop_requested) {
        // 故障状态不会被 STOP_STARTED 清除。
        if (fsm_->phase() != FlightPhase::FAULT)
            fsm_->process_event(FlightEvent::STOP_STARTED);

        const bool accepted = motion_executor_->requestStop(
            *takeoff_->trajectory_id,
            MotionExecutor::StopReason::CLIENT_CANCEL);

        takeoff_->stop_requested = true;

        if (!accepted) {
            // 可能刚完成、事件还在跨节点队列中。
            // 先等待匹配事件，不把 false 当成停止成功。
            RCLCPP_WARN(
                get_logger(),
                "Takeoff stop request not attached to active execution; "
                "waiting for the matching terminal event");
        }
    }

    // 执行器负责主要超时；此处防止终态事件丢失后永久占槽。
    const double stop_timeout =
        motion_executor_->get_parameter(
            "execution.stop_timeout").as_double();

    const double result_timeout =
        takeoff_->constraints.timeout +
        stop_timeout +
        takeoff_confirmation_timeout_;

    if (std::chrono::duration<double>(
            now - takeoff_->dispatched_at).count() >= result_timeout) {
        motion_executor_->requestStop(
            *takeoff_->trajectory_id,
            MotionExecutor::StopReason::CLIENT_CANCEL);

        latchFault("Takeoff terminal event timeout");

        finishTakeoff(
            ActionState::FAILED,
            ErrorCode::TIMEOUT,
            "Takeoff terminal event timeout; safe stop not confirmed");
        return;
    }

    publishTakeoffFeedback();
}

void ActionManager::finishTakeoff(
    ActionState state,
    ErrorCode error,
    const std::string& message)
{
    if (!takeoff_ || !takeoff_->attached)
        return;

    // message 可能引用任务中的 failure，必须先复制再释放任务。
    const std::string text = message;
    auto finish = std::move(takeoff_->finish);

    takeoff_.reset();

    try {
        finish(state, error, text);
    }
    catch (const std::exception& exception) {
        latchFault(
            std::string("Takeoff terminal delivery failed: ") +
            exception.what());
    }

    RCLCPP_INFO(
        get_logger(),
        "Takeoff terminal state=%d error=%u: %s",
        static_cast<int>(state),
        static_cast<unsigned>(error),
        text.c_str());
}

}  // namespace drone_node::exec