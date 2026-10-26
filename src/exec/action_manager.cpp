#include "drone_node/exec/action_manager.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace drone_node::exec
{

namespace
{

template<typename CacheT>
bool freshCache(
    const CacheT& cache,
    std::chrono::steady_clock::time_point now,
    double timeout)
{
    return cache.received &&
           cache.valid &&
           now >= cache.received_at &&
           std::chrono::duration<double>(
               now - cache.received_at).count() <= timeout;
}

double elapsedSince(
    std::chrono::steady_clock::time_point since)
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - since).count();
}

std::int64_t statusStamp(
    const drone_node::msg::ControlStatus& status)
{
    return static_cast<std::int64_t>(status.stamp.sec) *
               1000000000LL +
           static_cast<std::int64_t>(status.stamp.nanosec);
}

}  // namespace

// ============================================================
// Construction
// ============================================================

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

    goto_constraints_.position_tolerance = positive_parameter(
        "goto.position_tolerance", 0.15);

    goto_constraints_.yaw_tolerance = positive_parameter(
        "goto.yaw_tolerance", 0.10);

    goto_constraints_.velocity_tolerance = positive_parameter(
        "goto.velocity_tolerance", 0.15);

    goto_constraints_.settle_duration = positive_parameter(
        "goto.settle_duration", 0.5);

    goto_constraints_.timeout = positive_parameter(
        "goto.timeout", 30.0);

    goto_confirmation_timeout_ = positive_parameter(
        "goto.confirmation_timeout", 2.0);

    goto_constraints_.require_airborne = true;

    const double executor_stop_timeout =
        motion_executor_->get_parameter(
            "execution.stop_timeout").as_double();

    const double hover_ack_timeout =
        motion_executor_->get_parameter(
            "execution.hover_ack_timeout").as_double();

    const auto validate_constraints = [
        executor_stop_timeout,
        hover_ack_timeout
    ](
        const MotionExecutor::ExecutionConstraints& constraints,
        const std::string& name)
    {
        if (constraints.yaw_tolerance > std::acos(-1.0) ||
            constraints.timeout <= constraints.settle_duration ||
            executor_stop_timeout <=
                hover_ack_timeout + constraints.settle_duration) {
            throw std::invalid_argument(
                "Invalid " + name +
                " tolerance, execution or stop timeout");
        }
    };

    validate_constraints(takeoff_constraints_, "Takeoff");
    validate_constraints(goto_constraints_, "GoTo");

    stop_timeout_ = positive_parameter(
        "stop.timeout", 8.0);

    stop_settle_duration_ = positive_parameter(
        "stop.settle_duration", 0.5);

    if (stop_timeout_ <=
        executor_stop_timeout + stop_settle_duration_) {
        throw std::invalid_argument(
            "stop.timeout must exceed execution.stop_timeout "
            "plus stop.settle_duration");
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

    continue_server_ =
        createServer<Continue>("/drone_node/continue");

    hover_server_ =
        createServer<Hover>("/drone_node/hover");

    timer_ = create_wall_timer(
        period, [this] { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "ActionManager initialized: "
        "Arm/Disarm/Takeoff/GoTo/Stop/Continue/Hover enabled; "
        "Land goals are rejected");
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

// ============================================================
// Action server routing
// ============================================================

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
                else if constexpr (
                    std::is_same_v<ActionT, GoTo>) {
                    return handleGoToGoal(uuid, *goal);
                }
                else if constexpr (
                    std::is_same_v<ActionT, Stop>) {
                    return handleStopGoal(uuid);
                }
                else if constexpr (
                    std::is_same_v<ActionT, Continue>) {
                    return handleContinueGoal(uuid);
                }
                else if constexpr (
                    std::is_same_v<ActionT, Hover>) {
                    return handleHoverGoal(uuid, *goal);
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
                std::is_same_v<ActionT, Stop>) {
                return handleStopCancel(handle->get_goal_id());
            }
            else if constexpr (
                std::is_same_v<ActionT, Takeoff> ||
                std::is_same_v<ActionT, GoTo> ||
                std::is_same_v<ActionT, Continue> ||
                std::is_same_v<ActionT, Hover>) {
                return handleMotionCancel(handle->get_goal_id());
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
                std::is_same_v<ActionT, Stop>) {
                attachStopGoal(handle);
            }
            else if constexpr (
                std::is_same_v<ActionT, Takeoff> ||
                std::is_same_v<ActionT, GoTo> ||
                std::is_same_v<ActionT, Continue> ||
                std::is_same_v<ActionT, Hover>) {
                attachMotionGoal<ActionT>(handle);
            }
        });
}

// ============================================================
// Goal admission
// ============================================================

rclcpp_action::GoalResponse ActionManager::handleArmGoal(
    const GoalUUID& uuid,
    bool desired_arm)
{
    refreshSnapshot();

    if (active_ || motion_ || stop_) {
        RCLCPP_WARN(
            get_logger(),
            "Arm/Disarm rejected: another task is active");

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

rclcpp_action::GoalResponse ActionManager::handleTakeoffGoal(
    const GoalUUID& uuid,
    const Takeoff::Goal& goal)
{
    refreshSnapshot();

    if (active_ || motion_ || stop_)
        return rclcpp_action::GoalResponse::REJECT;

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

    MotionOperation operation;
    operation.uuid = uuid;
    operation.type = ActionType::TAKEOFF;
    operation.source_type = ActionType::TAKEOFF;

    operation.height = goal.height == 0.0
        ? takeoff_default_height_
        : goal.height;

    operation.constraints = takeoff_constraints_;
    operation.confirmation_timeout =
        takeoff_confirmation_timeout_;

    motion_.emplace(std::move(operation));
    paused_.reset();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionManager::handleGoToGoal(
    const GoalUUID& uuid,
    const GoTo::Goal& goal)
{
    refreshSnapshot();

    if (active_ || motion_ || stop_)
        return rclcpp_action::GoalResponse::REJECT;

    if (goal.header.frame_id != "map" ||
        !std::isfinite(goal.position.x) ||
        !std::isfinite(goal.position.y) ||
        !std::isfinite(goal.position.z) ||
        !std::isfinite(goal.yaw)) {
        RCLCPP_WARN(
            get_logger(),
            "GoTo rejected: requires finite map/ENU position and yaw");

        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto admission =
        fsm_->check_action(ActionType::GOTO);

    if (!admission.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "GoTo rejected: %s",
            admission.reason.c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    MotionOperation operation;
    operation.uuid = uuid;
    operation.type = ActionType::GOTO;
    operation.source_type = ActionType::GOTO;
    operation.constraints = goto_constraints_;
    operation.confirmation_timeout =
        goto_confirmation_timeout_;

    operation.request.planner_type = PlannerType::FIXED;
    operation.request.frame_id = "map";

    operation.request.target_position = Eigen::Vector3d(
        goal.position.x,
        goal.position.y,
        goal.position.z);

    operation.request.target_yaw = goal.yaw;
    operation.request.yaw_direction = 0;
    operation.request.controller_id = "position_yaw_pid";
    operation.request.config_name = "default";

    motion_.emplace(std::move(operation));
    paused_.reset();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionManager::handleStopGoal(
    const GoalUUID& uuid)
{
    refreshSnapshot();

    // Stop 仅抢占运动任务，不撤销不可撤回的 Arm 服务。
    if (active_ || stop_)
        return rclcpp_action::GoalResponse::REJECT;

    StopOperation operation;
    operation.uuid = uuid;

    stop_.emplace(std::move(operation));

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionManager::handleContinueGoal(
    const GoalUUID& uuid)
{
    refreshSnapshot();

    if (active_ || motion_ || stop_ || !paused_)
        return rclcpp_action::GoalResponse::REJECT;

    const auto saved = *paused_;

    const bool resume_ground_takeoff =
        saved.source_type == ActionType::TAKEOFF &&
        fsm_->phase() == FlightPhase::GROUNDED;

    const auto transition_type = resume_ground_takeoff
        ? ActionType::TAKEOFF
        : ActionType::GOTO;

    const auto admission =
        fsm_->check_action(transition_type);

    if (!admission.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "Continue rejected: %s",
            admission.reason.c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    MotionOperation operation;
    operation.uuid = uuid;
    operation.type = transition_type;
    operation.source_type = saved.source_type;

    operation.is_continue = true;
    operation.resumed_target = true;

    operation.request = saved.request;
    operation.constraints = saved.constraints;
    operation.confirmation_timeout =
        saved.confirmation_timeout;

    operation.height = saved.height;
    operation.start_z = saved.start_z;
    operation.progress_base = saved.progress;

    operation.is_hover = saved.is_hover;
    operation.hover_duration = saved.hover_remaining;
    operation.hover_total_duration =
        saved.hover_total_duration;
    operation.hover_elapsed_before =
        saved.hover_elapsed;

    motion_.emplace(std::move(operation));

    // 接纳后消费暂停槽；失败和取消均不恢复旧槽。
    paused_.reset();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionManager::handleHoverGoal(
    const GoalUUID& uuid,
    const Hover::Goal& goal)
{
    refreshSnapshot();

    if (active_ || motion_ || stop_ ||
        !std::isfinite(goal.duration) ||
        goal.duration < 0.0) {
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto admission =
        fsm_->check_action(ActionType::HOVER);

    if (!admission.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "Hover rejected: %s",
            admission.reason.c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    MotionOperation operation;
    operation.uuid = uuid;

    operation.type = ActionType::GOTO;
    operation.source_type = ActionType::HOVER;
    operation.is_hover = true;

    operation.constraints = goto_constraints_;
    operation.confirmation_timeout =
        goto_confirmation_timeout_;

    operation.hover_duration = goal.duration;
    operation.hover_total_duration = goal.duration;

    motion_.emplace(std::move(operation));
    paused_.reset();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ============================================================
// Cancel admission
// ============================================================

rclcpp_action::CancelResponse ActionManager::handleArmCancel(
    const GoalUUID& uuid)
{
    if (!active_ || active_->uuid != uuid)
        return rclcpp_action::CancelResponse::REJECT;

    if (active_->request_sent) {
        RCLCPP_WARN(
            get_logger(),
            "Arm/Disarm cancel rejected: request already sent");

        return rclcpp_action::CancelResponse::REJECT;
    }

    active_->cancel_requested = true;

    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse ActionManager::handleMotionCancel(
    const GoalUUID& uuid)
{
    if (!motion_ || motion_->uuid != uuid)
        return rclcpp_action::CancelResponse::REJECT;

    motion_->cancel_requested = true;
    paused_.reset();

    // 已有 Stop 时，客户端明确放弃优先于保存暂停任务。
    if (stop_ && motion_->trajectory_id) {
        motion_executor_->requestStop(
            *motion_->trajectory_id,
            MotionExecutor::StopReason::CLIENT_CANCEL);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse ActionManager::handleStopCancel(
    const GoalUUID& uuid)
{
    if (!stop_ ||
        stop_->internal_cancel ||
        stop_->uuid != uuid) {
        return rclcpp_action::CancelResponse::REJECT;
    }

    // 取消 Stop 不恢复原运动，仍继续完成安全停止。
    stop_->cancel_requested = true;
    paused_.reset();

    return rclcpp_action::CancelResponse::ACCEPT;
}

// ============================================================
// GoalHandle adapters
// ============================================================

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
        result->message = "Accepted Arm goal has no matching slot";

        handle->abort(result);
        latchFault(result->message);
        return;
    }

    active_->feedback =
        [handle](
            std::uint8_t stage,
            const std::string& message)
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
                if (!handle->is_canceling())
                    throw std::logic_error("Arm goal is not CANCELING");

                handle->canceled(result);
            }
            else {
                handle->abort(result);
            }
        };

    active_->attached = true;
    active_->feedback(0, "Accepted; no SetArming request sent");
}

template<typename ActionT>
void ActionManager::attachMotionGoal(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<ActionT>>& handle)
{
    if (!motion_ || motion_->uuid != handle->get_goal_id()) {
        auto result = std::make_shared<typename ActionT::Result>();
        result->success = false;
        result->error_code =
            static_cast<std::uint16_t>(ErrorCode::INTERNAL_ERROR);
        result->message = "Accepted motion goal has no matching slot";

        handle->abort(result);
        latchFault(result->message);
        return;
    }

    motion_->feedback =
        [handle](
            const std::string& phase,
            float progress,
            double value)
        {
            if (!handle->is_executing())
                return;

            auto feedback =
                std::make_shared<typename ActionT::Feedback>();

            feedback->phase = phase;
            feedback->progress = progress;

            if constexpr (std::is_same_v<ActionT, Takeoff>) {
                feedback->current_height = value;
            }
            else if constexpr (
                std::is_same_v<ActionT, GoTo> ||
                std::is_same_v<ActionT, Continue>) {
                feedback->remaining_distance = value;
            }
            else if constexpr (
                std::is_same_v<ActionT, Hover>) {
                feedback->elapsed_time = value;
            }

            handle->publish_feedback(feedback);
        };

    motion_->finish =
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
                        "Motion goal is not CANCELING");
                }

                handle->canceled(result);
            }
            else {
                handle->abort(result);
            }
        };

    motion_->attached = true;
}

void ActionManager::attachStopGoal(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<Stop>>& handle)
{
    if (!stop_ || stop_->uuid != handle->get_goal_id()) {
        auto result = std::make_shared<Stop::Result>();
        result->success = false;
        result->error_code =
            static_cast<std::uint16_t>(ErrorCode::INTERNAL_ERROR);
        result->message = "Accepted Stop has no matching slot";

        handle->abort(result);
        latchFault(result->message);
        return;
    }

    stop_->feedback =
        [handle](
            std::uint8_t stage,
            const std::string& message)
        {
            if (!handle->is_executing())
                return;

            auto feedback = std::make_shared<Stop::Feedback>();
            feedback->stage = stage;
            feedback->message = message;
            handle->publish_feedback(feedback);
        };

    stop_->finish =
        [handle](
            ActionState state,
            ErrorCode error,
            const std::string& message)
        {
            auto result = std::make_shared<Stop::Result>();
            result->success = state == ActionState::SUCCEEDED;
            result->error_code =
                static_cast<std::uint16_t>(error);
            result->message = message;

            if (state == ActionState::SUCCEEDED) {
                handle->succeed(result);
            }
            else if (state == ActionState::CANCELED) {
                if (!handle->is_canceling())
                    throw std::logic_error("Stop goal is not CANCELING");

                handle->canceled(result);
            }
            else {
                handle->abort(result);
            }
        };

    stop_->attached = true;
}

// ============================================================
// Observation and execution events
// ============================================================

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
        if (!motion_ ||
            !motion_->trajectory_id ||
            *motion_->trajectory_id != event.trajectory_id) {
            RCLCPP_WARN(
                get_logger(),
                "Unbound motion event ignored: trajectory_id=%llu",
                static_cast<unsigned long long>(
                    event.trajectory_id));
            continue;
        }

        if (!motion_->terminal_event ||
            event.outcome ==
                MotionExecutor::ExecutionOutcome::FAILED) {
            motion_->terminal_event = event;
            motion_->event_received_at = Clock::now();
        }

        if (event.outcome ==
            MotionExecutor::ExecutionOutcome::FAILED) {
            motion_->failure = event.message;
            latchFault(event.message);
        }
    }
}

void ActionManager::refreshSnapshot()
{
    fsm_->update_snapshot(
        motion_executor_->getVehicleSnapshot());

    if (paused_) {
        const auto observation = fsm_->snapshot();

        const bool invalid =
            fsm_->phase() == FlightPhase::FAULT ||
            !observation.connected_valid ||
            !observation.connected ||
            !observation.armed_valid ||
            !observation.landed_valid ||
            !observation.position_valid ||
            !observation.velocity_valid ||
            !observation.attitude_valid ||
            !observation.control_status_valid ||
            observation.control_fault ||
            (paused_->source_type != ActionType::TAKEOFF &&
             observation.landed);

        if (invalid) {
            paused_.reset();

            RCLCPP_WARN(
                get_logger(),
                "Paused task invalidated by vehicle observation");
        }
    }

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
    paused_.reset();
    fsm_->process_event(FlightEvent::FAULT_DETECTED);

    RCLCPP_ERROR(
        get_logger(),
        "Fault latched: %s",
        reason.c_str());
}

// ============================================================
// Terminal delivery
// ============================================================

void ActionManager::clearPendingRequest()
{
    if (!active_ || !active_->request)
        return;

    const auto request_id = active_->request->request_id;

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

    const std::string text = message;

    if (state == ActionState::SUCCEEDED &&
        !active_->desired_arm) {
        paused_.reset();
    }

    clearPendingRequest();

    auto finish = std::move(active_->finish);
    active_.reset();

    try {
        finish(state, error, text);
    }
    catch (const std::exception& exception) {
        latchFault(
            std::string("Arm terminal delivery failed: ") +
            exception.what());
    }

    RCLCPP_INFO(
        get_logger(),
        "Arm terminal state=%d error=%u: %s",
        static_cast<int>(state),
        static_cast<unsigned>(error),
        text.c_str());
}

void ActionManager::finishMotion(
    ActionState state,
    ErrorCode error,
    const std::string& message)
{
    if (!motion_ || !motion_->attached)
        return;

    const std::string text = message;
    auto finish = std::move(motion_->finish);

    motion_.reset();

    try {
        finish(state, error, text);
    }
    catch (const std::exception& exception) {
        latchFault(
            std::string("Motion terminal delivery failed: ") +
            exception.what());
    }

    RCLCPP_INFO(
        get_logger(),
        "Motion terminal state=%d error=%u: %s",
        static_cast<int>(state),
        static_cast<unsigned>(error),
        text.c_str());
}

void ActionManager::finishStop(
    ActionState state,
    ErrorCode error,
    const std::string& message)
{
    if (!stop_)
        return;

    const std::string text = message;
    auto finish = std::move(stop_->finish);

    stop_.reset();

    if (finish) {
        try {
            finish(state, error, text);
        }
        catch (const std::exception& exception) {
            latchFault(
                std::string("Stop terminal delivery failed: ") +
                exception.what());
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "Stop terminal state=%d error=%u paused=%s: %s",
        static_cast<int>(state),
        static_cast<unsigned>(error),
        paused_ ? "true" : "false",
        text.c_str());
}

// ============================================================
// Manager tick
// ============================================================

void ActionManager::tick()
{
    try {
        refreshSnapshot();
        processExecutionEvents();

        if (stop_) {
            tickStop();
            return;
        }

        if (motion_ && motion_->holding) {
            tickHoverHold();
            return;
        }

        if (motion_) {
            if (!motion_->attached)
                return;

            if (fsm_->phase() == FlightPhase::FAULT &&
                motion_->failure.empty()) {
                motion_->failure =
                    "FSM is in FAULT: " + fsm_->fault_reason();
            }

            tickMotion();
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
        if (stop_) {
            failStop(
                std::string("Stop processing exception: ") +
                error.what());
            return;
        }

        if (motion_ && motion_->attached) {
            if (motion_->trajectory_id) {
                try {
                    motion_executor_->requestStop(
                        *motion_->trajectory_id,
                        MotionExecutor::StopReason::CLIENT_CANCEL);
                }
                catch (...) {
                    // 不宣称已经安全停止。
                }
            }

            if (motion_->fsm_started) {
                latchFault(
                    std::string("Motion exception: ") +
                    error.what());
            }

            finishMotion(
                ActionState::FAILED,
                ErrorCode::INTERNAL_ERROR,
                error.what());
            return;
        }

        if (active_ && active_->request_sent) {
            latchFault(
                std::string("Arm exception after dispatch: ") +
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

// ============================================================
// Arm / Disarm
// ============================================================

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
        message = "Waiting for SetArming service";
        break;
    case ArmPhase::WAIT_RESPONSE:
        message = "Request sent; waiting for service result";
        break;
    case ArmPhase::WAIT_OBSERVATION:
        message = "Service succeeded; checking actual armed state";
        break;
    }

    active_->feedback(
        static_cast<std::uint8_t>(active_->phase),
        message);
}

void ActionManager::tickArm()
{
    if (!active_)
        return;

    if (active_->cancel_requested) {
        finishActive(
            ActionState::CANCELED,
            ErrorCode::CANCELED,
            "Canceled before SetArming request was sent");
        return;
    }

    if (elapsedSince(active_->started_at) >= action_timeout_) {
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

        const auto admission = fsm_->check_action(type);

        if (!admission.accepted) {
            finishActive(
                ActionState::FAILED,
                ErrorCode::STATE_REJECTED,
                "State changed before dispatch: " + admission.reason);
            return;
        }

        if (elapsedSince(active_->started_at) >=
            service_wait_timeout_) {
            finishActive(
                ActionState::FAILED,
                ErrorCode::SERVICE_UNAVAILABLE,
                "SetArming service availability timeout");
            return;
        }

        if (!motion_executor_->armServiceReady())
            return;

        active_->sent_at = Clock::now();
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
        if (elapsedSince(active_->sent_at) >= response_timeout_) {
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

            latchFault("SetArming failed: " + reason);

            finishActive(
                ActionState::FAILED,
                ErrorCode::SERVICE_FAILED,
                reason);
            return;
        }

        active_->response_at = Clock::now();
        active_->phase = ArmPhase::WAIT_OBSERVATION;
    }

    if (active_->phase != ArmPhase::WAIT_OBSERVATION)
        return;

    if (elapsedSince(active_->response_at) >=
        observation_timeout_) {
        latchFault("Actual armed state confirmation timeout");

        finishActive(
            ActionState::FAILED,
            ErrorCode::TIMEOUT,
            "Actual armed state confirmation timeout");
        return;
    }

    const auto raw = motion_executor_->getStateSnapshot();

    refreshSnapshot();

    if (fsm_->phase() == FlightPhase::FAULT) {
        finishActive(
            ActionState::FAILED,
            ErrorCode::VEHICLE_FAULT,
            fsm_->fault_reason());
        return;
    }

    const auto observation = fsm_->snapshot();

    const bool confirmed =
        raw.armed.received &&
        raw.armed.valid &&
        raw.armed.received_at > active_->sent_at &&
        raw.armed.message.data == active_->desired_arm &&
        observation.armed_valid &&
        observation.armed == active_->desired_arm;

    if (!confirmed)
        return;

    const auto transition = fsm_->process_event(
        active_->desired_arm
            ? FlightEvent::ARM_CONFIRMED
            : FlightEvent::DISARM_CONFIRMED);

    if (!transition.accepted) {
        latchFault(
            "Actual armed state could not confirm FSM transition");

        finishActive(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            transition.reason);
        return;
    }

    active_->feedback(
        4, "Service and actual armed state confirmed");

    finishActive(
        ActionState::SUCCEEDED,
        ErrorCode::NONE,
        "SetArming succeeded and actual armed state confirmed");
}

// ============================================================
// Motion planning and target confirmation
// ============================================================

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

void ActionManager::startMotion()
{
    if (!motion_)
        return;

    refreshSnapshot();

    const auto admission =
        fsm_->check_action(motion_->type);

    if (!admission.accepted) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            admission.reason);
        return;
    }

    const auto raw = motion_executor_->getStateSnapshot();
    const auto now = Clock::now();

    const double timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, timeout) ||
        !freshCache(raw.velocity, now, timeout) ||
        !freshCache(raw.attitude, now, timeout) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion start feedback is invalid or stale");
        return;
    }

    const auto& position = raw.position.message.point;
    PlanningRequest request = motion_->request;

    if (motion_->type == ActionType::TAKEOFF &&
        !motion_->resumed_target) {
        request.planner_type = PlannerType::FIXED;
        request.frame_id = "map";

        request.target_position = Eigen::Vector3d(
            position.x,
            position.y,
            position.z + motion_->height);

        request.target_yaw = measuredYaw(raw.attitude.message);
        request.yaw_direction = 0;
        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }
    else if (motion_->is_hover && !motion_->resumed_target) {
        request.planner_type = PlannerType::FIXED;
        request.frame_id = "map";

        request.target_position = Eigen::Vector3d(
            position.x,
            position.y,
            position.z);

        request.target_yaw = measuredYaw(raw.attitude.message);
        request.yaw_direction = 0;
        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }

    const auto planning = planner_.plan(request);

    if (!planning.success()) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion planning failed: " + planning.reason);
        return;
    }

    const double initial_distance = std::hypot(
        request.target_position.x() - position.x,
        request.target_position.y() - position.y,
        request.target_position.z() - position.z);

    if (!std::isfinite(initial_distance)) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion target distance is not finite");
        return;
    }

    if (!motion_->resumed_target)
        motion_->start_z = position.z;

    motion_->initial_distance = initial_distance;
    motion_->motion = planning.motion;

    const auto transition = fsm_->process_event(
        motion_->type == ActionType::TAKEOFF
            ? FlightEvent::TAKEOFF_STARTED
            : FlightEvent::GOTO_STARTED);

    if (!transition.accepted) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            transition.reason);
        return;
    }

    motion_->fsm_started = true;

    std::string rejection;

    const auto id = motion_executor_->startExecution(
        motion_->motion,
        motion_->constraints,
        rejection);

    if (!id) {
        latchFault("Motion dispatch failed: " + rejection);

        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            rejection);
        return;
    }

    motion_->trajectory_id = *id;
    motion_->dispatched_at = Clock::now();

    RCLCPP_INFO(
        get_logger(),
        "%s dispatched: id=%llu "
        "target=(%.3f, %.3f, %.3f) yaw=%.3f",
        motion_->is_continue
            ? "Continue"
            : to_string(motion_->source_type),
        static_cast<unsigned long long>(*id),
        request.target_position.x(),
        request.target_position.y(),
        request.target_position.z(),
        request.target_yaw);
}

bool ActionManager::motionAtTarget() const
{
    if (!motion_)
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

    const double timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, timeout) ||
        !freshCache(raw.velocity, now, timeout) ||
        !freshCache(raw.attitude, now, timeout) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        return false;
    }

    const auto& p = raw.position.message.point;
    const auto& v = raw.velocity.message.vector;
    const auto& target = motion_->motion.target_position;

    const double position_error = std::hypot(
        p.x - target.x(),
        p.y - target.y(),
        p.z - target.z());

    const double speed = std::hypot(v.x, v.y, v.z);

    const double delta =
        measuredYaw(raw.attitude.message) -
        motion_->motion.target_yaw;

    const double yaw_error =
        std::abs(std::atan2(std::sin(delta), std::cos(delta)));

    return
        position_error <=
            motion_->constraints.position_tolerance &&
        speed <=
            motion_->constraints.velocity_tolerance &&
        yaw_error <=
            motion_->constraints.yaw_tolerance;
}

// ============================================================
// Motion progress and feedback
// ============================================================

double ActionManager::motionProgress() const
{
    if (!motion_)
        return 0.0;

    if (motion_->is_hover) {
        if (motion_->hover_total_duration <= 0.0)
            return motion_->holding ? 1.0 : 0.0;

        double elapsed = motion_->hover_elapsed_before;

        if (motion_->holding)
            elapsed += elapsedSince(motion_->hold_started_at);

        return std::clamp(
            elapsed / motion_->hover_total_duration,
            0.0,
            1.0);
    }

    const auto raw = motion_executor_->getStateSnapshot();

    if (!raw.position.received || !raw.position.valid)
        return motion_->progress_base;

    const auto& p = raw.position.message.point;

    if (motion_->source_type == ActionType::TAKEOFF &&
        !motion_->is_continue) {
        return std::clamp(
            (p.z - motion_->start_z) / motion_->height,
            0.0,
            1.0);
    }

    const auto& target = motion_->motion.target_position;

    const double remaining = std::hypot(
        p.x - target.x(),
        p.y - target.y(),
        p.z - target.z());

    const double local = motion_->initial_distance > 1.0e-9
        ? std::clamp(
            1.0 - remaining / motion_->initial_distance,
            0.0,
            1.0)
        : 1.0;

    return motion_->progress_base +
           (1.0 - motion_->progress_base) * local;
}

void ActionManager::publishMotionFeedback()
{
    if (!motion_ || !motion_->trajectory_id)
        return;

    const auto now = Clock::now();

    if (now < motion_->next_feedback_at)
        return;

    motion_->next_feedback_at =
        now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(feedback_period_));

    const auto raw = motion_executor_->getStateSnapshot();

    const double timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, timeout) ||
        raw.position.message.header.frame_id != "map") {
        return;
    }

    std::string phase;

    if (motion_->terminal_event) {
        phase = "WAITING_FSM_CONFIRMATION";
    }
    else {
        switch (motion_executor_->getExecutionSnapshot().phase) {
        case MotionExecutor::ExecutionPhase::WAITING_TRACKING:
            phase = "WAITING_TRACKING";
            break;
        case MotionExecutor::ExecutionPhase::TRACKING:
            phase = "TRACKING";
            break;
        case MotionExecutor::ExecutionPhase::WAITING_HOVER:
            phase = "WAITING_HOVER";
            break;
        case MotionExecutor::ExecutionPhase::IDLE:
            phase = "WAITING_EXECUTION_EVENT";
            break;
        }
    }

    const auto& p = raw.position.message.point;
    double value = 0.0;

    if (motion_->is_hover && !motion_->is_continue) {
        value = motion_->hover_elapsed_before;
    }
    else if (
        motion_->type == ActionType::TAKEOFF &&
        !motion_->is_continue) {
        value = p.z - motion_->start_z;
    }
    else {
        const auto& target = motion_->motion.target_position;

        value = std::hypot(
            p.x - target.x(),
            p.y - target.y(),
            p.z - target.z());
    }

    motion_->feedback(
        phase,
        static_cast<float>(motionProgress()),
        value);
}

// ============================================================
// Motion execution supervision
// ============================================================

void ActionManager::tickMotion()
{
    if (!motion_ || !motion_->attached)
        return;

    if (!motion_->trajectory_id) {
        if (!motion_->failure.empty()) {
            finishMotion(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                motion_->failure);
        }
        else if (motion_->cancel_requested) {
            finishMotion(
                ActionState::CANCELED,
                ErrorCode::CANCELED,
                "Motion canceled before trajectory dispatch");
        }
        else {
            startMotion();
        }

        return;
    }

    if (motion_->terminal_event) {
        const auto event = *motion_->terminal_event;

        if (event.outcome ==
                MotionExecutor::ExecutionOutcome::FAILED ||
            !event.safe_stop_confirmed) {
            latchFault(event.message);

            finishMotion(
                ActionState::FAILED,
                ErrorCode::MOTION_FAILED,
                event.message);
            return;
        }

        if (!motion_->failure.empty()) {
            latchFault(motion_->failure);

            finishMotion(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                motion_->failure);
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
            !(motion_->cancel_requested && canceled)) {
            latchFault("Unexpected motion execution outcome");

            finishMotion(
                ActionState::FAILED,
                ErrorCode::MOTION_FAILED,
                "Unexpected motion execution outcome");
            return;
        }

        if (motion_->cancel_requested &&
            !motion_->stop_requested) {
            const auto transition =
                fsm_->process_event(FlightEvent::STOP_STARTED);

            if (!transition.accepted) {
                latchFault(transition.reason);

                finishMotion(
                    ActionState::FAILED,
                    ErrorCode::STATE_REJECTED,
                    transition.reason);
                return;
            }

            motion_->stop_requested = true;
        }

        refreshSnapshot();

        if (fsm_->phase() == FlightPhase::FAULT) {
            finishMotion(
                ActionState::FAILED,
                ErrorCode::VEHICLE_FAULT,
                fsm_->fault_reason());
            return;
        }

        const auto observation = fsm_->snapshot();

        if (motion_->cancel_requested) {
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
            else if (
                observation.hover_confirmed &&
                !observation.execution_active) {
                confirmation =
                    FlightEvent::STOP_HOVER_CONFIRMED;
            }

            if (confirmation) {
                const auto transition =
                    fsm_->process_event(*confirmation);

                if (transition.accepted) {
                    finishMotion(
                        ActionState::CANCELED,
                        ErrorCode::CANCELED,
                        "Motion canceled after confirmed safe stop");
                    return;
                }
            }
        }
        else if (motionAtTarget()) {
            const auto transition = fsm_->process_event(
                motion_->type == ActionType::TAKEOFF
                    ? FlightEvent::TAKEOFF_COMPLETED
                    : FlightEvent::GOTO_COMPLETED);

            if (transition.accepted) {
                if (motion_->is_hover) {
                    motion_->holding = true;
                    motion_->hold_started_at = Clock::now();

                    if (motion_->hover_duration == 0.0) {
                        finishMotion(
                            ActionState::SUCCEEDED,
                            ErrorCode::NONE,
                            "Stable Hover confirmed");
                    }

                    return;
                }

                finishMotion(
                    ActionState::SUCCEEDED,
                    ErrorCode::NONE,
                    "Motion target reached; stable HOVER confirmed");
                return;
            }
        }

        if (elapsedSince(motion_->event_received_at) >=
            motion_->confirmation_timeout) {
            latchFault("Motion FSM confirmation timeout");

            finishMotion(
                ActionState::FAILED,
                ErrorCode::TIMEOUT,
                "Motion FSM confirmation timeout");
            return;
        }

        publishMotionFeedback();
        return;
    }

    if ((motion_->cancel_requested ||
         !motion_->failure.empty()) &&
        !motion_->stop_requested) {
        if (fsm_->phase() != FlightPhase::FAULT) {
            const auto transition =
                fsm_->process_event(FlightEvent::STOP_STARTED);

            if (!transition.accepted) {
                motion_->failure =
                    "Stop transition rejected: " + transition.reason;
                latchFault(motion_->failure);
            }
        }

        const bool accepted = motion_executor_->requestStop(
            *motion_->trajectory_id,
            MotionExecutor::StopReason::CLIENT_CANCEL);

        motion_->stop_requested = true;

        if (!accepted) {
            RCLCPP_WARN(
                get_logger(),
                "Motion stop request not attached; "
                "waiting for matching terminal event");
        }
    }

    const double executor_stop_timeout =
        motion_executor_->get_parameter(
            "execution.stop_timeout").as_double();

    const double result_timeout =
        motion_->constraints.timeout +
        executor_stop_timeout +
        motion_->confirmation_timeout;

    if (elapsedSince(motion_->dispatched_at) >= result_timeout) {
        motion_executor_->requestStop(
            *motion_->trajectory_id,
            MotionExecutor::StopReason::CLIENT_CANCEL);

        latchFault("Motion terminal event timeout");

        finishMotion(
            ActionState::FAILED,
            ErrorCode::TIMEOUT,
            "Motion terminal event timeout; safe stop not confirmed");
        return;
    }

    publishMotionFeedback();
}

// ============================================================
// Hover hold
// ============================================================

void ActionManager::tickHoverHold()
{
    if (!motion_ || !motion_->holding)
        return;

    if (fsm_->phase() == FlightPhase::FAULT ||
        !motionAtTarget()) {
        latchFault(
            "Hover hold observation or target constraint lost");

        finishMotion(
            ActionState::FAILED,
            ErrorCode::VEHICLE_FAULT,
            "Hover hold observation or target constraint lost");
        return;
    }

    if (motion_->cancel_requested) {
        StopOperation operation;
        operation.internal_cancel = true;
        operation.attached = true;

        stop_.emplace(std::move(operation));
        tickStop();
        return;
    }

    const double elapsed =
        elapsedSince(motion_->hold_started_at);

    if (elapsed >= motion_->hover_duration) {
        finishMotion(
            ActionState::SUCCEEDED,
            ErrorCode::NONE,
            "Requested stable Hover duration completed");
        return;
    }

    const auto now = Clock::now();

    if (now < motion_->next_feedback_at)
        return;

    motion_->next_feedback_at =
        now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(feedback_period_));

    double value =
        motion_->hover_elapsed_before + elapsed;

    if (motion_->is_continue) {
        const auto raw = motion_executor_->getStateSnapshot();
        const auto& p = raw.position.message.point;
        const auto& target = motion_->motion.target_position;

        value = std::hypot(
            p.x - target.x(),
            p.y - target.y(),
            p.z - target.z());
    }

    motion_->feedback(
        "HOLDING",
        static_cast<float>(motionProgress()),
        value);
}

// ============================================================
// Pause capture
// ============================================================

std::optional<ActionManager::PausedTask>
ActionManager::capturePausedTask() const
{
    if (!motion_)
        return std::nullopt;

    PausedTask saved;
    saved.source_type = motion_->source_type;

    saved.constraints = motion_->constraints;
    saved.confirmation_timeout =
        motion_->confirmation_timeout;

    saved.height = motion_->height;
    saved.start_z = motion_->start_z;
    saved.progress = motionProgress();

    saved.is_hover = motion_->is_hover;
    saved.hover_total_duration =
        motion_->hover_total_duration;
    saved.hover_elapsed =
        motion_->hover_elapsed_before;
    saved.hover_remaining =
        motion_->hover_duration;

    if (motion_->holding) {
        const double elapsed =
            elapsedSince(motion_->hold_started_at);

        const double counted =
            std::min(elapsed, motion_->hover_duration);

        saved.hover_elapsed += counted;
        saved.hover_remaining =
            std::max(0.0, motion_->hover_duration - counted);
    }
    else if (motion_->trajectory_id && stop_) {
        const double spent = std::max(
            0.0,
            std::chrono::duration<double>(
                stop_->requested_at -
                motion_->dispatched_at).count());

        saved.constraints.timeout -= spent;

        if (saved.constraints.timeout <=
            saved.constraints.settle_duration) {
            return std::nullopt;
        }
    }

    if (motion_->trajectory_id) {
        saved.request.planner_type = PlannerType::FIXED;
        saved.request.frame_id = motion_->motion.frame_id;

        saved.request.target_position =
            motion_->motion.target_position;

        saved.request.target_yaw =
            motion_->motion.target_yaw;

        saved.request.yaw_direction =
            motion_->motion.yaw_direction;

        saved.request.controller_id =
            motion_->motion.controller_id;

        saved.request.config_name =
            motion_->motion.config_name;
    }
    else {
        saved.request = motion_->request;

        const auto raw = motion_executor_->getStateSnapshot();
        const auto now = Clock::now();

        const double timeout =
            motion_executor_->get_parameter(
                "execution.state_timeout").as_double();

        if (!freshCache(raw.position, now, timeout) ||
            !freshCache(raw.attitude, now, timeout) ||
            raw.position.message.header.frame_id != "map" ||
            raw.attitude.message.header.frame_id != "map") {
            return std::nullopt;
        }

        const auto& p = raw.position.message.point;

        if (motion_->type == ActionType::TAKEOFF &&
            !motion_->resumed_target) {
            saved.start_z = p.z;

            saved.request.target_position = Eigen::Vector3d(
                p.x, p.y, p.z + motion_->height);

            saved.request.target_yaw =
                measuredYaw(raw.attitude.message);
        }
        else if (motion_->is_hover && !motion_->resumed_target) {
            saved.request.target_position =
                Eigen::Vector3d(p.x, p.y, p.z);

            saved.request.target_yaw =
                measuredYaw(raw.attitude.message);
        }
    }

    return saved;
}

// ============================================================
// Stop confirmation
// ============================================================

bool ActionManager::stopObservationConfirmed() const
{
    if (!stop_)
        return false;

    const auto observation = fsm_->snapshot();
    const auto raw = motion_executor_->getStateSnapshot();

    if (!observation.connected_valid ||
        !observation.connected ||
        !observation.armed_valid ||
        !observation.landed_valid ||
        !observation.control_status_valid ||
        observation.control_fault ||
        observation.execution_active) {
        return false;
    }

    if (raw.control_status.received_at <= stop_->requested_at ||
        statusStamp(raw.control_status.message) <=
            stop_->status_stamp_floor) {
        return false;
    }

    if (observation.landed) {
        return !observation.control_tracking &&
               raw.landed.received_at > stop_->requested_at;
    }

    return observation.control_hover &&
           observation.hover_confirmed &&
           raw.position.received_at > stop_->requested_at &&
           raw.velocity.received_at > stop_->requested_at &&
           raw.attitude.received_at > stop_->requested_at;
}

void ActionManager::tickStop()
{
    if (!stop_ || !stop_->attached)
        return;

    if (motion_ && !motion_->attached)
        return;

    if (!stop_->started) {
        stop_->requested_at = Clock::now();

        const auto raw = motion_executor_->getStateSnapshot();
        const auto& status = raw.control_status.message;

        stop_->status_stamp_floor = statusStamp(status);

        if (fsm_->phase() != FlightPhase::FAULT) {
            const auto transition =
                fsm_->process_event(FlightEvent::STOP_STARTED);

            if (!transition.accepted) {
                failStop(transition.reason);
                return;
            }
        }

        bool attached_to_execution = false;

        if (motion_ &&
            motion_->trajectory_id &&
            !motion_->terminal_event) {
            const auto reason = motion_->cancel_requested
                ? MotionExecutor::StopReason::CLIENT_CANCEL
                : MotionExecutor::StopReason::STOP;

            attached_to_execution =
                motion_executor_->requestStop(
                    *motion_->trajectory_id,
                    reason);

            motion_->stop_requested = true;
        }

        if (!attached_to_execution &&
            status.state ==
                MotionExecutor::ControlStatus::TRACKING) {
            MotionExecutor::ControlCommand command;
            command.command =
                MotionExecutor::ControlCommand::CANCEL;

            motion_executor_->sendCommand(command);
        }

        stop_->started = true;

        if (stop_->feedback) {
            stop_->feedback(
                1, "Waiting for actual safe stop");
        }
    }

    if (fsm_->phase() == FlightPhase::FAULT ||
        (motion_ && !motion_->failure.empty())) {
        failStop(
            motion_ && !motion_->failure.empty()
                ? motion_->failure
                : "FSM is in FAULT: " + fsm_->fault_reason());
        return;
    }

    if (elapsedSince(stop_->requested_at) >= stop_timeout_) {
        failStop("Stop confirmation timeout");
        return;
    }

    if (motion_ && motion_->trajectory_id) {
        if (!motion_->terminal_event)
            return;

        const auto& event = *motion_->terminal_event;

        if (event.outcome ==
                MotionExecutor::ExecutionOutcome::FAILED ||
            !event.safe_stop_confirmed) {
            failStop(event.message);
            return;
        }
    }

    if (!stopObservationConfirmed()) {
        stop_->stable_since.reset();
        return;
    }

    const auto now = Clock::now();

    if (!stop_->stable_since) {
        stop_->stable_since = now;
        return;
    }

    if (std::chrono::duration<double>(
            now - *stop_->stable_since).count() <
        stop_settle_duration_) {
        return;
    }

    const auto observation = fsm_->snapshot();

    const auto transition = fsm_->process_event(
        observation.landed
            ? FlightEvent::STOP_GROUNDED_CONFIRMED
            : FlightEvent::STOP_HOVER_CONFIRMED);

    if (!transition.accepted) {
        failStop(transition.reason);
        return;
    }

    const bool parent_canceled =
        motion_ && motion_->cancel_requested;

    const bool abandon =
        parent_canceled ||
        stop_->cancel_requested ||
        stop_->internal_cancel;

    if (abandon) {
        paused_.reset();
    }
    else if (motion_) {
        paused_ = capturePausedTask();
    }

    if (stop_->feedback) {
        stop_->feedback(
            3, "Actual safe stop confirmed");
    }

    if (motion_) {
        finishMotion(
            parent_canceled
                ? ActionState::CANCELED
                : ActionState::FAILED,
            parent_canceled
                ? ErrorCode::CANCELED
                : ErrorCode::PREEMPTED,
            parent_canceled
                ? "Client cancellation confirmed after safe stop"
                : "Preempted by Stop; original Goal will not resume");
    }

    // 原任务的终态发送异常也属于故障，不能再报告 Stop 成功。
    if (fsm_->phase() == FlightPhase::FAULT) {
        finishStop(
            ActionState::FAILED,
            ErrorCode::INTERNAL_ERROR,
            "Original goal terminal delivery failed");
        return;
    }

    const bool stop_canceled = stop_->cancel_requested;

    finishStop(
        stop_canceled
            ? ActionState::CANCELED
            : ActionState::SUCCEEDED,
        stop_canceled
            ? ErrorCode::CANCELED
            : ErrorCode::NONE,
        stop_canceled
            ? "Stop canceled after completing safe stop"
            : "Safe stop confirmed");
}

void ActionManager::failStop(const std::string& reason)
{
    const std::string text = reason;

    // 即使停止事务异常，也尽力请求底层停止。
    if (motion_ && motion_->trajectory_id) {
        try {
            motion_executor_->requestStop(
                *motion_->trajectory_id,
                MotionExecutor::StopReason::CLIENT_CANCEL);
        }
        catch (...) {
            // 继续报告失败，不宣称已停止。
        }
    }

    latchFault(text);

    if (motion_) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::MOTION_FAILED,
            text);
    }

    finishStop(
        ActionState::FAILED,
        ErrorCode::MOTION_FAILED,
        text);
}

}  // namespace drone_node::exec