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

}  // namespace

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

    const double stop_timeout =
        motion_executor_->get_parameter(
            "execution.stop_timeout").as_double();

    const double hover_ack_timeout =
        motion_executor_->get_parameter(
            "execution.hover_ack_timeout").as_double();

    const auto validate_constraints = [
        stop_timeout,
        hover_ack_timeout
    ](
        const MotionExecutor::ExecutionConstraints& constraints,
        const std::string& name)
    {
        if (constraints.yaw_tolerance > std::acos(-1.0) ||
            constraints.timeout <= constraints.settle_duration ||
            stop_timeout <=
                hover_ack_timeout + constraints.settle_duration) {
            throw std::invalid_argument(
                "Invalid " + name +
                " tolerance, execution or stop timeout");
        }
    };

    validate_constraints(takeoff_constraints_, "Takeoff");
    validate_constraints(goto_constraints_, "GoTo");

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

    timer_ = create_wall_timer(
        period, [this] { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "ActionManager initialized: Arm/Disarm/Takeoff/GoTo enabled; "
        "Land/Stop goals are rejected");
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
                else if constexpr (
                    std::is_same_v<ActionT, GoTo>) {
                    return handleGoToGoal(uuid, *goal);
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
                std::is_same_v<ActionT, Takeoff> ||
                std::is_same_v<ActionT, GoTo>) {
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
                std::is_same_v<ActionT, Takeoff> ||
                std::is_same_v<ActionT, GoTo>) {
                attachMotionGoal<ActionT>(handle);
            }
        });
}

rclcpp_action::GoalResponse ActionManager::handleArmGoal(
    const GoalUUID& uuid,
    bool desired_arm)
{
    refreshSnapshot();

    // 在 Goal 回调中预留任务槽，避免 accepted 回调前重复接纳。
    if (active_ || motion_) {
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

rclcpp_action::CancelResponse ActionManager::handleArmCancel(
    const GoalUUID& uuid)
{
    if (!active_ || active_->uuid != uuid)
        return rclcpp_action::CancelResponse::REJECT;

    // SetArming 没有撤回接口，不发送反向命令冒充取消。
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

rclcpp_action::GoalResponse ActionManager::handleTakeoffGoal(
    const GoalUUID& uuid,
    const Takeoff::Goal& goal)
{
    refreshSnapshot();

    if (active_ || motion_) {
        RCLCPP_WARN(
            get_logger(),
            "Takeoff rejected: another task is active");

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

    MotionOperation operation;
    operation.uuid = uuid;
    operation.type = ActionType::TAKEOFF;

    operation.height = goal.height == 0.0
        ? takeoff_default_height_
        : goal.height;

    operation.constraints = takeoff_constraints_;
    operation.confirmation_timeout =
        takeoff_confirmation_timeout_;

    motion_.emplace(std::move(operation));

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionManager::handleGoToGoal(
    const GoalUUID& uuid,
    const GoTo::Goal& goal)
{
    refreshSnapshot();

    if (active_ || motion_) {
        RCLCPP_WARN(
            get_logger(),
            "GoTo rejected: another task is active");

        return rclcpp_action::GoalResponse::REJECT;
    }

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

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionManager::handleMotionCancel(
    const GoalUUID& uuid)
{
    if (!motion_ || motion_->uuid != uuid)
        return rclcpp_action::CancelResponse::REJECT;

    // 接受取消仅表示记录放弃任务的意图。
    // 发布后的任务必须等待真实安全停止确认。
    motion_->cancel_requested = true;

    return rclcpp_action::CancelResponse::ACCEPT;
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
            else if constexpr (std::is_same_v<ActionT, GoTo>) {
                feedback->remaining_distance = value;
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

    // 释放队列锁后再处理其他模块。
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

        // 重复普通事件不延长确认期限；失败事件优先。
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

    // 只清理本地关联，不撤销已经发送的服务请求。
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

    clearPendingRequest();

    auto finish = std::move(active_->finish);
    active_.reset();

    try {
        finish(state, error, text);
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
        text.c_str());
}

void ActionManager::finishMotion(
    ActionState state,
    ErrorCode error,
    const std::string& message)
{
    if (!motion_ || !motion_->attached)
        return;

    // message 可能引用任务内的 failure，先复制再释放任务。
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

    const double state_timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, state_timeout) ||
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

    const auto& position = raw.position.message.point;

    double value = 0.0;
    double progress = 0.0;

    if (motion_->type == ActionType::TAKEOFF) {
        value = position.z - motion_->start_z;

        progress = std::clamp(
            value / motion_->height,
            0.0,
            1.0);
    }
    else {
        const auto& target = motion_->motion.target_position;

        value = std::hypot(
            position.x - target.x(),
            position.y - target.y(),
            position.z - target.z());

        if (motion_->initial_distance > 1.0e-9) {
            progress = std::clamp(
                1.0 - value / motion_->initial_distance,
                0.0,
                1.0);
        }
        else {
            // 位置相同仍然需要确认 Yaw、稳定和 HOVER。
            progress =
                value <= motion_->constraints.position_tolerance
                    ? 1.0
                    : 0.0;
        }
    }

    motion_->feedback(
        phase,
        static_cast<float>(progress),
        value);
}

void ActionManager::tick()
{
    try {
        refreshSnapshot();
        processExecutionEvents();

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
        if (motion_ && motion_->attached) {
            if (motion_->trajectory_id) {
                try {
                    motion_executor_->requestStop(
                        *motion_->trajectory_id,
                        MotionExecutor::StopReason::CLIENT_CANCEL);
                }
                catch (...) {
                    // 继续报告故障，不宣称已安全停止。
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

    const auto elapsed = [](Clock::time_point since)
    {
        return std::chrono::duration<double>(
            Clock::now() - since).count();
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

        // 发送异常时保守地按可能已经发送处理。
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

    const double state_timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, state_timeout) ||
        !freshCache(raw.velocity, now, state_timeout) ||
        !freshCache(raw.attitude, now, state_timeout) ||
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

    if (motion_->type == ActionType::TAKEOFF) {
        request.planner_type = PlannerType::FIXED;
        request.frame_id = "map";

        request.target_position = Eigen::Vector3d(
            position.x,
            position.y,
            position.z + motion_->height);

        request.target_yaw =
            measuredYaw(raw.attitude.message);

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

    motion_->start_z = position.z;
    motion_->initial_distance = initial_distance;
    motion_->motion = planning.motion;

    const auto started_event =
        motion_->type == ActionType::TAKEOFF
            ? FlightEvent::TAKEOFF_STARTED
            : FlightEvent::GOTO_STARTED;

    // 先建立任务飞行阶段，再向 Ctrl 发布轨迹。
    const auto transition =
        fsm_->process_event(started_event);

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
        motion_->type == ActionType::TAKEOFF ? "Takeoff" : "GoTo",
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

    const double state_timeout =
        motion_executor_->get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, state_timeout) ||
        !freshCache(raw.velocity, now, state_timeout) ||
        !freshCache(raw.attitude, now, state_timeout) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        return false;
    }

    const auto& position = raw.position.message.point;
    const auto& velocity = raw.velocity.message.vector;
    const auto& target = motion_->motion.target_position;

    const double position_error = std::hypot(
        position.x - target.x(),
        position.y - target.y(),
        position.z - target.z());

    const double speed =
        std::hypot(velocity.x, velocity.y, velocity.z);

    const double yaw_delta =
        measuredYaw(raw.attitude.message) -
        motion_->motion.target_yaw;

    const double yaw_error = std::abs(std::atan2(
        std::sin(yaw_delta),
        std::cos(yaw_delta)));

    return
        position_error <=
            motion_->constraints.position_tolerance &&
        speed <=
            motion_->constraints.velocity_tolerance &&
        yaw_error <=
            motion_->constraints.yaw_tolerance;
}

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

    const auto now = Clock::now();

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

        // 已发生故障时，安全停止成功也不能改报正常完成。
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
            // Cancel 与正常完成事件可能交叉到达。
            // 已有安全停止证明时，不重复发送 Ctrl CANCEL。
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
            const auto completed_event =
                motion_->type == ActionType::TAKEOFF
                    ? FlightEvent::TAKEOFF_COMPLETED
                    : FlightEvent::GOTO_COMPLETED;

            const auto transition =
                fsm_->process_event(completed_event);

            if (transition.accepted) {
                finishMotion(
                    ActionState::SUCCEEDED,
                    ErrorCode::NONE,
                    "Motion target reached; stable HOVER confirmed");
                return;
            }
        }

        // 执行完成事件与独立稳定悬停观测可能相差几个周期。
        if (std::chrono::duration<double>(
                now - motion_->event_received_at).count() >=
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
        // FAULT 不会被停止请求解除。
        if (fsm_->phase() != FlightPhase::FAULT) {
            const auto transition =
                fsm_->process_event(FlightEvent::STOP_STARTED);

            if (!transition.accepted) {
                motion_->failure =
                    "Stop transition rejected: " + transition.reason;
                latchFault(motion_->failure);
            }
        }

        const bool accepted =
            motion_executor_->requestStop(
                *motion_->trajectory_id,
                MotionExecutor::StopReason::CLIENT_CANCEL);

        motion_->stop_requested = true;

        if (!accepted) {
            // 可能执行已结束，而事件尚在跨节点队列中。
            // false 不代表停止成功，继续等待匹配终态事件。
            RCLCPP_WARN(
                get_logger(),
                "Motion stop request not attached to active execution; "
                "waiting for the matching terminal event");
        }
    }

    // 执行器负责主要超时；这里防止终态事件丢失后永久占槽。
    const double stop_timeout =
        motion_executor_->get_parameter(
            "execution.stop_timeout").as_double();

    const double result_timeout =
        motion_->constraints.timeout +
        stop_timeout +
        motion_->confirmation_timeout;

    if (std::chrono::duration<double>(
            now - motion_->dispatched_at).count() >=
        result_timeout) {
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

}  // namespace drone_node::exec