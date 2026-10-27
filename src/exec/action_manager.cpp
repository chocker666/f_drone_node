#include "drone_node/exec/action_manager.hpp"
#include "actions/action_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace drone_node::exec
{

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
    takeoff_constraints_.complete_on_landed = false;

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
    goto_constraints_.complete_on_landed = false;

    land_constraints_.position_tolerance = positive_parameter(
        "land.position_tolerance", 0.15);
    land_constraints_.yaw_tolerance = positive_parameter(
        "land.yaw_tolerance", 0.10);
    land_constraints_.velocity_tolerance = positive_parameter(
        "land.velocity_tolerance", 0.15);
    land_constraints_.settle_duration = positive_parameter(
        "land.settle_duration", 0.8);
    land_constraints_.timeout = positive_parameter(
        "land.timeout", 30.0);
    land_confirmation_timeout_ = positive_parameter(
        "land.confirmation_timeout", 2.0);

    land_auto_disarm_ = declare_parameter<bool>(
        "land.auto_disarm", false);

    land_constraints_.require_airborne = false;
    land_constraints_.complete_on_landed = true;

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
    validate_constraints(land_constraints_, "Land");

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
    continue_server_ = createServer<Continue>("/drone_node/continue");
    hover_server_ = createServer<Hover>("/drone_node/hover");

    reset_fault_service_ =
        create_service<std_srvs::srv::Trigger>(
            "/drone_node/reset_fault",
            std::bind(
                &ActionManager::handleResetFault,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    timer_ = create_wall_timer(period, [this] { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "ActionManager initialized: "
        "Arm/Disarm/Takeoff/GoTo/Land/Stop/Continue/Hover enabled; "
        "Land auto_disarm=%s",
        land_auto_disarm_ ? "true" : "false");
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
                    std::is_same_v<ActionT, Land>) {
                    return handleLandGoal(uuid, *goal);
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
                std::is_same_v<ActionT, Land> ||
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
                std::is_same_v<ActionT, Land> ||
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

rclcpp_action::GoalResponse ActionManager::handleLandGoal(
    const GoalUUID& uuid,
    const Land::Goal& goal)
{
    refreshSnapshot();

    if (active_ || motion_ || stop_)
        return rclcpp_action::GoalResponse::REJECT;

    // 保留 action 字段兼容性，但本版不控制下降速度。
    if (!std::isfinite(goal.descent_speed) ||
        goal.descent_speed < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "Land rejected: descent_speed must be finite and >= 0");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto admission =
        fsm_->check_action(ActionType::LAND);

    if (!admission.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "Land rejected: %s",
            admission.reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal.descent_speed > 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "Land descent_speed is ignored: "
            "POINT descent uses the existing controller configuration");
    }

    MotionOperation operation;
    operation.uuid = uuid;
    operation.type = ActionType::LAND;
    operation.source_type = ActionType::LAND;
    operation.constraints = land_constraints_;
    operation.confirmation_timeout =
        land_confirmation_timeout_;

    motion_.emplace(std::move(operation));
    paused_.reset();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionManager::handleStopGoal(
    const GoalUUID& uuid)
{
    refreshSnapshot();

    // Stop 不撤销已发送或正在准备发送的 Arm/Disarm 服务。
    // Land 的内部 Disarm 阶段同样占用 active_。
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

    const auto transition_type =
        saved.source_type == ActionType::LAND
            ? ActionType::LAND
            : (resume_ground_takeoff
                ? ActionType::TAKEOFF
                : ActionType::GOTO);

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

    if (motion_->landing_disarm) {
        // 服务发送后无法撤回，不接纳取消。
        if (!active_ || active_->request_sent)
            return rclcpp_action::CancelResponse::REJECT;

        motion_->cancel_requested = true;
        active_->cancel_requested = true;
        paused_.reset();

        return rclcpp_action::CancelResponse::ACCEPT;
    }

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
                if (!handle->is_canceling()) {
                    throw std::logic_error(
                        "Arm goal is not CANCELING");
                }
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

            if constexpr (
                std::is_same_v<ActionT, Takeoff> ||
                std::is_same_v<ActionT, Land>) {
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
                if (!handle->is_canceling()) {
                    throw std::logic_error(
                        "Stop goal is not CANCELING");
                }
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

void ActionManager::handleResetFault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    response->success = false;

    // 1. 必须已经处于 FAULT。
    if (fsm_->phase() != FlightPhase::FAULT) {
        response->message = "Exec is not in FAULT";
        return;
    }

    // 2. 不允许有尚未结束的 Action 事务。
    if (active_ || motion_ || stop_) {
        response->message =
            "Cannot reset: an Action operation is still active";
        return;
    }

    // 3. 不允许存在尚未处理的执行事件。
    {
        std::lock_guard<std::mutex> lock(event_mutex_);

        if (!execution_events_.empty()) {
            response->message =
                "Cannot reset: pending execution events";
            return;
        }
    }

    // 4. 获取 MotionExecutor 的最新真实观测。
    refreshSnapshot();

    const auto snapshot = fsm_->snapshot();

    // 5. 必须确认 PX4 已连接。
    if (!snapshot.connected_valid || !snapshot.connected) {
        response->message =
            "Cannot reset: PX4 connection is unavailable";
        return;
    }

    // 6. 第一版只允许已锁定、已落地的情况。
    if (!snapshot.armed_valid || snapshot.armed) {
        response->message =
            "Cannot reset: vehicle must be disarmed";
        return;
    }

    if (!snapshot.landed_valid || !snapshot.landed) {
        response->message =
            "Cannot reset: vehicle must be landed";
        return;
    }

    // 7. Ctrl 必须已经停止轨迹执行，且没有故障。
    if (!snapshot.control_status_valid ||
        !snapshot.control_available ||
        snapshot.control_fault ||
        snapshot.control_tracking ||
        snapshot.execution_active) {
        response->message =
            "Cannot reset: Ctrl is not ready";
        return;
    }

    // 8. 调用 FSM 已有的显式恢复事件。
    // FSM 会再次检查快照新鲜度及控制条件。
    const auto result =
        fsm_->process_event(FlightEvent::STATE_RECOVERED);

    if (!result.accepted) {
        response->message =
            "FSM recovery rejected: " + result.reason;

        RCLCPP_WARN(
            get_logger(),
            "%s",
            response->message.c_str());

        return;
    }

    // 9. 清除失效的暂停任务。
    // 不调用 MotionExecutor 的运动命令，
    // 不改变 PX4 Arm 状态。
    paused_.reset();

    response->success = true;
    response->message =
        "Exec FAULT cleared; flight phase is GROUNDED";

    RCLCPP_INFO(
        get_logger(),
        "Manual FAULT reset succeeded: %s -> %s",
        to_string(result.previous),
        to_string(result.current));
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

            if (motion_->landing_disarm) {
                if (!active_) {
                    latchFault(
                        "Land Disarm stage has no active service operation");

                    finishMotion(
                        ActionState::FAILED,
                        ErrorCode::INTERNAL_ERROR,
                        "Land Disarm stage has no active service operation");
                    return;
                }

                if (fsm_->phase() == FlightPhase::FAULT) {
                    finishActive(
                        ActionState::FAILED,
                        ErrorCode::VEHICLE_FAULT,
                        fsm_->fault_reason());
                    return;
                }

                // 已接纳的取消优先处理，避免取消后再报告成功。
                if (active_->cancel_requested) {
                    tickArm();
                    return;
                }

                const auto observation = fsm_->snapshot();

                // PX4 可能已在落地后自动 Disarm。
                // 尚未发送服务时，使用当前真实状态完成内部步骤。
                if (!active_->request_sent &&
                    observation.armed_valid &&
                    !observation.armed &&
                    observation.landed_valid &&
                    observation.landed) {
                    finishActive(
                        ActionState::SUCCEEDED,
                        ErrorCode::NONE,
                        "Actual Disarm already confirmed");
                    return;
                }

                tickArm();
                return;
            }

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

        if (motion_ && motion_->landing_disarm && active_) {
            latchFault(
                std::string("Land Disarm exception: ") +
                error.what());

            finishActive(
                ActionState::FAILED,
                ErrorCode::INTERNAL_ERROR,
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
        else if (motion_->type == ActionType::LAND &&
                 !motion_->resumed_target) {
            if (p.z <= LAND_TARGET_Z)
                return std::nullopt;

            saved.start_z = p.z;
            saved.request.target_position = Eigen::Vector3d(
                p.x, p.y, LAND_TARGET_Z);
            saved.request.target_yaw =
                measuredYaw(raw.attitude.message);
        }
        else if (motion_->is_hover &&
                 !motion_->resumed_target) {
            saved.request.target_position = Eigen::Vector3d(
                p.x, p.y, p.z);
            saved.request.target_yaw =
                measuredYaw(raw.attitude.message);
        }
    }

    return saved;
}

}  // namespace drone_node::exec