#include "drone_node/exec/action_manager.hpp"

#include <cmath>
#include <future>
#include <stdexcept>
#include <type_traits>
#include <utility>

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
        "ActionManager initialized: Arm/Disarm enabled; "
        "Takeoff/GoTo/Land/Stop goals are rejected");
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
            (void)goal;

            if constexpr (
                std::is_same_v<ActionT, Arm> ||
                std::is_same_v<ActionT, Disarm>) {
                try {
                    return handleArmGoal(
                        uuid, std::is_same_v<ActionT, Arm>);
                }
                catch (const std::exception& error) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "%s goal rejected after exception: %s",
                        name.c_str(),
                        error.what());

                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
            else {
                RCLCPP_WARN(
                    get_logger(),
                    "%s rejected: not implemented in this stage",
                    name.c_str());

                return rclcpp_action::GoalResponse::REJECT;
            }
        },
        [this](
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<ActionT>> handle)
        {
            if constexpr (
                std::is_same_v<ActionT, Arm> ||
                std::is_same_v<ActionT, Disarm>) {
                if (!handle->is_active())
                    return rclcpp_action::CancelResponse::REJECT;

                return handleArmCancel(handle->get_goal_id());
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
            // 未实现 Action 的 Goal 始终被拒绝，
            // 因此不会进入 accepted 回调。
        });
}

rclcpp_action::GoalResponse ActionManager::handleArmGoal(
    const GoalUUID& uuid,
    bool desired_arm)
{
    refreshSnapshot();

    // 在 goal 回调中预留执行槽，防止多个 Goal 在
    // accepted 回调到达前同时通过准入。
    if (active_) {
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

    // 已释放队列锁，之后才访问 FSM 或其他模块。
    for (const auto& event : events) {
        if (event.outcome ==
            MotionExecutor::ExecutionOutcome::FAILED) {
            latchFault(event.message);
        }
        else {
            // 阶段 5 没有已实现的运动 Action。
            // 不把不属于当前 Arm/Disarm 的执行事件映射为成功。
            RCLCPP_WARN(
                get_logger(),
                "Unbound motion event ignored: trajectory_id=%llu",
                static_cast<unsigned long long>(
                    event.trajectory_id));
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

        if (!active_ || !active_->attached)
            return;

        // 故障优先于普通取消或完成。
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

}  // namespace drone_node::exec