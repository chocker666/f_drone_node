#include "drone_node/exec/action_manager.hpp"
#include "action_helpers.hpp"

#include <chrono>
#include <string>
#include <future>
#include <utility>

namespace drone_node::exec
{

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

}  // namespace drone_node::exec
