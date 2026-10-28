#include "drone_node/exec/action_manager.hpp"
#include "action_helpers.hpp"

#include <chrono>
#include <optional>

namespace drone_node::exec
{

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
        else if (motion_->type == ActionType::LAND) {
            if (landingConfirmed()) {
                const auto transition =
                    fsm_->process_event(FlightEvent::LAND_COMPLETED);

                if (transition.accepted) {
                    paused_.reset();

                    if (land_auto_disarm_ &&
                        observation.armed) {
                        beginLandDisarm();
                        return;
                    }

                    finishMotion(
                        ActionState::SUCCEEDED,
                        ErrorCode::NONE,
                        observation.armed
                            ? "Landing confirmed; vehicle remains armed"
                            : "Landing confirmed; vehicle is disarmed");
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
                node_.get_logger(),
                "Motion stop request not attached; "
                "waiting for matching terminal event");
        }
    }

    const double executor_stop_timeout =
        node_.get_parameter(
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

}  // namespace drone_node::exec
