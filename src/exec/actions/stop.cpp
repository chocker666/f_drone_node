#include "drone_node/exec/action_manager.hpp"
#include "action_helpers.hpp"

#include <chrono>
#include <string>

namespace drone_node::exec
{

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
        // 已落地的 Land 无需保存为可继续任务。
        if (motion_->source_type == ActionType::LAND &&
            observation.landed) {
            paused_.reset();
        }
        else {
            paused_ = capturePausedTask();
        }
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

    // 原任务终态发送异常时，不能再报告 Stop 成功。
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
