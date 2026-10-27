#include "drone_node/exec/action_manager.hpp"
#include "action_helpers.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace drone_node::exec
{

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

}  // namespace drone_node::exec
