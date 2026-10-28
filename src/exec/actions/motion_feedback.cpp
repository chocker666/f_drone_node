#include "drone_node/exec/action_manager.hpp"
#include "action_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <cmath>

namespace drone_node::exec
{

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

    if (motion_->source_type == ActionType::LAND) {
        if (motion_->terminal_event &&
            motion_->terminal_event->outcome ==
                MotionExecutor::ExecutionOutcome::COMPLETED &&
            motion_->terminal_event->safe_stop_confirmed) {
            return 1.0;
        }

        const double total =
            motion_->start_z - LAND_TARGET_Z;

        if (total <= 0.0)
            return motion_->progress_base;

        return std::clamp(
            (motion_->start_z - p.z) / total,
            0.0,
            0.99);
    }

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
        node_.get_parameter(
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
        motion_->source_type == ActionType::LAND &&
        !motion_->is_continue) {
        value = p.z;
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

}  // namespace drone_node::exec
