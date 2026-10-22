#include "drone_node/ctrl/trajectory_manager.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "drone_node/ctrl/trajectory_sampler/fixed_sampler.hpp"
#include "drone_node/ctrl/trajectory_sampler/discrete_sampler.hpp"

namespace drone_node
{

TrajectoryManager::AcceptResult TrajectoryManager::accept(
    const ControlTrajectory& msg,
    SteadyTime now,
    std::string& reason)
{
    reason.clear();

    if (msg.frame_id != "map") {
        reason = "unsupported frame_id";
        return AcceptResult::REJECTED;
    }

    if (msg.controller_id.empty() || msg.config_name.empty()) {
        reason = "missing controller or configuration selection";
        return AcceptResult::REJECTED;
    }

    try {
        std::unique_ptr<SamplerBase> candidate;
        uint8_t direction =
            drone_node::msg::FixedTrajectory::SHORTEST;

        switch (msg.trajectory_type) {
        case ControlTrajectory::POINT:
            candidate =
                std::make_unique<trajectory_sampler::PointSampler>(
                    msg.point_params);
            direction = msg.point_params.yaw_direction;
            break;

        case ControlTrajectory::LIST:
            candidate =
                std::make_unique<trajectory_sampler::ListSampler>(
                    msg.list_params);

            // ListSampler rejects an empty list before this access.
            direction =
                msg.list_params.points.front().yaw_direction;
            break;

        case ControlTrajectory::PARAMETRIC:
            reason = "PARAMETRIC sampling is reserved";
            return AcceptResult::REJECTED;

        default:
            reason = "unsupported trajectory_type";
            return AcceptResult::REJECTED;
        }

        reason = "accepted";

        // Validation and sampler construction have completed.
        // Commit immediately; no cross-trajectory blending.
        sampler_ = std::move(candidate);
        yaw_direction_ = direction;
        start_time_ = now;
        trajectory_id_ = msg.trajectory_id;
        has_trajectory_ = true;

        return AcceptResult::ACCEPTED;
    } catch (const std::exception& e) {
        reason = e.what();
        return AcceptResult::REJECTED;
    }
}

bool TrajectoryManager::has_trajectory() const noexcept
{
    return has_trajectory_;
}

bool TrajectoryManager::is_started(SteadyTime now) const noexcept
{
    return has_trajectory_ && now >= start_time_;
}

double TrajectoryManager::elapsed(SteadyTime now) const
{
    if (!has_trajectory_) {
        throw std::logic_error("No active trajectory");
    }

    if (now < start_time_) {
        throw std::invalid_argument(
            "Sample time precedes trajectory activation");
    }

    return std::chrono::duration<double>(
        now - start_time_).count();
}

ReferenceState TrajectoryManager::sample(SteadyTime now) const
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error("No active sampler");
    }

    return sampler_->sample(elapsed(now));
}

double TrajectoryManager::duration() const
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error("No active sampler");
    }

    return sampler_->duration();
}

uint64_t TrajectoryManager::trajectory_id() const noexcept
{
    return trajectory_id_;
}

uint8_t TrajectoryManager::yaw_direction() const noexcept
{
    return yaw_direction_;
}

void TrajectoryManager::clear() noexcept
{
    sampler_.reset();
    yaw_direction_ = drone_node::msg::FixedTrajectory::SHORTEST;
    trajectory_id_ = 0;
    start_time_ = SteadyTime{};
    has_trajectory_ = false;
}

void TrajectoryManager::activate(SteadyTime now)
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error(
            "Cannot activate an empty trajectory");
    }

    start_time_ = now;
}


}  // namespace drone_node