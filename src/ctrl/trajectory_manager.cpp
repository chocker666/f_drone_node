#include "drone_node/ctrl/trajectory_manager.hpp"

#include <stdexcept>
#include <utility>

#include "drone_node/ctrl/trajectory_sampler/fixed_sampler.hpp"

namespace drone_node
{

TrajectoryManager::AcceptResult TrajectoryManager::accept(
    const ControlTrajectory& msg,
    const rclcpp::Time& now,
    std::string& reason)
{
    reason.clear();

    if (msg.frame_id != "map") {
        reason = "unsupported frame_id";
        return AcceptResult::REJECTED;
    }

    // LIST execution is introduced in the sampler migration stage.
    // PARAMETRIC remains reserved.
    if (msg.trajectory_type != ControlTrajectory::POINT) {
        reason = "unsupported trajectory_type";
        return AcceptResult::REJECTED;
    }

    // Temporary selection checks until the controller factory
    // and named configuration profiles are introduced.
    if (msg.controller_id != "position_yaw_pid") {
        reason = "unsupported controller_id";
        return AcceptResult::REJECTED;
    }

    if (msg.config_name != "default") {
        reason = "unsupported config_name";
        return AcceptResult::REJECTED;
    }

    using FixedTrajectory = drone_node::msg::FixedTrajectory;

    const uint8_t direction = msg.point_params.yaw_direction;

    if (direction != FixedTrajectory::SHORTEST &&
        direction != FixedTrajectory::CW &&
        direction != FixedTrajectory::CCW) {
        reason = "invalid yaw_direction";
        return AcceptResult::REJECTED;
    }

    try {
        // Construction validates position and yaw before replacement.
        std::unique_ptr<SamplerBase> new_sampler =
            std::make_unique<trajectory_sampler::FixedSampler>(
                msg.point_params);

        reason = "accepted";

        // Commit only after all validation and construction succeed.
        sampler_ = std::move(new_sampler);
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

bool TrajectoryManager::is_started(
    const rclcpp::Time& now) const
{
    return has_trajectory_ && now >= start_time_;
}

double TrajectoryManager::elapsed(
    const rclcpp::Time& now) const
{
    if (!has_trajectory_) {
        throw std::logic_error("No active trajectory");
    }

    return (now - start_time_).seconds();
}

ReferenceState TrajectoryManager::sample(
    const rclcpp::Time& now) const
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error("No active sampler");
    }

    if (!is_started(now)) {
        throw std::logic_error("Trajectory has not started");
    }

    return sampler_->sample(elapsed(now));
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
    has_trajectory_ = false;
}

}  // namespace drone_node