
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

    if (msg.trajectory_type != ControlTrajectory::FIXED) {
        reason = "unsupported trajectory_type";
        return AcceptResult::REJECTED;
    }

    if (msg.controller_type != ControlTrajectory::PID) {
        reason = "unsupported controller_type";
        return AcceptResult::REJECTED;
    }

    const uint8_t direction =
        msg.fixed_params.yaw_direction;

    using FixedTrajectory = drone_node::msg::FixedTrajectory;

    if (direction != FixedTrajectory::SHORTEST &&
        direction != FixedTrajectory::CW &&
        direction != FixedTrajectory::CCW) {
        reason = "invalid yaw_direction";
        return AcceptResult::REJECTED;
    }

    rclcpp::Time new_start_time(
        msg.start_time, now.get_clock_type());

    if (new_start_time.nanoseconds() == 0) {
        new_start_time = now;
    }

    try {
        std::unique_ptr<SamplerBase> new_sampler =
            std::make_unique<
                trajectory_sampler::FixedSampler>(
                    msg.fixed_params);

        // Commit only after validation.
        sampler_ = std::move(new_sampler);

        yaw_direction_ = direction;

        start_time_ = new_start_time;
        trajectory_id_ = msg.trajectory_id;

        has_trajectory_ = true;

        reason = "accepted";
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
        throw std::logic_error(
            "Trajectory has not started");
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
