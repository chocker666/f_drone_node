
#include "drone_node/trajectory_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "drone_node/trajectory_sampler/fixed_sampler.hpp"

namespace drone_node
{

TrajectoryManager::AcceptResult TrajectoryManager::accept(
    const ControlTrajectory& msg,
    const rclcpp::Time& now,
    std::string& reason)
{
    reason.clear();

    if (msg.trajectory_id == 0) {
        reason = "trajectory_id must be nonzero";
        return AcceptResult::REJECTED;
    }

    if (has_trajectory_ &&
        msg.trajectory_id == trajectory_id_) {
        reason = "duplicate trajectory_id";
        return AcceptResult::DUPLICATE;
    }

    if (has_trajectory_ &&
        msg.trajectory_id < trajectory_id_) {
        reason = "outdated trajectory_id";
        return AcceptResult::REJECTED;
    }

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

    PIDParameters params;
    params.kp = msg.pid_params.kp;
    params.ki = msg.pid_params.ki;
    params.kd = msg.pid_params.kd;
    params.max_speed = msg.pid_params.max_speed;
    params.max_acceleration =
        msg.pid_params.max_acceleration;
    params.integral_limit =
        msg.pid_params.integral_limit;

    // Validate all parameters before replacing the
    // active sampler.
    const auto nonnegative = [](double x) {
        return std::isfinite(x) && x >= 0.0;
    };

    const auto positive = [](double x) {
        return std::isfinite(x) && x > 0.0;
    };

    if (!nonnegative(params.kp) ||
        !nonnegative(params.ki) ||
        !nonnegative(params.kd) ||
        !positive(params.max_speed) ||
        !positive(params.max_acceleration) ||
        !positive(params.integral_limit)) {
        reason = "invalid PID parameters";
        return AcceptResult::REJECTED;
    }

    rclcpp::Time new_start_time(
        msg.start_time, now.get_clock_type());

    // Zero time means "start immediately".
    // This is a deliberate convenience for manual testing.
    if (new_start_time.nanoseconds() == 0) {
        new_start_time = now;
    }

    try {
        std::unique_ptr<SamplerBase> new_sampler =
            std::make_unique<
                drone_node::trajectory_sampler::FixedSampler>(
                    msg.fixed_params);

        // Commit only after validation and construction.
        sampler_ = std::move(new_sampler);
        pid_parameters_ = params;
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

const TrajectoryManager::PIDParameters&
TrajectoryManager::pid_parameters() const
{
    if (!has_trajectory_) {
        throw std::logic_error("No active trajectory");
    }

    return pid_parameters_;
}

void TrajectoryManager::clear() noexcept
{
    sampler_.reset();
    has_trajectory_ = false;
    // Retain the last ID for diagnostics.
}

}  // namespace drone_node
