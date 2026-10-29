
#include "drone_node/ctrl/trajectory_manager.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace drone_node
{

TrajectoryManager::PreparedTrajectory
TrajectoryManager::prepare(
    const ControlTrajectory& msg) const
{
    if (msg.trajectory_id == 0) {
        throw std::invalid_argument(
            "TrajectoryManager: trajectory_id must be non-zero");
    }

    // CTRL currently accepts only the "map" frame.
    // Trajectory coordinates must be expressed in the
    // same local ENU frame as the vehicle state feedback.
    // No coordinate transformation is performed here.
    if (msg.frame_id != "map") {
        throw std::invalid_argument(
            "TrajectoryManager: unsupported frame_id");
    }

    // PolynomialSampler performs detailed trajectory validation:
    // segment count, array sizes, finite coefficients,
    // durations and boundary continuity.
    auto candidate = std::make_unique<const Sampler>(msg);

    return PreparedTrajectory{
        msg.trajectory_id,
        std::move(candidate)
    };
}

void TrajectoryManager::commit(
    PreparedTrajectory&& prepared,
    SteadyTime now) noexcept
{
    // Preconditions:
    // - prepared.sampler is non-null.
    // - physical controller preparation has succeeded.
    // - FlightController serializes commit with control updates.
    //
    // Commit the prepared sampler, trajectory ID and start time.
    // Physical controller switching is handled by FlightController.
    // No validation or allocation occurs here.
    sampler_ = std::move(prepared.sampler);
    trajectory_id_ = prepared.trajectory_id;
    start_time_ = now;
}

bool TrajectoryManager::has_trajectory() const noexcept
{
    return static_cast<bool>(sampler_);
}

double TrajectoryManager::elapsed(
    SteadyTime now) const
{
    if (!sampler_) {
        throw std::logic_error(
            "TrajectoryManager: no active trajectory");
    }

    if (now < start_time_) {
        throw std::invalid_argument(
            "TrajectoryManager: time precedes activation");
    }

    return std::chrono::duration<double>(
        now - start_time_).count();
}

ReferenceSample
TrajectoryManager::sample_with_validity(
    SteadyTime now) const
{
    if (!sampler_) {
        throw std::logic_error(
            "TrajectoryManager: no active sampler");
    }

    return sampler_->sample_with_validity(elapsed(now));
}

double TrajectoryManager::duration() const
{
    if (!sampler_) {
        throw std::logic_error(
            "TrajectoryManager: no active sampler");
    }

    return sampler_->duration();
}

uint64_t TrajectoryManager::trajectory_id() const noexcept
{
    return trajectory_id_;
}

void TrajectoryManager::clear() noexcept
{
    sampler_.reset();
    trajectory_id_ = 0;
    start_time_ = SteadyTime{};
}

}  // namespace drone_node
