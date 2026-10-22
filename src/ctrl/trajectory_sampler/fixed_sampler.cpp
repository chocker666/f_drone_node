#include "drone_node/ctrl/trajectory_sampler/fixed_sampler.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_node::trajectory_sampler
{

PointSampler::PointSampler(
    const drone_node::msg::FixedTrajectory& trajectory)
{
    using Point = drone_node::msg::FixedTrajectory;

    const auto& p = trajectory.position;

    if (!std::isfinite(p.x) ||
        !std::isfinite(p.y) ||
        !std::isfinite(p.z) ||
        !std::isfinite(trajectory.yaw)) {
        throw std::invalid_argument(
            "PointSampler: non-finite reference");
    }

    if (trajectory.yaw_direction != Point::SHORTEST &&
        trajectory.yaw_direction != Point::CW &&
        trajectory.yaw_direction != Point::CCW) {
        throw std::invalid_argument(
            "PointSampler: invalid yaw direction");
    }

    reference_.position = Eigen::Vector3d(p.x, p.y, p.z);
    reference_.velocity.setZero();
    reference_.acceleration.setZero();
    reference_.yaw = trajectory.yaw;
    reference_.yaw_rate = 0.0;
}

ReferenceState PointSampler::sample(double t) const
{
    if (!std::isfinite(t) || t < 0.0) {
        throw std::invalid_argument(
            "PointSampler: invalid sample time");
    }

    // No interpolation or smoothing of a new POINT target.
    return reference_;
}

double PointSampler::duration() const
{
    return std::numeric_limits<double>::infinity();
}

}  // namespace drone_node::trajectory_sampler