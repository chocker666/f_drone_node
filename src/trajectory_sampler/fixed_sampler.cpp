
#include "drone_node/trajectory_sampler/fixed_sampler.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_node::trajectory_sampler
{

FixedSampler::FixedSampler(
    const drone_node::msg::FixedTrajectory& trajectory)
{
    const auto& p = trajectory.position;

    if (!std::isfinite(p.x) ||
        !std::isfinite(p.y) ||
        !std::isfinite(p.z) ||
        !std::isfinite(trajectory.yaw)) {
        throw std::invalid_argument(
            "FixedSampler: invalid fixed reference");
    }

    reference_.position = Eigen::Vector3d(
        p.x, p.y, p.z);

    reference_.velocity.setZero();
    reference_.acceleration.setZero();

    reference_.yaw = trajectory.yaw;
    reference_.yaw_rate = 0.0;
}

ReferenceState FixedSampler::sample(double t) const
{
    if (!std::isfinite(t) || t < 0.0) {
        throw std::invalid_argument(
            "FixedSampler: invalid sample time");
    }

    return reference_;
}

double FixedSampler::duration() const
{
    return std::numeric_limits<double>::infinity();
}

}  // namespace drone_node::trajectory_sampler
