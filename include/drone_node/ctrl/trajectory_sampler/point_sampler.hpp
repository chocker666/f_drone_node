#pragma once

#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"
#include "drone_node/msg/fixed_trajectory.hpp"

namespace drone_node::trajectory_sampler
{

// POINT sampler: maintains a fixed position/yaw reference indefinitely.
// FixedTrajectory is retained as the ROS message payload type.
class PointSampler final : public TrajectorySamplerBase
{
public:
    explicit PointSampler(
        const drone_node::msg::FixedTrajectory& trajectory);

    ReferenceState sample(double t) const override;

    double duration() const override;

private:
    ReferenceState reference_{};
};

}  // namespace drone_node::trajectory_sampler