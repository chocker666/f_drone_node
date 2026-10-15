
#pragma once

#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"
#include "drone_node/msg/fixed_trajectory.hpp"

namespace drone_node::trajectory_sampler
{

class FixedSampler final : public TrajectorySamplerBase
{
public:
    explicit FixedSampler(
        const drone_node::msg::FixedTrajectory& trajectory);

    ReferenceState sample(double t) const override;

    double duration() const override;

private:
    ReferenceState reference_{};
};

}  // namespace drone_node::trajectory_sampler
