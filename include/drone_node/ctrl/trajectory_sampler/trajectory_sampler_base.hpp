
#pragma once

#include "drone_node/types.hpp"

namespace drone_node::trajectory_sampler
{

class TrajectorySamplerBase
{
public:
    virtual ~TrajectorySamplerBase() = default;

    // Sample the reference state at trajectory-relative time t.
    // t is expressed in seconds.
    virtual ReferenceState sample(double t) const = 0;

    // Duration in seconds.
    // Infinity indicates an indefinite trajectory.
    virtual double duration() const = 0;
};

}  // namespace drone_node::trajectory_sampler
