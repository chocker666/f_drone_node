#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/msg/fixed_trajectory.hpp"
#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"
#include "drone_node/types.hpp"

namespace drone_node
{

class TrajectoryManager
{
public:
    using ControlTrajectory = drone_node::msg::ControlTrajectory;
    using SamplerBase = trajectory_sampler::TrajectorySamplerBase;

    enum class AcceptResult
    {
        ACCEPTED,
        REJECTED
    };

    // The caller supplies a monotonic activation time.
    // A rejected trajectory leaves the active trajectory unchanged.
    AcceptResult accept(
        const ControlTrajectory& msg,
        SteadyTime now,
        std::string& reason);

    bool has_trajectory() const noexcept;

    bool is_started(SteadyTime now) const noexcept;

    double elapsed(SteadyTime now) const;

    ReferenceState sample(SteadyTime now) const;

    double duration() const;

    uint64_t trajectory_id() const noexcept;

    uint8_t yaw_direction() const noexcept;

    void clear() noexcept;

    // Rebase a prepared trajectory immediately before committing it.
    void activate(SteadyTime now);

private:
    std::unique_ptr<SamplerBase> sampler_;

    uint8_t yaw_direction_{
        drone_node::msg::FixedTrajectory::SHORTEST};

    uint64_t trajectory_id_{0};
    SteadyTime start_time_{};
    bool has_trajectory_{false};
};

}  // namespace drone_node