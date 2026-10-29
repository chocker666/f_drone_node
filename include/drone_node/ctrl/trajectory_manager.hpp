
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "drone_node/msg/discrete_traj.hpp"
#include "drone_node/msg/polynomial_traj.hpp"
#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"
#include "drone_node/types.hpp"

namespace drone_node
{

class TrajectoryManager
{
public:
    using DiscreteTraj = drone_node::msg::DiscreteTraj;
    using PolynomialTraj = drone_node::msg::PolynomialTraj;
    using SamplerBase = trajectory_sampler::TrajectorySamplerBase;

    enum class AcceptResult
    {
        ACCEPTED,
        REJECTED
    };

    struct SampleResult
    {
        ReferenceState state{};
        ReferenceValidity validity{};
    };

    // Validate and prepare a new discrete trajectory.
    // On failure, the active and previously prepared
    // trajectories remain unchanged.
    AcceptResult prepare(
        const DiscreteTraj& msg,
        std::string& reason);

    // Validate and prepare a new polynomial trajectory.
    // On failure, the active and previously prepared
    // trajectories remain unchanged.
    AcceptResult prepare(
        const PolynomialTraj& msg,
        std::string& reason);

    // Commit the prepared trajectory as the active one.
    // The supplied time becomes the trajectory time origin.
    // Returns false if no prepared trajectory exists.
    bool commit(SteadyTime now) noexcept;

    // Discard only the prepared trajectory.
    // The active trajectory is not affected.
    void discard_prepared() noexcept;

    bool has_prepared() const noexcept;

    // Active trajectory state.
    bool has_trajectory() const noexcept;

    bool is_started(SteadyTime now) const noexcept;

    // Elapsed time since activation, in seconds.
    // Throws if no trajectory is active or now is
    // earlier than its activation time.
    double elapsed(SteadyTime now) const;

    // Legacy-compatible sampling interface.
    ReferenceState sample(SteadyTime now) const;

    // Sample the reference state and validity flags.
    SampleResult sample_with_validity(SteadyTime now) const;

    // Duration of the active trajectory, in seconds.
    double duration() const;

    uint64_t trajectory_id() const noexcept;

    // Clear both active and prepared trajectories.
    void clear() noexcept;

    // Rebase the active trajectory's time origin.
    // Does not replace its sampler or trajectory ID.
    void activate(SteadyTime now);

private:
    enum class SamplerKind
    {
        NONE,
        DISCRETE,
        POLYNOMIAL
    };

    // Active trajectory.
    std::unique_ptr<SamplerBase> sampler_;
    SamplerKind sampler_kind_{SamplerKind::NONE};

    uint64_t trajectory_id_{0};
    SteadyTime start_time_{};
    bool has_trajectory_{false};

    // Validated candidate awaiting commit.
    std::unique_ptr<SamplerBase> prepared_sampler_;
    SamplerKind prepared_kind_{SamplerKind::NONE};
    uint64_t prepared_trajectory_id_{0};
};

}  // namespace drone_node
