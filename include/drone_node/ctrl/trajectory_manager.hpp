
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/ctrl/trajectory_sampler/discrete_sampler.hpp"
#include "drone_node/ctrl/trajectory_sampler/polynomial_sampler.hpp"
#include "drone_node/types.hpp"

namespace drone_node
{

class TrajectoryManager
{
public:
    using ControlTrajectory = drone_node::msg::ControlTrajectory;

    using Sampler = std::variant<
        trajectory_sampler::DiscreteSampler,
        trajectory_sampler::PolynomialSampler>;

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

    // Validate and initialize a new trajectory.
    // On success, replace the current trajectory immediately.
    // On failure, keep the current trajectory unchanged.
    //
    // trajectory_id is only an identifier and is not validated.
    // frame_id is validated but not stored.
    AcceptResult accept(
        const ControlTrajectory& msg,
        SteadyTime now,
        std::string& reason);

    // Whether an active trajectory exists.
    bool has_trajectory() const noexcept;

    // Whether the trajectory has reached its start time.
    bool is_started(SteadyTime now) const noexcept;

    // Elapsed time since trajectory activation [s].
    double elapsed(SteadyTime now) const;

    // Sample the reference state at the given time.
    ReferenceState sample(SteadyTime now) const;

    // Sample reference state and validity flags.
    SampleResult sample_with_validity(SteadyTime now) const;

    // Duration of the active trajectory [s].
    double duration() const;

    // Current trajectory identifier.
    // All uint64_t values, including 0, are valid.
    uint64_t trajectory_id() const noexcept;

    // Cancel and remove the current trajectory.
    void clear() noexcept;

    // Reset the trajectory time origin without
    // replacing the sampler or trajectory ID.
    void activate(SteadyTime now);

private:
    // Exactly one active sampler, or no trajectory.
    std::optional<Sampler> sampler_;

    // Identifier of the current trajectory.
    uint64_t trajectory_id_{0};

    // Activation time of the current trajectory.
    SteadyTime start_time_{};
};

}  // namespace drone_node
