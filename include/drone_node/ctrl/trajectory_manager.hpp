
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/ctrl/trajectory_sampler/polynomial_sampler.hpp"
#include "drone_node/types.hpp"

namespace drone_node
{

class TrajectoryManager
{
public:
    using ControlTrajectory = drone_node::msg::ControlTrajectory;
    using Sampler = trajectory_sampler::PolynomialSampler;

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

    // Fully validated candidate trajectory.
    // Does not affect the currently active trajectory.
    struct PreparedTrajectory
    {
        uint64_t trajectory_id{0};
        std::unique_ptr<const Sampler> sampler;

        PreparedTrajectory(
            uint64_t id,
            std::unique_ptr<const Sampler> ptr)
            : trajectory_id(id),
              sampler(std::move(ptr))
        {
        }

        PreparedTrajectory(PreparedTrajectory&&) = default;
        PreparedTrajectory& operator=(PreparedTrajectory&&) = default;

        PreparedTrajectory(const PreparedTrajectory&) = delete;
        PreparedTrajectory& operator=(const PreparedTrajectory&) = delete;
    };

    // Validate and construct a candidate.
    // Never modifies the active trajectory.
    PreparedTrajectory prepare(
        const ControlTrajectory& msg) const;

    // Activate an already prepared trajectory.
    // Intended to be coordinated with controller activation.
    // Does not allocate or rebuild the sampler.
    void commit(
        PreparedTrajectory&& prepared,
        SteadyTime now) noexcept;

    bool has_trajectory() const noexcept;

    bool is_started(SteadyTime now) const noexcept;

    double elapsed(SteadyTime now) const;

    ReferenceState sample(SteadyTime now) const;

    SampleResult sample_with_validity(
        SteadyTime now) const;

    double duration() const;

    uint64_t trajectory_id() const noexcept;

    void clear() noexcept;

    void activate(SteadyTime now);

private:
    std::unique_ptr<const Sampler> sampler_;

    uint64_t trajectory_id_{0};

    SteadyTime start_time_{};
};

}  // namespace drone_node
