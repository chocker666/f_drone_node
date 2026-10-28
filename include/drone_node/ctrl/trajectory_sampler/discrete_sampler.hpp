
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "drone_node/types.hpp"
#include "drone_node/msg/discrete_traj.hpp"
#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"

namespace drone_node::trajectory_sampler
{

class DiscreteSampler final : public TrajectorySamplerBase
{
public:
    // Reference state together with its validity flags.
    struct SampleResult
    {
        ReferenceState state{};
        ReferenceValidity validity{};
    };

    // Validate and preprocess the complete discrete trajectory.
    // All preprocessing is performed during construction.
    explicit DiscreteSampler(
        const drone_node::msg::DiscreteTraj& trajectory);

    ~DiscreteSampler() override = default;

    // Legacy-compatible sampling interface.
    // Returns the reference state without validity flags.
    ReferenceState sample(double t) const override;

    // Extended sampling interface.
    // Returns both reference state and validity flags.
    SampleResult sample_with_validity(double t) const;

    // Total trajectory duration in seconds.
    double duration() const override;

private:
    // Preprocessed trajectory node.
    struct Node
    {
        // Trajectory-relative time in seconds.
        double time{0.0};

        // Mandatory reference components.
        Eigen::Vector3d position{
            Eigen::Vector3d::Zero()};
        double yaw{0.0};

        // Optional reference components.
        // Missing values are reconstructed during initialization.
        Eigen::Vector3d velocity{
            Eigen::Vector3d::Zero()};
        Eigen::Vector3d acceleration{
            Eigen::Vector3d::Zero()};
        double yaw_rate{0.0};

        // Indicates whether the corresponding derivative
        // is available after preprocessing.
        bool velocity_valid{false};
        bool acceleration_valid{false};
        bool yaw_rate_valid{false};
    };

    // Preprocessed nodes ordered by strictly increasing time.
    // Immutable after construction.
    std::vector<Node> nodes_;

    // Duration measured from the first trajectory node.
    double duration_{0.0};
};

}  // namespace drone_node::trajectory_sampler
