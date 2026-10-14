
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/time.hpp>

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/algorithm/pid_controller.hpp"
#include "drone_node/trajectory_sampler/trajectory_sampler_base.hpp"
#include "drone_node/types.hpp"

namespace drone_node
{

class TrajectoryManager
{
public:
    using ControlTrajectory = drone_node::msg::ControlTrajectory;
    using PIDParameters = drone_node::algorithm::PIDParameters;
    using SamplerBase =
        drone_node::trajectory_sampler::TrajectorySamplerBase;

    enum class AcceptResult
    {
        ACCEPTED,
        DUPLICATE,
        REJECTED
    };

    // Strong guarantee: invalid trajectories do not replace
    // the currently active trajectory.
    AcceptResult accept(
        const ControlTrajectory& msg,
        const rclcpp::Time& now,
        std::string& reason);

    bool has_trajectory() const noexcept;

    bool is_started(const rclcpp::Time& now) const;

    double elapsed(const rclcpp::Time& now) const;

    ReferenceState sample(const rclcpp::Time& now) const;

    uint64_t trajectory_id() const noexcept;

    const PIDParameters& pid_parameters() const;

    void clear() noexcept;

private:
    std::unique_ptr<SamplerBase> sampler_;

    PIDParameters pid_parameters_{};

    uint64_t trajectory_id_{0};

    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    bool has_trajectory_{false};
};

}  // namespace drone_node
