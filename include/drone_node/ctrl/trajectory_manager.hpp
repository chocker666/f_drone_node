#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/time.hpp>

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/ctrl/algorithm/pid_controller.hpp"
#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"
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

    AcceptResult accept(
        const ControlTrajectory& msg,
        const rclcpp::Time& now,
        std::string& reason);

    bool has_trajectory() const noexcept;

    bool is_started(const rclcpp::Time& now) const;

    double elapsed(const rclcpp::Time& now) const;

    ReferenceState sample(const rclcpp::Time& now) const;

    uint64_t trajectory_id() const noexcept;

    const PIDParameters& position_pid_parameters() const;
    const PIDParameters& yaw_pid_parameters() const;

    uint8_t yaw_direction() const noexcept;

    void clear() noexcept;

private:
    std::unique_ptr<SamplerBase> sampler_;

    PIDParameters position_pid_parameters_{};
    PIDParameters yaw_pid_parameters_{};

    uint8_t yaw_direction_{
        drone_node::msg::FixedTrajectory::SHORTEST};

    uint64_t trajectory_id_{0};
    uint64_t last_accepted_id_{0};

    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    bool has_trajectory_{false};
};

}  // namespace drone_node