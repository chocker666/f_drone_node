#pragma once

#include <cstdint>
#include <string>

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/exec/planner/quintic_planner.hpp"

namespace drone_node::exec
{

enum class PlannerType : std::uint8_t
{
    QUINTIC = 0,
    EXTERNAL = 1
};

enum class PlanningStatus : std::uint8_t
{
    SUCCESS,
    INVALID_REQUEST,
    UNSUPPORTED,
    FAILED
};

struct PlanningRequest
{
    using BoundaryState =
        planner::QuinticPlanner::BoundaryState;

    using Limits =
        planner::QuinticPlanner::Limits;

    using YawDirection =
        planner::QuinticPlanner::YawDirection;

    PlannerType planner_type{PlannerType::QUINTIC};

    BoundaryState start;
    BoundaryState goal;

    double duration{0.0};

    YawDirection yaw_direction{
        YawDirection::SHORTEST
    };

    Limits limits;

    std::uint64_t trajectory_id{0};

    std::string frame_id{"map"};

    std::string controller_id{"position_yaw_pid"};
    std::string config_name{"default"};
};

struct PlanningResult
{
    PlanningStatus status{PlanningStatus::FAILED};

    std::string reason;

    drone_node::msg::ControlTrajectory trajectory;

    bool success() const noexcept
    {
        return status == PlanningStatus::SUCCESS;
    }
};

}  // namespace drone_node::exec