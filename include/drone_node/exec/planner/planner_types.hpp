#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Core>

#include "drone_node/exec/action_manager.hpp"

namespace drone_node::exec
{

enum class PlannerType : uint8_t
{
    FIXED = 0,
    EXTERNAL = 1
};

enum class PlanningStatus : uint8_t
{
    SUCCESS,
    INVALID_REQUEST,
    UNSUPPORTED,
    FAILED
};

struct PlanningRequest
{
    ActionId action_id{0};

    PlannerType planner_type{PlannerType::FIXED};

    std::string frame_id{"map"};

    Eigen::Vector3d target_position{
        Eigen::Vector3d::Zero()};

    double target_yaw{0.0};

    uint8_t yaw_direction{0};
};

struct PlannedMotion
{
    ActionId action_id{0};

    PlannerType planner_type{PlannerType::FIXED};

    std::string frame_id{"map"};

    Eigen::Vector3d target_position{
        Eigen::Vector3d::Zero()};

    double target_yaw{0.0};

    uint8_t yaw_direction{0};
};

struct PlanningResult
{
    PlanningStatus status{PlanningStatus::FAILED};

    std::string reason;

    PlannedMotion motion;

    bool success() const
    {
        return status == PlanningStatus::SUCCESS;
    }
};

}  // namespace drone_node::exec