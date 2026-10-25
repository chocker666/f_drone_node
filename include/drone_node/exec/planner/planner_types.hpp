#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace drone_node::exec
{

enum class PlannerType : std::uint8_t
{
    FIXED = 0,
    EXTERNAL = 1
};

enum class PlanningStatus : std::uint8_t
{
    SUCCESS,
    INVALID_REQUEST,
    UNSUPPORTED,
    FAILED
};

// 规划层的轨迹类型，不依赖 ROS2 消息。
enum class PlannedMotionType : std::uint8_t
{
    POINT = 0
};

struct PlanningRequest
{
    PlannerType planner_type{PlannerType::FIXED};

    // 固定目标使用 map/ENU 坐标。
    std::string frame_id{"map"};

    Eigen::Vector3d target_position{
        Eigen::Vector3d::Zero()};

    double target_yaw{0.0};

    // 与现有 FixedTrajectory 语义一致：
    // 0 = SHORTEST，1 = CW，2 = CCW。
    std::uint8_t yaw_direction{0};

    std::string controller_id{"position_yaw_pid"};
    std::string config_name{"default"};
};

struct PlannedMotion
{
    PlannerType planner_type{PlannerType::FIXED};
    PlannedMotionType motion_type{PlannedMotionType::POINT};

    std::string frame_id{"map"};

    Eigen::Vector3d target_position{
        Eigen::Vector3d::Zero()};

    double target_yaw{0.0};
    std::uint8_t yaw_direction{0};

    std::string controller_id{"position_yaw_pid"};
    std::string config_name{"default"};
};

struct PlanningResult
{
    PlanningStatus status{PlanningStatus::FAILED};
    std::string reason;
    PlannedMotion motion;

    bool success() const noexcept
    {
        return status == PlanningStatus::SUCCESS;
    }
};

}  // namespace drone_node::exec