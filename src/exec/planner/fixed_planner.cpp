#include "drone_node/exec/planner/fixed_planner.hpp"

#include <cmath>

namespace drone_node::exec
{

PlanningResult FixedPlanner::plan(
    const PlanningRequest& request)
{
    PlanningResult result;

    if (request.planner_type != PlannerType::FIXED) {
        result.status = PlanningStatus::UNSUPPORTED;
        result.reason = "FixedPlanner requires FIXED planner type";
        return result;
    }

    if (request.frame_id != "map") {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "FixedPlanner requires map/ENU coordinates";
        return result;
    }

    if (!request.target_position.allFinite() ||
        !std::isfinite(request.target_yaw)) {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Target position and yaw must be finite";
        return result;
    }

    if (request.yaw_direction > 2) {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Invalid yaw direction";
        return result;
    }

    if (request.controller_id.empty() ||
        request.config_name.empty()) {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Controller ID and config name must not be empty";
        return result;
    }

    if (request.controller_id != "position_yaw_pid") {
        result.status = PlanningStatus::UNSUPPORTED;
        result.reason = "Only position_yaw_pid is currently supported";
        return result;
    }

    auto& motion = result.motion;

    motion.planner_type = PlannerType::FIXED;
    motion.motion_type = PlannedMotionType::POINT;
    motion.frame_id = request.frame_id;
    motion.target_position = request.target_position;
    motion.target_yaw = request.target_yaw;
    motion.yaw_direction = request.yaw_direction;
    motion.controller_id = request.controller_id;
    motion.config_name = request.config_name;

    result.status = PlanningStatus::SUCCESS;
    result.reason = "Fixed POINT motion planned";

    return result;
}

}  // namespace drone_node::exec