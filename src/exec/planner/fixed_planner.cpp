#include "drone_node/exec/planner/fixed_planner.hpp"

#include <cmath>

namespace drone_node::exec
{

PlanningResult FixedPlanner::plan(
    const PlanningRequest& request)
{
    PlanningResult result;

    if (request.action_id == 0) {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Invalid ActionId";
        return result;
    }

    if (request.frame_id != "map") {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Unsupported frame_id";
        return result;
    }

    if (!request.target_position.allFinite() ||
        !std::isfinite(request.target_yaw)) {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Non-finite target";
        return result;
    }

    if (request.yaw_direction > 2) {
        result.status = PlanningStatus::INVALID_REQUEST;
        result.reason = "Invalid yaw direction";
        return result;
    }

    result.motion.action_id = request.action_id;
    result.motion.planner_type = PlannerType::FIXED;
    result.motion.frame_id = request.frame_id;
    result.motion.target_position = request.target_position;
    result.motion.target_yaw = request.target_yaw;
    result.motion.yaw_direction = request.yaw_direction;

    result.status = PlanningStatus::SUCCESS;
    result.reason = "Fixed trajectory planned";

    return result;
}

}  // namespace drone_node::exec