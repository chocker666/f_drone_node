#include "drone_node/exec/planner/motion_planner.hpp"

namespace drone_node::exec
{

PlanningResult MotionPlanner::plan(
    const PlanningRequest& request)
{
    switch (request.planner_type) {
    case PlannerType::FIXED:
        return fixed_planner_.plan(request);

    case PlannerType::EXTERNAL:
        return {
            PlanningStatus::UNSUPPORTED,
            "External planner is not implemented",
            {}
        };
    }

    return {
        PlanningStatus::UNSUPPORTED,
        "Unknown planner type",
        {}
    };
}

}  // namespace drone_node::exec