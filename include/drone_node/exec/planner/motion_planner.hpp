#pragma once

#include "drone_node/exec/planner/planner_types.hpp"
#include "drone_node/exec/planner/quintic_planner.hpp"

namespace drone_node::exec
{

class MotionPlanner
{
public:
    PlanningResult plan(
        const PlanningRequest& request);
};

}  // namespace drone_node::exec