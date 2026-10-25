#pragma once

#include "drone_node/exec/planner/fixed_planner.hpp"

namespace drone_node::exec
{

class MotionPlanner
{
public:
    PlanningResult plan(
        const PlanningRequest& request);

private:
    FixedPlanner fixed_planner_;
};

}  // namespace drone_node::exec