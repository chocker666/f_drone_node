#pragma once

#include <memory>

#include "drone_node/exec/planner/fixed_planner.hpp"

namespace drone_node::exec
{

class MotionPlanner
{
public:
    MotionPlanner();

    PlanningResult plan(
        const PlanningRequest& request);

private:
    std::unique_ptr<FixedPlanner> fixed_planner_;
};

}  // namespace drone_node::exec