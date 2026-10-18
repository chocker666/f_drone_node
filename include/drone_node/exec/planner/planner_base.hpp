#pragma once

#include "drone_node/exec/planner/planner_types.hpp"

namespace drone_node::exec
{

class PlannerBase
{
public:
    virtual ~PlannerBase() = default;

    virtual PlanningResult plan(
        const PlanningRequest& request) = 0;
};

}  // namespace drone_node::exec