#pragma once

#include "drone_node/exec/planner/planner_base.hpp"

namespace drone_node::exec
{

class FixedPlanner final : public PlannerBase
{
public:
    PlanningResult plan(
        const PlanningRequest& request) override;
};

}  // namespace drone_node::exec