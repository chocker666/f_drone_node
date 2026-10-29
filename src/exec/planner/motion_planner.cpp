#include "drone_node/exec/planner/motion_planner.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace drone_node::exec
{

PlanningResult MotionPlanner::plan(
    const PlanningRequest& request)
{
    if (request.planner_type == PlannerType::EXTERNAL) {
        return {
            PlanningStatus::UNSUPPORTED,
            "External planner is not implemented",
            {}
        };
    }

    if (request.planner_type != PlannerType::QUINTIC) {
        return {
            PlanningStatus::UNSUPPORTED,
            "Unknown planner type",
            {}
        };
    }

    try {
        planner::QuinticPlanner::Request quintic_request;

        quintic_request.start =
            request.start;

        quintic_request.goal =
            request.goal;

        quintic_request.duration =
            request.duration;

        quintic_request.yaw_direction =
            request.yaw_direction;

        quintic_request.limits =
            request.limits;

        quintic_request.trajectory_id =
            request.trajectory_id;

        quintic_request.frame_id =
            request.frame_id;

        quintic_request.controller_id =
            request.controller_id;

        quintic_request.config_name =
            request.config_name;

        auto trajectory =
            planner::QuinticPlanner::plan(quintic_request);

        return {
            PlanningStatus::SUCCESS,
            "",
            std::move(trajectory)
        };

    } catch (const std::invalid_argument& e) {
        return {
            PlanningStatus::INVALID_REQUEST,
            e.what(),
            {}
        };

    } catch (const std::exception& e) {
        return {
            PlanningStatus::FAILED,
            e.what(),
            {}
        };
    }
}

}  // namespace drone_node::exec