#pragma once

#include <memory>
#include <string>

#include "drone_node/ctrl/controller/position_yaw_pid_controller.hpp"

namespace drone_node::controller
{

class ControllerFactory
{
public:
    // Typed overload for PID configuration.
    // Future algorithms receive their own configuration overloads.
    static std::unique_ptr<ControllerBase> create(
        const std::string& controller_id,
        const PositionYawPIDConfig& config);
};

}  // namespace drone_node::controller