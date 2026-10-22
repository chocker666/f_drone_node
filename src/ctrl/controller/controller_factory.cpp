#include "drone_node/ctrl/controller/controller_factory.hpp"

#include <stdexcept>

namespace drone_node::controller
{

std::unique_ptr<ControllerBase> ControllerFactory::create(
    const std::string& controller_id,
    const PositionYawPIDConfig& config)
{
    if (controller_id == "position_yaw_pid") {
        return std::make_unique<PositionYawPIDController>(config);
    }

    throw std::invalid_argument(
        "Unsupported physical controller: " + controller_id);
}

}  // namespace drone_node::controller