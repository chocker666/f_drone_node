
#pragma once

#include <Eigen/Core>

namespace drone_node::algorithm
{

struct ControllerInput
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
    double dt{0.02};
};

struct ControllerOutput
{
    Eigen::Vector3d velocity_setpoint{Eigen::Vector3d::Zero()};
};

class ControllerBase
{
public:
    virtual ~ControllerBase() = default;

    virtual void reset() = 0;

    virtual ControllerOutput update(
        const ControllerInput & input) = 0;
};

}  // namespace drone_node::algorithm
