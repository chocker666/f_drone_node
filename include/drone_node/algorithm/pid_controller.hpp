
#pragma once

#include "drone_node/algorithm/controller_base.hpp"

namespace drone_node::algorithm
{

struct PIDParameters
{
    double kp{0.8};
    double ki{0.0};
    double kd{0.0};

    double max_speed{1.0};
    double max_acceleration{1.0};
    double integral_limit{1.0};
};

class PIDController final : public ControllerBase
{
public:
    explicit PIDController(const PIDParameters & parameters);

    void reset() override;

    ControllerOutput update(
        const ControllerInput & input) override;

private:
    PIDParameters parameters_;

    Eigen::Vector3d integral_error_{
        Eigen::Vector3d::Zero()};

    Eigen::Vector3d previous_velocity_command_{
        Eigen::Vector3d::Zero()};
};

}  // namespace drone_node::algorithm
