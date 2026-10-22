#pragma once

#include "drone_node/ctrl/algorithm/pid_controller.hpp"
#include "drone_node/ctrl/controller/controller_base.hpp"

namespace drone_node::controller
{

struct PositionYawPIDConfig
{
    algorithm::PIDParameters position{};

    algorithm::PIDParameters yaw = [] {
        algorithm::PIDParameters parameters;
        parameters.limit_mode = algorithm::PIDLimitMode::PER_AXIS;
        return parameters;
    }();
};

class PositionYawPIDController final : public ControllerBase
{
public:
    explicit PositionYawPIDController(
        const PositionYawPIDConfig& config);

    void reset() override;

    void initialize(
        const PhysicalControllerInput& input,
        YawDirection direction) override;

    ControllerResult update(
        const PhysicalControllerInput& input) override;

private:
    static void validate_input(
        const PhysicalControllerInput& input);

    static double select_yaw_delta(
        double current_yaw,
        double target_yaw,
        YawDirection direction);

    algorithm::PIDController<3> position_pid_;
    algorithm::PIDController<1> yaw_pid_;

    bool initialized_{false};
    double initial_reference_yaw_{0.0};
    double initial_target_yaw_{0.0};
};

}  // namespace drone_node::controller