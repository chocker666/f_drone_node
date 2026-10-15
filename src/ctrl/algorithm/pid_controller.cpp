
#include "drone_node/ctrl/algorithm/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_node::algorithm
{

namespace
{

Eigen::Vector3d limit_norm(
    const Eigen::Vector3d & vector,
    double maximum_norm)
{
    const double norm = vector.norm();

    if (norm <= maximum_norm || norm <= 0.0) {
        return vector;
    }

    return vector * (maximum_norm / norm);
}

}  // namespace

PIDController::PIDController(
    const PIDParameters& parameters)
{
    set_parameters(parameters);
}

void PIDController::set_parameters(
    const PIDParameters& parameters)
{
    const auto nonnegative = [](double value) {
        return std::isfinite(value) && value >= 0.0;
    };

    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };

    if (!nonnegative(parameters.kp) ||
        !nonnegative(parameters.ki) ||
        !nonnegative(parameters.kd) ||
        !positive(parameters.max_speed) ||
        !positive(parameters.max_acceleration) ||
        !positive(parameters.integral_limit)) {
        throw std::invalid_argument(
            "Invalid PID controller parameters");
    }

    parameters_ = parameters;
    reset();
}

void PIDController::reset()
{
    integral_error_.setZero();
    previous_velocity_command_.setZero();
}

ControllerOutput PIDController::update(
    const ControllerInput & input)
{
    if (!std::isfinite(input.dt) ||
        input.dt <= 0.0 ||
        !input.position.allFinite() ||
        !input.velocity.allFinite() ||
        !input.target_position.allFinite()) {
        throw std::invalid_argument(
            "Invalid PID controller input");
    }

    const Eigen::Vector3d error =
        input.target_position - input.position;

    // Integrate position error with a norm limit.
    // Freeze integration when the previous command
    // was saturated in the direction of the error.
    const Eigen::Vector3d previous_unsaturated =
        parameters_.kp * error +
        parameters_.ki * integral_error_ -
        parameters_.kd * input.velocity;

    const bool saturated_outward =
        previous_unsaturated.norm() >
            parameters_.max_speed &&
        previous_unsaturated.dot(error) > 0.0;

    if (!saturated_outward) {
        integral_error_ += error * input.dt;
        integral_error_ = limit_norm(
            integral_error_,
            parameters_.integral_limit);
    }

    const Eigen::Vector3d raw_velocity =
        parameters_.kp * error +
        parameters_.ki * integral_error_ -
        parameters_.kd * input.velocity;

    const Eigen::Vector3d limited_velocity =
        limit_norm(raw_velocity, parameters_.max_speed);

    // Rate-limit velocity setpoint changes.
    const Eigen::Vector3d delta_velocity =
        limited_velocity - previous_velocity_command_;

    const Eigen::Vector3d limited_delta =
        limit_norm(
            delta_velocity,
            parameters_.max_acceleration * input.dt);

    const Eigen::Vector3d velocity_command =
        previous_velocity_command_ + limited_delta;

    previous_velocity_command_ = velocity_command;

    ControllerOutput output;
    output.velocity_setpoint = velocity_command;
    return output;
}

}  // namespace drone_node::algorithm
