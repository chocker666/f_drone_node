#include "drone_node/ctrl/controller/position_yaw_pid_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_node::controller
{

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double ANGLE_EPS = 1e-10;

double wrap_pi(double angle)
{
    double result = std::remainder(angle, TWO_PI);

    if (result <= -PI) {
        result += TWO_PI;
    }

    return result;
}

}  // namespace

PositionYawPIDController::PositionYawPIDController(
    const PositionYawPIDConfig& config)
    : position_pid_(config.position),
      yaw_pid_(config.yaw)
{
}

void PositionYawPIDController::reset()
{
    position_pid_.reset();
    yaw_pid_.reset();

    initialized_ = false;
    initial_reference_yaw_ = 0.0;
    initial_target_yaw_ = 0.0;
}

void PositionYawPIDController::validate_input(
    const PhysicalControllerInput& input)
{
    const auto& state = input.state;
    const auto& reference = input.reference;
    const auto& valid = input.reference_validity;

    if (!state.position_feedback.valid ||
        !state.velocity_feedback.valid ||
        !state.attitude_feedback.valid ||
        !state.yaw_valid) {
        throw std::invalid_argument(
            "PositionYawPIDController: unavailable feedback");
    }

    if (!valid.position ||
        !valid.velocity ||
        !valid.yaw ||
        !valid.yaw_rate) {
        throw std::invalid_argument(
            "PositionYawPIDController: unavailable reference");
    }

    if (!std::isfinite(input.dt) || input.dt <= 0.0 ||
        !state.position.allFinite() ||
        !state.velocity.allFinite() ||
        !std::isfinite(state.unwrapped_yaw) ||
        !std::isfinite(state.measured_yaw_rate) ||
        !reference.position.allFinite() ||
        !reference.velocity.allFinite() ||
        !std::isfinite(reference.yaw) ||
        !std::isfinite(reference.yaw_rate)) {
        throw std::invalid_argument(
            "PositionYawPIDController: non-finite input or invalid dt");
    }
}

double PositionYawPIDController::select_yaw_delta(
    double current_yaw,
    double target_yaw,
    YawDirection direction)
{
    double delta = wrap_pi(target_yaw - current_yaw);

    if (!std::isfinite(delta)) {
        throw std::invalid_argument(
            "PositionYawPIDController: invalid yaw displacement");
    }

    // Preserve the existing +/-pi tie handling.
    if (std::abs(std::abs(delta) - PI) < ANGLE_EPS) {
        delta = PI;
    }

    switch (direction) {
    case YawDirection::SHORTEST:
        break;

    case YawDirection::CW:
        if (delta > 0.0) {
            delta -= TWO_PI;
        }
        break;

    case YawDirection::CCW:
        if (delta < 0.0) {
            delta += TWO_PI;
        }
        break;

    default:
        throw std::invalid_argument(
            "PositionYawPIDController: invalid yaw direction");
    }

    return delta;
}

void PositionYawPIDController::initialize(
    const PhysicalControllerInput& input,
    YawDirection direction)
{
    validate_input(input);

    const double target =
        input.state.unwrapped_yaw +
        select_yaw_delta(
            input.state.unwrapped_yaw,
            input.reference.yaw,
            direction);

    if (!std::isfinite(target)) {
        throw std::invalid_argument(
            "PositionYawPIDController: non-finite yaw target");
    }

    reset();

    initial_reference_yaw_ = input.reference.yaw;
    initial_target_yaw_ = target;
    initialized_ = true;
}

ControllerResult PositionYawPIDController::update(
    const PhysicalControllerInput& input)
{
    if (!initialized_) {
        throw std::logic_error(
            "PositionYawPIDController: not initialized");
    }

    validate_input(input);

    algorithm::AlgorithmInput<3> position_input;
    position_input.error =
        input.reference.position - input.state.position;
    position_input.error_rate =
        input.reference.velocity - input.state.velocity;
    position_input.dt = input.dt;

    // Preserve the initial continuous yaw alignment.
    // Subsequent polynomial yaw references are applied relative
    // to the initial reference without introducing angle wrapping.
    const double target_yaw =
        initial_target_yaw_ +
        (input.reference.yaw - initial_reference_yaw_);

    algorithm::AlgorithmInput<1> yaw_input;
    yaw_input.error(0) =
        target_yaw - input.state.unwrapped_yaw;
    yaw_input.error_rate(0) =
        input.reference.yaw_rate - input.state.measured_yaw_rate;
    yaw_input.dt = input.dt;

    // Validate derived errors before updating either PID history.
    const double position_error = position_input.error.norm();

    if (!position_input.error.allFinite() ||
        !position_input.error_rate.allFinite() ||
        !yaw_input.error.allFinite() ||
        !yaw_input.error_rate.allFinite() ||
        !std::isfinite(position_error)) {
        throw std::runtime_error(
            "PositionYawPIDController: non-finite tracking error");
    }

    const auto position_output =
        position_pid_.update_with_feedforward(
            position_input,
            input.reference.velocity);

    const auto yaw_output =
        yaw_pid_.update(yaw_input);

    if (!position_output.command.allFinite() ||
        !yaw_output.command.allFinite()) {
        throw std::runtime_error(
            "PositionYawPIDController: non-finite output");
    }

    ControllerResult result;
    result.output.mode = ControlMode::VELOCITY;
    result.output.velocity = position_output.command;
    result.output.yaw_rate = yaw_output.command(0);

    result.errors.position_error = position_error;
    result.errors.yaw_error = yaw_input.error(0);

    return result;
}

}  // namespace drone_node::controller