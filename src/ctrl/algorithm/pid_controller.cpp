
#include "drone_node/ctrl/algorithm/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_node::algorithm
{

template<int N>
PIDController<N>::PIDController(
    const PIDParameters& parameters)
{
    set_parameters(parameters);
}

template<int N>
void PIDController<N>::set_parameters(
    const PIDParameters& parameters)
{
    const auto nonnegative = [](double x) {
        return std::isfinite(x) && x >= 0.0;
    };

    const auto positive = [](double x) {
        return std::isfinite(x) && x > 0.0;
    };

    if (!nonnegative(parameters.kp) ||
        !nonnegative(parameters.ki) ||
        !nonnegative(parameters.kd) ||
        !positive(parameters.max_output) ||
        !positive(parameters.max_output_rate) ||
        !positive(parameters.integral_limit)) {
        throw std::invalid_argument(
            "Invalid PID parameters");
    }

    parameters_ = parameters;
    reset();
}

template<int N>
const PIDParameters&
PIDController<N>::parameters() const noexcept
{
    return parameters_;
}

template<int N>
void PIDController<N>::reset()
{
    integral_error_.setZero();
    previous_command_.setZero();
}

template<int N>
typename PIDController<N>::Vector
PIDController<N>::limit_vector(
    const Vector& value,
    double limit) const
{
    if (parameters_.limit_mode ==
        PIDLimitMode::PER_AXIS) {
        return value.cwiseMax(-limit).cwiseMin(limit);
    }

    const double norm = value.norm();

    if (norm <= limit || norm <= 0.0) {
        return value;
    }

    return value * (limit / norm);
}

template<int N>
ControllerOutput<N> PIDController<N>::update(
    const ControllerInput<N>& input)
{
    if (!std::isfinite(input.dt) ||
        input.dt <= 0.0 ||
        !input.error.allFinite() ||
        !input.error_rate.allFinite()) {
        throw std::invalid_argument(
            "Invalid PID input");
    }

    const Vector previous_unsaturated =
        parameters_.kp * input.error +
        parameters_.ki * integral_error_ +
        parameters_.kd * input.error_rate;

    // Conditional integration anti-windup.
    bool saturated_outward = false;

    if (parameters_.limit_mode ==
        PIDLimitMode::NORM) {

        saturated_outward =
            previous_unsaturated.norm() >
                parameters_.max_output &&
            previous_unsaturated.dot(input.error) > 0.0;

    } else {

        for (int i = 0; i < N; ++i) {
            if (std::abs(previous_unsaturated(i)) >
                    parameters_.max_output &&
                previous_unsaturated(i) *
                    input.error(i) > 0.0) {
                saturated_outward = true;
                break;
            }
        }
    }

    if (!saturated_outward) {
        integral_error_ += input.error * input.dt;

        integral_error_ = limit_vector(
            integral_error_,
            parameters_.integral_limit);
    }

    const Vector raw_command =
        parameters_.kp * input.error +
        parameters_.ki * integral_error_ +
        parameters_.kd * input.error_rate;

    const Vector limited_command = limit_vector(
        raw_command,
        parameters_.max_output);

    const Vector delta_command =
        limited_command - previous_command_;

    const Vector limited_delta = limit_vector(
        delta_command,
        parameters_.max_output_rate * input.dt);

    const Vector command =
        previous_command_ + limited_delta;

    previous_command_ = command;

    ControllerOutput<N> output;
    output.command = command;
    return output;
}

// Explicit template instantiations.
template class PIDController<1>;
template class PIDController<2>;
template class PIDController<3>;

}  // namespace drone_node::algorithm
