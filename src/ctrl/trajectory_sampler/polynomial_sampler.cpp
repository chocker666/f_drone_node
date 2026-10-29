
#include "drone_node/ctrl/trajectory_sampler/polynomial_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace drone_node::trajectory_sampler
{

namespace
{

constexpr double TIME_EPS = 1e-9;

// Absolute and relative tolerances for boundary continuity.
// These are numerical tolerances, not flight-dynamics limits.
constexpr double POSITION_TOL = 1e-5;
constexpr double VELOCITY_TOL = 1e-4;
constexpr double ACCELERATION_TOL = 1e-3;
constexpr double YAW_TOL = 1e-5;
constexpr double YAW_RATE_TOL = 1e-4;

constexpr std::size_t COEFFICIENT_COUNT = 6;

bool finite(double value)
{
    return std::isfinite(value);
}

bool approximately_equal(
    double a,
    double b,
    double tolerance)
{
    if (!finite(a) || !finite(b)) {
        return false;
    }

    const double scale =
        std::max({1.0, std::abs(a), std::abs(b)});

    return std::abs(a - b) <= tolerance * scale;
}

void validate_coefficients(
    const std::vector<double>& coefficients,
    const char* name)
{
    for (double value : coefficients) {
        if (!finite(value)) {
            throw std::invalid_argument(
                std::string("PolynomialSampler: non-finite ") +
                name + " coefficient");
        }
    }
}

}  // namespace


PolynomialSampler::PolynomialSampler(
    const drone_node::msg::ControlTrajectory& trajectory)
{
    const std::size_t count =
        static_cast<std::size_t>(trajectory.segment_count);

    if (count == 0) {
        throw std::invalid_argument(
            "PolynomialSampler: segment_count must be positive");
    }

    // Prevent size multiplication overflow.
    if (count >
        std::numeric_limits<std::size_t>::max() /
            COEFFICIENT_COUNT) {
        throw std::invalid_argument(
            "PolynomialSampler: segment count overflow");
    }

    const std::size_t expected_coefficients =
        count * COEFFICIENT_COUNT;

    if (trajectory.segment_duration.size() != count ||
        trajectory.coeff_x.size() != expected_coefficients ||
        trajectory.coeff_y.size() != expected_coefficients ||
        trajectory.coeff_z.size() != expected_coefficients ||
        trajectory.coeff_yaw.size() != expected_coefficients) {
        throw std::invalid_argument(
            "PolynomialSampler: inconsistent array lengths");
    }

    validate_coefficients(trajectory.coeff_x, "X");
    validate_coefficients(trajectory.coeff_y, "Y");
    validate_coefficients(trajectory.coeff_z, "Z");
    validate_coefficients(trajectory.coeff_yaw, "yaw");

    segments_.reserve(count);

    double elapsed = 0.0;

    for (std::size_t i = 0; i < count; ++i) {
        const double segment_duration =
            trajectory.segment_duration[i];

        if (!finite(segment_duration) ||
            segment_duration <= TIME_EPS) {
            throw std::invalid_argument(
                "PolynomialSampler: invalid segment duration");
        }

        Segment segment;
        segment.start_time = elapsed;
        segment.duration = segment_duration;

        const std::size_t offset =
            i * COEFFICIENT_COUNT;

        segment.coeff_x.assign(
            trajectory.coeff_x.begin() + offset,
            trajectory.coeff_x.begin() +
                offset + COEFFICIENT_COUNT);

        segment.coeff_y.assign(
            trajectory.coeff_y.begin() + offset,
            trajectory.coeff_y.begin() +
                offset + COEFFICIENT_COUNT);

        segment.coeff_z.assign(
            trajectory.coeff_z.begin() + offset,
            trajectory.coeff_z.begin() +
                offset + COEFFICIENT_COUNT);

        segment.coeff_yaw.assign(
            trajectory.coeff_yaw.begin() + offset,
            trajectory.coeff_yaw.begin() +
                offset + COEFFICIENT_COUNT);

        const double next_elapsed =
            elapsed + segment_duration;

        if (!finite(next_elapsed) ||
            next_elapsed <= elapsed) {
            throw std::invalid_argument(
                "PolynomialSampler: duration overflow "
                "or insufficient time resolution");
        }

        // Validate finite position, velocity and acceleration
        // at both ends of each segment.
        const auto validate_endpoint =
            [&](const std::vector<double>& coefficients,
                unsigned int max_derivative) {
                for (double t : {0.0, segment_duration}) {
                    for (unsigned int d = 0;
                         d <= max_derivative; ++d) {
                        (void)evaluate(coefficients, t, d);
                    }
                }
            };

        validate_endpoint(segment.coeff_x, 2);
        validate_endpoint(segment.coeff_y, 2);
        validate_endpoint(segment.coeff_z, 2);
        validate_endpoint(segment.coeff_yaw, 1);

        if (!segments_.empty()) {
            validate_boundary(segments_.back(), segment);
        }

        segments_.push_back(std::move(segment));
        elapsed = next_elapsed;
    }

    duration_ = elapsed;
}


double PolynomialSampler::evaluate(
    const std::vector<double>& coefficients,
    double t,
    unsigned int derivative_order)
{
    if (coefficients.empty()) {
        throw std::invalid_argument(
            "PolynomialSampler: empty coefficients");
    }

    if (!finite(t)) {
        throw std::invalid_argument(
            "PolynomialSampler: non-finite evaluation time");
    }

    const std::size_t degree = coefficients.size() - 1;

    if (derivative_order > degree) {
        return 0.0;
    }

    // Horner evaluation of the requested derivative.
    // Coefficients are stored in ascending power order.
    double result = 0.0;

    for (std::size_t k = coefficients.size();
         k-- > derivative_order;) {
        double factor = 1.0;

        for (unsigned int d = 0;
             d < derivative_order; ++d) {
            factor *= static_cast<double>(k - d);
        }

        result = result * t + coefficients[k] * factor;
    }

    if (!finite(result)) {
        throw std::runtime_error(
            "PolynomialSampler: polynomial evaluation overflow");
    }

    return result;
}

void PolynomialSampler::validate_boundary(
    const Segment& left,
    const Segment& right)
{
    const double t = left.duration;

    const auto check = [&](const std::vector<double>& a,
                           const std::vector<double>& b,
                           unsigned int derivative,
                           double tolerance,
                           const char* name) {
        const double left_value =
            evaluate(a, t, derivative);

        const double right_value =
            evaluate(b, 0.0, derivative);

        if (!approximately_equal(
                left_value, right_value, tolerance)) {
            throw std::invalid_argument(
                std::string(
                    "PolynomialSampler: discontinuous ") +
                name + " at segment boundary");
        }
    };

    // Position: C2 continuity.
    for (unsigned int derivative = 0;
         derivative <= 2; ++derivative) {
        const double tolerance =
            derivative == 0 ? POSITION_TOL :
            derivative == 1 ? VELOCITY_TOL :
                              ACCELERATION_TOL;

        check(
            left.coeff_x, right.coeff_x,
            derivative, tolerance, "X");

        check(
            left.coeff_y, right.coeff_y,
            derivative, tolerance, "Y");

        check(
            left.coeff_z, right.coeff_z,
            derivative, tolerance, "Z");
    }

    // Yaw is an unwrapped continuous angle.
    // Require yaw and yaw rate continuity.
    check(
        left.coeff_yaw, right.coeff_yaw,
        0, YAW_TOL, "yaw");

    check(
        left.coeff_yaw, right.coeff_yaw,
        1, YAW_RATE_TOL, "yaw rate");
}

ReferenceState PolynomialSampler::sample(double t) const
{
    return sample_with_validity(t).state;
}

ReferenceSample PolynomialSampler::sample_with_validity(double t) const
{
    if (!finite(t) || t < 0.0) {
        throw std::invalid_argument(
            "PolynomialSampler: invalid sample time");
    }

    if (segments_.empty()) {
        throw std::runtime_error(
            "PolynomialSampler: no trajectory segments");
    }

    ReferenceSample result;

    result.validity.position = true;
    result.validity.velocity = true;
    result.validity.acceleration = true;
    result.validity.yaw = true;
    result.validity.yaw_rate = true;

    // After the trajectory ends, hold the terminal position
    // and yaw while setting velocity, acceleration and yaw
    // rate references to zero.
    if (t >= duration_) {
        const Segment& last = segments_.back();
        const double end_time = last.duration;

        result.state.position.x() =
            evaluate(last.coeff_x, end_time, 0);

        result.state.position.y() =
            evaluate(last.coeff_y, end_time, 0);

        result.state.position.z() =
            evaluate(last.coeff_z, end_time, 0);

        result.state.yaw =
            evaluate(last.coeff_yaw, end_time, 0);

        result.state.velocity.setZero();
        result.state.acceleration.setZero();
        result.state.yaw_rate = 0.0;

        return result;
    }

    // Find the segment containing trajectory-relative time t.
    // At an internal boundary, select the following segment.
    auto upper = std::upper_bound(
        segments_.begin(),
        segments_.end(),
        t,
        [](double value, const Segment& segment) {
            return value < segment.start_time;
        });

    const std::size_t index =
        upper == segments_.begin()
            ? 0
            : static_cast<std::size_t>(
                std::distance(segments_.begin(), upper) - 1);

    const Segment& segment = segments_[index];

    const double local_time = std::clamp(
        t - segment.start_time,
        0.0,
        segment.duration);

    result.state.position.x() =
        evaluate(segment.coeff_x, local_time, 0);

    result.state.position.y() =
        evaluate(segment.coeff_y, local_time, 0);

    result.state.position.z() =
        evaluate(segment.coeff_z, local_time, 0);

    result.state.velocity.x() =
        evaluate(segment.coeff_x, local_time, 1);

    result.state.velocity.y() =
        evaluate(segment.coeff_y, local_time, 1);

    result.state.velocity.z() =
        evaluate(segment.coeff_z, local_time, 1);

    result.state.acceleration.x() =
        evaluate(segment.coeff_x, local_time, 2);

    result.state.acceleration.y() =
        evaluate(segment.coeff_y, local_time, 2);

    result.state.acceleration.z() =
        evaluate(segment.coeff_z, local_time, 2);

    result.state.yaw =
        evaluate(segment.coeff_yaw, local_time, 0);

    result.state.yaw_rate =
        evaluate(segment.coeff_yaw, local_time, 1);

    return result;
}

double PolynomialSampler::duration() const
{
    return duration_;
}

}  // namespace drone_node::trajectory_sampler
