
#include "drone_node/exec/planner/quintic_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_node::exec::planner
{

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

bool finite(double value)
{
    return std::isfinite(value);
}

void requireFinite(
    double value,
    const char* name)
{
    if (!finite(value)) {
        throw std::invalid_argument(
            std::string("QuinticPlanner: non-finite ") + name);
    }
}

void requireVectorFinite(
    const Eigen::Vector3d& value,
    const char* name)
{
    if (!value.allFinite()) {
        throw std::invalid_argument(
            std::string("QuinticPlanner: non-finite ") + name);
    }
}

void requireLimit(
    double value,
    const char* name)
{
    if (!finite(value)) {
        throw std::invalid_argument(
            std::string("QuinticPlanner: invalid limit ") + name);
    }
}

// Binomial coefficient for n <= 5.
double choose(unsigned int n, unsigned int k)
{
    if (k > n) {
        return 0.0;
    }

    double result = 1.0;

    for (unsigned int i = 1; i <= k; ++i) {
        result *= static_cast<double>(n - k + i);
        result /= static_cast<double>(i);
    }

    return result;
}

// Convert the derivative polynomial on [0,T] into
// Bernstein coefficients on the normalized interval [0,1].
//
// A polynomial is contained in the convex hull of its
// Bernstein control values. Consequently, the maximum
// absolute control value is a conservative upper bound.
//
// For vector-valued polynomials, the maximum Euclidean
// norm of the Bernstein control vectors is also an
// upper bound on the Euclidean norm over the interval.
std::vector<double> derivativeBernstein(
    const QuinticPlanner::Coefficients& c,
    double duration,
    unsigned int derivative)
{
    if (derivative > 5) {
        throw std::invalid_argument(
            "QuinticPlanner: unsupported derivative");
    }

    const unsigned int degree = 5 - derivative;

    std::vector<double> power(degree + 1, 0.0);
    std::vector<double> bernstein(degree + 1, 0.0);

    double time_power = 1.0;

    for (unsigned int j = 0; j <= degree; ++j) {
        double factor = 1.0;

        for (unsigned int k = 1; k <= derivative; ++k) {
            factor *= static_cast<double>(j + k);
        }

        power[j] =
            c[j + derivative] * factor * time_power;

        if (!finite(power[j])) {
            throw std::invalid_argument(
                "QuinticPlanner: derivative coefficient overflow");
        }

        time_power *= duration;
    }

    for (unsigned int k = 0; k <= degree; ++k) {
        double value = 0.0;

        for (unsigned int j = 0; j <= k; ++j) {
            value += power[j] *
                choose(k, j) / choose(degree, j);
        }

        if (!finite(value)) {
            throw std::invalid_argument(
                "QuinticPlanner: Bernstein conversion overflow");
        }

        bernstein[k] = value;
    }

    return bernstein;
}

void checkVectorLimit(
    const QuinticPlanner::Coefficients& x,
    const QuinticPlanner::Coefficients& y,
    const QuinticPlanner::Coefficients& z,
    double duration,
    unsigned int derivative,
    double limit,
    const char* name)
{
    if (limit <= 0.0) {
        return;
    }

    const auto bx = derivativeBernstein(
        x, duration, derivative);
    const auto by = derivativeBernstein(
        y, duration, derivative);
    const auto bz = derivativeBernstein(
        z, duration, derivative);

    for (std::size_t i = 0; i < bx.size(); ++i) {
        const double norm = std::hypot(
            bx[i], by[i], bz[i]);

        if (!finite(norm) || norm > limit) {
            throw std::invalid_argument(
                std::string("QuinticPlanner: ") +
                name + " limit cannot be certified");
        }
    }
}

void checkScalarLimit(
    const QuinticPlanner::Coefficients& c,
    double duration,
    unsigned int derivative,
    double limit,
    const char* name)
{
    if (limit <= 0.0) {
        return;
    }

    const auto b = derivativeBernstein(
        c, duration, derivative);

    for (double value : b) {
        if (std::abs(value) > limit) {
            throw std::invalid_argument(
                std::string("QuinticPlanner: ") +
                name + " limit cannot be certified");
        }
    }
}

void appendCoefficients(
    std::vector<double>& destination,
    const QuinticPlanner::Coefficients& coefficients)
{
    destination.insert(
        destination.end(),
        coefficients.begin(),
        coefficients.end());
}

}  // namespace

// ============================================================
// Input validation
// ============================================================

void QuinticPlanner::validateRequest(
    const Request& request)
{
    if (!finite(request.duration) ||
        request.duration <= 0.0) {
        throw std::invalid_argument(
            "QuinticPlanner: duration must be finite and positive");
    }

    if (request.frame_id != "map") {
        throw std::invalid_argument(
            "QuinticPlanner: frame_id must be map");
    }

    if (request.controller_id.empty() ||
        request.config_name.empty()) {
        throw std::invalid_argument(
            "QuinticPlanner: controller identity is empty");
    }

    const auto validateState =
        [](const BoundaryState& state) {
            requireVectorFinite(
                state.position, "position");
            requireVectorFinite(
                state.velocity, "velocity");
            requireVectorFinite(
                state.acceleration, "acceleration");

            requireFinite(state.yaw, "yaw");
            requireFinite(state.yaw_rate, "yaw_rate");
            requireFinite(
                state.yaw_acceleration,
                "yaw_acceleration");
        };

    validateState(request.start);
    validateState(request.goal);

    requireLimit(
        request.limits.max_velocity,
        "max_velocity");
    requireLimit(
        request.limits.max_acceleration,
        "max_acceleration");
    requireLimit(
        request.limits.max_jerk,
        "max_jerk");
    requireLimit(
        request.limits.max_yaw_rate,
        "max_yaw_rate");
    requireLimit(
        request.limits.max_yaw_acceleration,
        "max_yaw_acceleration");
}

// ============================================================
// Quintic polynomial solver
// ============================================================

QuinticPlanner::Coefficients QuinticPlanner::solve(
    double p0,
    double v0,
    double a0,
    double pf,
    double vf,
    double af,
    double duration)
{
    requireFinite(p0, "p0");
    requireFinite(v0, "v0");
    requireFinite(a0, "a0");
    requireFinite(pf, "pf");
    requireFinite(vf, "vf");
    requireFinite(af, "af");

    if (!finite(duration) || duration <= 0.0) {
        throw std::invalid_argument(
            "QuinticPlanner: invalid duration");
    }

    const double T = duration;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    if (!finite(T5) || T5 == 0.0) {
        throw std::invalid_argument(
            "QuinticPlanner: duration is numerically unsuitable");
    }

    const double c0 = p0;
    const double c1 = v0;
    const double c2 = 0.5 * a0;

    // Residual terminal boundary conditions.
    const double dp =
        pf - (c0 + c1 * T + c2 * T2);

    const double dv =
        vf - (c1 + 2.0 * c2 * T);

    const double da =
        af - 2.0 * c2;

    const double c3 =
        (10.0 * dp - 4.0 * dv * T +
         0.5 * da * T2) / T3;

    const double c4 =
        (-15.0 * dp + 7.0 * dv * T -
         da * T2) / T4;

    const double c5 =
        (6.0 * dp - 3.0 * dv * T +
         0.5 * da * T2) / T5;

    Coefficients coefficients{
        c0, c1, c2, c3, c4, c5
    };

    for (double value : coefficients) {
        if (!finite(value)) {
            throw std::invalid_argument(
                "QuinticPlanner: coefficient overflow");
        }
    }

    return coefficients;
}

// ============================================================
// Polynomial evaluation
// ============================================================

double QuinticPlanner::evaluate(
    const Coefficients& coefficients,
    double time,
    unsigned int derivative)
{
    if (!finite(time) || derivative > 3) {
        throw std::invalid_argument(
            "QuinticPlanner: invalid evaluation arguments");
    }

    double result = 0.0;

    for (int i = 5;
         i >= static_cast<int>(derivative);
         --i) {
        double factor = 1.0;

        for (unsigned int j = 0; j < derivative; ++j) {
            factor *= static_cast<double>(
                i - static_cast<int>(j));
        }

        result =
            result * time + coefficients[i] * factor;
    }

    if (!finite(result)) {
        throw std::invalid_argument(
            "QuinticPlanner: evaluation overflow");
    }

    return result;
}

// ============================================================
// Yaw unwrapping
// ============================================================

double QuinticPlanner::unwrapYaw(
    double start_yaw,
    double target_yaw,
    YawDirection direction)
{
    requireFinite(start_yaw, "start_yaw");
    requireFinite(target_yaw, "target_yaw");

    const double delta = std::remainder(
        target_yaw - start_yaw, TWO_PI);

    double adjusted = delta;

    switch (direction) {
    case YawDirection::SHORTEST:
        break;

    case YawDirection::POSITIVE:
        if (adjusted < 0.0) {
            adjusted += TWO_PI;
        }
        break;

    case YawDirection::NEGATIVE:
        if (adjusted > 0.0) {
            adjusted -= TWO_PI;
        }
        break;

    default:
        throw std::invalid_argument(
            "QuinticPlanner: invalid yaw direction");
    }

    const double result = start_yaw + adjusted;

    if (!finite(result)) {
        throw std::invalid_argument(
            "QuinticPlanner: yaw unwrap overflow");
    }

    return result;
}

// ============================================================
// Motion constraint validation
// ============================================================

void QuinticPlanner::validateLimits(
    const Request& request,
    const Coefficients& x,
    const Coefficients& y,
    const Coefficients& z,
    const Coefficients& yaw)
{
    const double T = request.duration;
    const auto& limits = request.limits;

    // Bernstein convex-hull bounds are conservative.
    // Passing certifies the bound on the entire segment;
    // failing does not necessarily imply an actual violation.

    checkVectorLimit(
        x, y, z, T, 1,
        limits.max_velocity, "velocity");

    checkVectorLimit(
        x, y, z, T, 2,
        limits.max_acceleration, "acceleration");

    checkVectorLimit(
        x, y, z, T, 3,
        limits.max_jerk, "jerk");

    checkScalarLimit(
        yaw, T, 1,
        limits.max_yaw_rate, "yaw_rate");

    checkScalarLimit(
        yaw, T, 2,
        limits.max_yaw_acceleration,
        "yaw_acceleration");
}

// ============================================================
// Main planning interface
// ============================================================

QuinticPlanner::ControlTrajectory QuinticPlanner::plan(
    const Request& request)
{
    validateRequest(request);

    const auto& start = request.start;
    const auto& goal = request.goal;

    const double T = request.duration;

    const double target_yaw = unwrapYaw(
        start.yaw,
        goal.yaw,
        request.yaw_direction);

    const Coefficients x = solve(
        start.position.x(),
        start.velocity.x(),
        start.acceleration.x(),
        goal.position.x(),
        goal.velocity.x(),
        goal.acceleration.x(),
        T);

    const Coefficients y = solve(
        start.position.y(),
        start.velocity.y(),
        start.acceleration.y(),
        goal.position.y(),
        goal.velocity.y(),
        goal.acceleration.y(),
        T);

    const Coefficients z = solve(
        start.position.z(),
        start.velocity.z(),
        start.acceleration.z(),
        goal.position.z(),
        goal.velocity.z(),
        goal.acceleration.z(),
        T);

    const Coefficients yaw = solve(
        start.yaw,
        start.yaw_rate,
        start.yaw_acceleration,
        target_yaw,
        goal.yaw_rate,
        goal.yaw_acceleration,
        T);

    // Validate finite values over the full interval
    // using Bernstein representations.
    (void)derivativeBernstein(x, T, 0);
    (void)derivativeBernstein(y, T, 0);
    (void)derivativeBernstein(z, T, 0);
    (void)derivativeBernstein(yaw, T, 0);

    validateLimits(request, x, y, z, yaw);

    ControlTrajectory trajectory{};

    trajectory.trajectory_id =
        request.trajectory_id;

    trajectory.frame_id =
        request.frame_id;

    trajectory.segment_count = 1;

    trajectory.segment_duration = {T};

    trajectory.coeff_x.reserve(6);
    trajectory.coeff_y.reserve(6);
    trajectory.coeff_z.reserve(6);
    trajectory.coeff_yaw.reserve(6);

    appendCoefficients(trajectory.coeff_x, x);
    appendCoefficients(trajectory.coeff_y, y);
    appendCoefficients(trajectory.coeff_z, z);
    appendCoefficients(trajectory.coeff_yaw, yaw);

    trajectory.controller_id =
        request.controller_id;

    trajectory.config_name =
        request.config_name;

    return trajectory;
}

}  // namespace drone_node::exec::planner
