#include "drone_node/ctrl/trajectory_sampler/discrete_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_node::trajectory_sampler
{

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double ANGLE_EPS = 1e-10;

// Numerical collinearity tolerance, not a corner smoothing radius.
constexpr double COLLINEAR_EPS = 1e-12;

bool positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

Eigen::Vector3d position(
    const drone_node::msg::FixedTrajectory& point)
{
    return Eigen::Vector3d(
        point.position.x,
        point.position.y,
        point.position.z);
}

void check_duration_sum(double previous, double next)
{
    if (!std::isfinite(next) || next <= previous) {
        throw std::invalid_argument(
            "ListSampler: unrepresentable trajectory duration");
    }
}

}  // namespace

ListSampler::Profile ListSampler::plan(
    double distance,
    double max_speed,
    double max_acceleration)
{
    if (!positive(distance) ||
        !positive(max_speed) ||
        !positive(max_acceleration)) {
        throw std::invalid_argument(
            "ListSampler: invalid profile parameters");
    }

    Profile profile;
    profile.distance = distance;
    profile.acceleration = max_acceleration;

    // The shorter ramp produces either a trapezoid or a triangle.
    profile.ramp = std::min(
        max_speed / max_acceleration,
        std::sqrt(distance / max_acceleration));

    profile.peak = max_acceleration * profile.ramp;

    if (!positive(profile.ramp) || !positive(profile.peak)) {
        throw std::invalid_argument(
            "ListSampler: unrepresentable motion profile");
    }

    profile.cruise = std::max(
        0.0,
        (distance - profile.peak * profile.ramp) /
            profile.peak);

    profile.duration = 2.0 * profile.ramp + profile.cruise;

    if (!std::isfinite(profile.cruise) ||
        !positive(profile.duration)) {
        throw std::invalid_argument(
            "ListSampler: invalid profile duration");
    }

    return profile;
}

ListSampler::Motion ListSampler::evaluate(
    const Profile& profile,
    double t)
{
    Motion motion;

    if (t >= profile.duration) {
        motion.distance = profile.distance;
        return motion;
    }

    if (t < profile.ramp) {
        motion.distance =
            0.5 * profile.acceleration * t * t;
        motion.velocity = profile.acceleration * t;
        motion.acceleration = profile.acceleration;
        return motion;
    }

    if (t < profile.ramp + profile.cruise) {
        motion.distance =
            0.5 * profile.peak * profile.ramp +
            profile.peak * (t - profile.ramp);
        motion.velocity = profile.peak;
        return motion;
    }

    const double remaining = profile.duration - t;

    motion.distance =
        profile.distance -
        0.5 * profile.acceleration * remaining * remaining;
    motion.velocity = profile.acceleration * remaining;
    motion.acceleration = -profile.acceleration;

    return motion;
}

double ListSampler::yaw_delta(
    double current,
    double target,
    uint8_t direction)
{
    using Point = drone_node::msg::FixedTrajectory;

    double delta = std::remainder(target - current, TWO_PI);

    if (!std::isfinite(delta)) {
        throw std::invalid_argument(
            "ListSampler: invalid yaw displacement");
    }

    if (delta <= -PI) {
        delta += TWO_PI;
    }

    // Preserve existing +/-pi tie handling.
    if (std::abs(std::abs(delta) - PI) < ANGLE_EPS) {
        delta = PI;
    }

    switch (direction) {
    case Point::SHORTEST:
        break;

    case Point::CW:
        if (delta > 0.0) {
            delta -= TWO_PI;
        }
        break;

    case Point::CCW:
        if (delta < 0.0) {
            delta += TWO_PI;
        }
        break;

    default:
        throw std::invalid_argument(
            "ListSampler: invalid yaw direction");
    }

    return delta;
}

ListSampler::ListSampler(
    const drone_node::msg::ListTrajectory& trajectory)
{
    using Point = drone_node::msg::FixedTrajectory;

    if (trajectory.points.empty()) {
        throw std::invalid_argument(
            "ListSampler: empty path");
    }

    if (!std::isfinite(trajectory.target_speed) ||
        trajectory.target_speed < 0.0 ||
        !positive(trajectory.max_speed) ||
        !positive(trajectory.max_acceleration) ||
        !positive(trajectory.max_yaw_rate) ||
        !positive(trajectory.max_yaw_acceleration)) {
        throw std::invalid_argument(
            "ListSampler: invalid motion limits");
    }

    // Validate every point, including points later removed from
    // the translation path as duplicate or collinear positions.
    for (const auto& point : trajectory.points) {
        if (!position(point).allFinite() ||
            !std::isfinite(point.yaw)) {
            throw std::invalid_argument(
                "ListSampler: non-finite path point");
        }

        if (point.yaw_direction != Point::SHORTEST &&
            point.yaw_direction != Point::CW &&
            point.yaw_direction != Point::CCW) {
            throw std::invalid_argument(
                "ListSampler: invalid path yaw direction");
        }
    }

    const double speed =
        trajectory.target_speed == 0.0
            ? trajectory.max_speed
            : std::min(
                trajectory.target_speed,
                trajectory.max_speed);

    // Translation: retain corners, remove duplicate positions,
    // and merge successive segments in the same direction.
    std::vector<Eigen::Vector3d> vertices;

    for (const auto& point : trajectory.points) {
        const Eigen::Vector3d next = position(point);

        if (vertices.empty()) {
            vertices.push_back(next);
            continue;
        }

        const Eigen::Vector3d delta = next - vertices.back();
        const double length = delta.stableNorm();

        if (!std::isfinite(length)) {
            throw std::invalid_argument(
                "ListSampler: invalid path displacement");
        }

        if (length == 0.0) {
            continue;
        }

        if (vertices.size() >= 2) {
            const Eigen::Vector3d previous =
                vertices.back() - vertices[vertices.size() - 2];

            const double previous_length = previous.stableNorm();

            if (!positive(previous_length)) {
                throw std::invalid_argument(
                    "ListSampler: invalid previous segment");
            }

            const Eigen::Vector3d previous_direction =
                previous / previous_length;
            const Eigen::Vector3d next_direction =
                delta / length;

            if (previous_direction.dot(next_direction) > 0.0 &&
                previous_direction.cross(next_direction).norm() <=
                    COLLINEAR_EPS) {
                vertices.back() = next;
                continue;
            }
        }

        vertices.push_back(next);
    }

    final_position_ = position(trajectory.points.back());

    for (std::size_t i = 1; i < vertices.size(); ++i) {
        const Eigen::Vector3d delta = vertices[i] - vertices[i - 1];
        const double length = delta.stableNorm();

        if (!positive(length)) {
            throw std::invalid_argument(
                "ListSampler: invalid merged segment");
        }

        LinearSegment segment;
        segment.begin = linear_duration_;
        segment.origin = vertices[i - 1];
        segment.direction = delta / length;
        segment.profile = plan(
            length,
            speed,
            trajectory.max_acceleration);

        const double next =
            linear_duration_ + segment.profile.duration;
        check_duration_sum(linear_duration_, next);

        linear_segments_.push_back(segment);
        linear_duration_ = next;
    }

    // Yaw is planned independently from translation.
    // Every nonzero yaw segment starts and ends at zero yaw rate.
    final_yaw_ = trajectory.points.front().yaw;

    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
        const auto& point = trajectory.points[i];

        const double delta = yaw_delta(
            final_yaw_, point.yaw, point.yaw_direction);

        const double next_yaw = final_yaw_ + delta;

        if (!std::isfinite(next_yaw)) {
            throw std::invalid_argument(
                "ListSampler: non-finite unwrapped yaw");
        }

        if (delta != 0.0) {
            YawSegment segment;
            segment.begin = yaw_duration_;
            segment.origin = final_yaw_;
            segment.direction = delta > 0.0 ? 1.0 : -1.0;
            segment.profile = plan(
                std::abs(delta),
                trajectory.max_yaw_rate,
                trajectory.max_yaw_acceleration);

            const double next =
                yaw_duration_ + segment.profile.duration;
            check_duration_sum(yaw_duration_, next);

            yaw_segments_.push_back(segment);
            yaw_duration_ = next;
        }

        final_yaw_ = next_yaw;
    }
}

ReferenceState ListSampler::sample(double t) const
{
    if (!std::isfinite(t) || t < 0.0) {
        throw std::invalid_argument(
            "ListSampler: invalid sample time");
    }

    // A finished channel holds its endpoint with zero derivatives.
    ReferenceState reference;
    reference.position = final_position_;
    reference.yaw = final_yaw_;

    if (t < linear_duration_) {
        auto it = std::upper_bound(
            linear_segments_.begin(),
            linear_segments_.end(),
            t,
            [](double time, const LinearSegment& segment) {
                return time < segment.begin;
            });
        --it;

        const auto motion = evaluate(
            it->profile, t - it->begin);

        reference.position =
            it->origin + it->direction * motion.distance;
        reference.velocity =
            it->direction * motion.velocity;
        reference.acceleration =
            it->direction * motion.acceleration;
    }

    if (t < yaw_duration_) {
        auto it = std::upper_bound(
            yaw_segments_.begin(),
            yaw_segments_.end(),
            t,
            [](double time, const YawSegment& segment) {
                return time < segment.begin;
            });
        --it;

        const auto motion = evaluate(
            it->profile, t - it->begin);

        reference.yaw =
            it->origin + it->direction * motion.distance;
        reference.yaw_rate =
            it->direction * motion.velocity;
    }

    return reference;
}

double ListSampler::duration() const
{
    return std::max(linear_duration_, yaw_duration_);
}

}  // namespace drone_node::trajectory_sampler