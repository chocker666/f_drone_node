
#include "drone_node/ctrl/trajectory_sampler/discrete_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_node::trajectory_sampler
{

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double TIME_EPS = 1e-9;

bool finite(double value)
{
    return std::isfinite(value);
}

bool finite_vector(const Eigen::Vector3d& value)
{
    return value.allFinite();
}

double unwrap_yaw(double previous, double current)
{
    const double delta = std::remainder(
        current - previous, TWO_PI);

    return previous + delta;
}

Eigen::Vector3d to_eigen(
    const geometry_msgs::msg::Vector3& value)
{
    return Eigen::Vector3d(
        value.x, value.y, value.z);
}

Eigen::Vector3d to_eigen(
    const geometry_msgs::msg::Point& value)
{
    return Eigen::Vector3d(
        value.x, value.y, value.z);
}

bool all_nan(const Eigen::Vector3d& value)
{
    return std::isnan(value.x()) &&
           std::isnan(value.y()) &&
           std::isnan(value.z());
}

bool valid_optional_vector(const Eigen::Vector3d& value)
{
    return finite_vector(value) || all_nan(value);
}

// Quintic Hermite interpolation.
// The polynomial is expressed in normalized time u in [0, 1].
double quintic(
    double p0, double v0, double a0,
    double p1, double v1, double a1,
    double duration,
    double u,
    int derivative)
{
    const double h = duration;

    const double c0 = p0;
    const double c1 = v0 * h;
    const double c2 = 0.5 * a0 * h * h;

    const double dp = p1 - (c0 + c1 + c2);
    const double dv = v1 * h - (c1 + 2.0 * c2);
    const double da = a1 * h * h - 2.0 * c2;

    const double c3 = 10.0 * dp - 4.0 * dv + 0.5 * da;
    const double c4 = -15.0 * dp + 7.0 * dv - da;
    const double c5 = 6.0 * dp - 3.0 * dv + 0.5 * da;

    if (derivative == 0) {
        return c0 + u * (c1 + u * (c2 + u *
            (c3 + u * (c4 + u * c5))));
    }

    if (derivative == 1) {
        return (c1 + u * (2.0 * c2 + u *
            (3.0 * c3 + u * (4.0 * c4 +
            u * 5.0 * c5)))) / h;
    }

    if (derivative == 2) {
        return (2.0 * c2 + u * (6.0 * c3 +
            u * (12.0 * c4 + u * 20.0 * c5))) /
            (h * h);
    }

    throw std::invalid_argument(
        "DiscreteSampler: unsupported derivative order");
}

}  // namespace

DiscreteSampler::DiscreteSampler(
    const drone_node::msg::DiscreteTraj& trajectory)
{
    if (trajectory.points.size() < 2) {
        throw std::invalid_argument(
            "DiscreteSampler: at least two points required");
    }

    nodes_.reserve(trajectory.points.size());

    for (const auto& point : trajectory.points) {
        Node node;

        node.time = point.time_from_start;

        node.position = to_eigen(point.position);
        node.velocity = to_eigen(point.velocity);
        node.acceleration = to_eigen(point.acceleration);

        node.yaw = point.yaw;
        node.yaw_rate = point.yaw_speed;

        if (!finite(node.time) ||
            !finite_vector(node.position) ||
            !finite(node.yaw)) {
            throw std::invalid_argument(
                "DiscreteSampler: invalid time, position or yaw");
        }

        if (!valid_optional_vector(node.velocity) ||
            !valid_optional_vector(node.acceleration)) {
            throw std::invalid_argument(
                "DiscreteSampler: velocity/acceleration must "
                "be fully finite or fully NaN");
        }

        if (!finite(node.yaw_rate) &&
            !std::isnan(node.yaw_rate)) {
            throw std::invalid_argument(
                "DiscreteSampler: invalid yaw speed");
        }

        node.velocity_valid = finite_vector(node.velocity);
        node.acceleration_valid =
            finite_vector(node.acceleration);
        node.yaw_rate_valid = finite(node.yaw_rate);

        if (!nodes_.empty()) {
            const Node& previous = nodes_.back();

            if (node.time <= previous.time + TIME_EPS) {
                throw std::invalid_argument(
                    "DiscreteSampler: timestamps must "
                    "be strictly increasing");
            }

            node.yaw = unwrap_yaw(previous.yaw, node.yaw);

            if (!finite(node.yaw)) {
                throw std::invalid_argument(
                    "DiscreteSampler: yaw unwrap overflow");
            }
        }

        nodes_.push_back(node);
    }

    const double start_time = nodes_.front().time;

    for (auto& node : nodes_) {
        node.time -= start_time;
    }

    duration_ = nodes_.back().time;

    if (!finite(duration_) || duration_ <= 0.0) {
        throw std::invalid_argument(
            "DiscreteSampler: invalid duration");
    }

    // Fill missing velocity from position differences.
    // Interior nodes use a nonuniform central estimate.
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        Node& node = nodes_[i];

        if (node.velocity_valid) {
            continue;
        }

        if (i == 0) {
            const double dt =
                nodes_[1].time - nodes_[0].time;

            node.velocity =
                (nodes_[1].position - nodes_[0].position) / dt;
        } else if (i + 1 == nodes_.size()) {
            const double dt =
                nodes_[i].time - nodes_[i - 1].time;

            node.velocity =
                (nodes_[i].position -
                 nodes_[i - 1].position) / dt;
        } else {
            const double h0 =
                nodes_[i].time - nodes_[i - 1].time;
            const double h1 =
                nodes_[i + 1].time - nodes_[i].time;

            const Eigen::Vector3d left =
                (nodes_[i].position -
                 nodes_[i - 1].position) / h0;

            const Eigen::Vector3d right =
                (nodes_[i + 1].position -
                 nodes_[i].position) / h1;

            node.velocity =
                (h1 * left + h0 * right) / (h0 + h1);
        }

        if (!finite_vector(node.velocity)) {
            throw std::invalid_argument(
                "DiscreteSampler: velocity estimation overflow");
        }

        node.velocity_valid = true;
    }

    // Estimate missing acceleration from neighboring velocities.
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        Node& node = nodes_[i];

        if (node.acceleration_valid) {
            continue;
        }

        if (i == 0) {
            const double dt =
                nodes_[1].time - nodes_[0].time;

            node.acceleration =
                (nodes_[1].velocity - nodes_[0].velocity) / dt;
        } else if (i + 1 == nodes_.size()) {
            const double dt =
                nodes_[i].time - nodes_[i - 1].time;

            node.acceleration =
                (nodes_[i].velocity -
                 nodes_[i - 1].velocity) / dt;
        } else {
            const double dt =
                nodes_[i + 1].time - nodes_[i - 1].time;

            node.acceleration =
                (nodes_[i + 1].velocity -
                 nodes_[i - 1].velocity) / dt;
        }

        if (!finite_vector(node.acceleration)) {
            throw std::invalid_argument(
                "DiscreteSampler: acceleration estimation overflow");
        }

        node.acceleration_valid = true;
    }

    // Estimate missing yaw rate using unwrapped yaw.
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        Node& node = nodes_[i];

        if (node.yaw_rate_valid) {
            continue;
        }

        if (i == 0) {
            const double dt =
                nodes_[1].time - nodes_[0].time;

            node.yaw_rate =
                (nodes_[1].yaw - nodes_[0].yaw) / dt;
        } else if (i + 1 == nodes_.size()) {
            const double dt =
                nodes_[i].time - nodes_[i - 1].time;

            node.yaw_rate =
                (nodes_[i].yaw - nodes_[i - 1].yaw) / dt;
        } else {
            const double dt =
                nodes_[i + 1].time - nodes_[i - 1].time;

            node.yaw_rate =
                (nodes_[i + 1].yaw -
                 nodes_[i - 1].yaw) / dt;
        }

        if (!finite(node.yaw_rate)) {
            throw std::invalid_argument(
                "DiscreteSampler: yaw rate estimation overflow");
        }

        node.yaw_rate_valid = true;
    }
}

ReferenceState DiscreteSampler::sample(double t) const
{
    return sample_with_validity(t).state;
}

DiscreteSampler::SampleResult
DiscreteSampler::sample_with_validity(double t) const
{
    if (!finite(t) || t < 0.0) {
        throw std::invalid_argument(
            "DiscreteSampler: invalid sample time");
    }

    SampleResult result;

    result.validity.position = true;
    result.validity.velocity = true;
    result.validity.acceleration = true;
    result.validity.yaw = true;
    result.validity.yaw_rate = true;

    // Endpoint holding is retained for compatibility with
    // the existing CTRL behavior.
    if (t >= duration_) {
        const Node& last = nodes_.back();

        result.state.position = last.position;
        result.state.yaw = last.yaw;

        result.state.velocity.setZero();
        result.state.acceleration.setZero();
        result.state.yaw_rate = 0.0;

        return result;
    }

    auto upper = std::upper_bound(
        nodes_.begin(),
        nodes_.end(),
        t,
        [](double value, const Node& node) {
            return value < node.time;
        });

    const std::size_t index =
        upper == nodes_.begin()
            ? 0
            : static_cast<std::size_t>(
                std::distance(nodes_.begin(), upper) - 1);

    const Node& left = nodes_[index];
    const Node& right = nodes_[index + 1];

    const double h = right.time - left.time;
    const double u = std::clamp(
        (t - left.time) / h, 0.0, 1.0);

    for (int axis = 0; axis < 3; ++axis) {
        result.state.position[axis] = quintic(
            left.position[axis],
            left.velocity[axis],
            left.acceleration[axis],
            right.position[axis],
            right.velocity[axis],
            right.acceleration[axis],
            h, u, 0);

        result.state.velocity[axis] = quintic(
            left.position[axis],
            left.velocity[axis],
            left.acceleration[axis],
            right.position[axis],
            right.velocity[axis],
            right.acceleration[axis],
            h, u, 1);

        result.state.acceleration[axis] = quintic(
            left.position[axis],
            left.velocity[axis],
            left.acceleration[axis],
            right.position[axis],
            right.velocity[axis],
            right.acceleration[axis],
            h, u, 2);
    }

    // Cubic Hermite interpolation for yaw.
    const double y0 = left.yaw;
    const double y1 = right.yaw;
    const double m0 = left.yaw_rate * h;
    const double m1 = right.yaw_rate * h;

    const double u2 = u * u;
    const double u3 = u2 * u;

    result.state.yaw =
        (2.0 * u3 - 3.0 * u2 + 1.0) * y0 +
        (u3 - 2.0 * u2 + u) * m0 +
        (-2.0 * u3 + 3.0 * u2) * y1 +
        (u3 - u2) * m1;

    result.state.yaw_rate =
        ((6.0 * u2 - 6.0 * u) * y0 +
         (3.0 * u2 - 4.0 * u + 1.0) * m0 +
         (-6.0 * u2 + 6.0 * u) * y1 +
         (3.0 * u2 - 2.0 * u) * m1) / h;

    if (!finite_vector(result.state.position) ||
        !finite_vector(result.state.velocity) ||
        !finite_vector(result.state.acceleration) ||
        !finite(result.state.yaw) ||
        !finite(result.state.yaw_rate)) {
        throw std::runtime_error(
            "DiscreteSampler: non-finite interpolated state");
    }

    return result;
}

double DiscreteSampler::duration() const
{
    return duration_;
}

}  // namespace drone_node::trajectory_sampler
