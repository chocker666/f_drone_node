#pragma once

#include <cmath>
#include <mutex>
#include <stdexcept>

#include "drone_node/types.hpp"

namespace drone_node
{

class StateCache
{
public:
    enum class AttitudeResult
    {
        ACCEPTED,
        REJECTED,
        YAW_DISCONTINUITY
    };

    explicit StateCache(double timeout)
        : timeout_(timeout)
    {
        if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
            throw std::invalid_argument("Invalid state timeout");
        }
    }

    bool update_position(
        const Eigen::Vector3d& position,
        SteadyTime received_at)
    {
        if (!position.allFinite()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        if (state_.position_feedback.valid &&
            received_at < state_.position_feedback.received_at) {
            return false;
        }

        state_.position = position;
        state_.position_feedback = {true, received_at};
        return true;
    }

    bool update_velocity(
        const Eigen::Vector3d& velocity,
        SteadyTime received_at)
    {
        if (!velocity.allFinite()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        if (state_.velocity_feedback.valid &&
            received_at < state_.velocity_feedback.received_at) {
            return false;
        }

        state_.velocity = velocity;
        state_.velocity_feedback = {true, received_at};
        return true;
    }

    AttitudeResult update_attitude(
        Eigen::Quaterniond orientation,
        SteadyTime received_at)
    {
        if (!orientation.coeffs().allFinite()) {
            return AttitudeResult::REJECTED;
        }

        const double norm2 =
            orientation.w() * orientation.w() +
            orientation.x() * orientation.x() +
            orientation.y() * orientation.y() +
            orientation.z() * orientation.z();

        if (!std::isfinite(norm2) || norm2 <= 1e-12) {
            return AttitudeResult::REJECTED;
        }

        const double norm = std::sqrt(norm2);
        orientation.w() /= norm;
        orientation.x() /= norm;
        orientation.y() /= norm;
        orientation.z() /= norm;

        const double normalized_norm2 =
            orientation.w() * orientation.w() +
            orientation.x() * orientation.x() +
            orientation.y() * orientation.y() +
            orientation.z() * orientation.z();

        const double siny =
            2.0 * (
                orientation.w() * orientation.z() +
                orientation.x() * orientation.y()) /
            normalized_norm2;

        const double cosy =
            1.0 -
            2.0 * (
                orientation.y() * orientation.y() +
                orientation.z() * orientation.z()) /
            normalized_norm2;

        const double measured_yaw = std::atan2(siny, cosy);

        std::lock_guard<std::mutex> lock(mutex_);

        if (!state_.yaw_valid) {
            state_.unwrapped_yaw = measured_yaw;
            state_.measured_yaw_rate = 0.0;
        } else {
            const double dt = std::chrono::duration<double>(
                received_at -
                state_.attitude_feedback.received_at).count();

            if (dt <= 0.0) {
                return AttitudeResult::REJECTED;
            }

            const double delta =
                wrap_pi(measured_yaw - previous_wrapped_yaw_);

            if (dt > timeout_ ||
                std::abs(delta) >= PI - ANGLE_EPS) {
                state_.orientation = orientation;
                state_.attitude_feedback = {false, received_at};
                state_.yaw_valid = false;
                state_.measured_yaw_rate = 0.0;
                return AttitudeResult::YAW_DISCONTINUITY;
            }

            state_.unwrapped_yaw += delta;
            state_.measured_yaw_rate = delta / dt;
        }

        state_.orientation = orientation;
        state_.attitude_feedback = {true, received_at};
        state_.yaw_valid = true;
        previous_wrapped_yaw_ = measured_yaw;

        return AttitudeResult::ACCEPTED;
    }

    StateSnapshot snapshot(SteadyTime now) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        StateSnapshot result = state_;
        result.captured_at = now;

        result.position_feedback.valid =
            fresh(state_.position_feedback, now);
        result.velocity_feedback.valid =
            fresh(state_.velocity_feedback, now);
        result.attitude_feedback.valid =
            fresh(state_.attitude_feedback, now);

        result.yaw_valid =
            state_.yaw_valid && result.attitude_feedback.valid;

        return result;
    }

    static bool usable(const StateSnapshot& state)
    {
        return
            state.position_feedback.valid &&
            state.velocity_feedback.valid &&
            state.attitude_feedback.valid &&
            state.yaw_valid &&
            state.position.allFinite() &&
            state.velocity.allFinite() &&
            state.orientation.coeffs().allFinite() &&
            std::isfinite(state.unwrapped_yaw) &&
            std::isfinite(state.measured_yaw_rate);
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double TWO_PI = 2.0 * PI;
    static constexpr double ANGLE_EPS = 1e-10;

    static double wrap_pi(double angle)
    {
        double result = std::remainder(angle, TWO_PI);

        if (result <= -PI) {
            result += TWO_PI;
        }

        return result;
    }

    bool fresh(
        const FeedbackValidity& feedback,
        SteadyTime now) const
    {
        return
            feedback.valid &&
            now >= feedback.received_at &&
            std::chrono::duration<double>(
                now - feedback.received_at).count() <= timeout_;
    }

    const double timeout_;
    mutable std::mutex mutex_;

    StateSnapshot state_{};
    double previous_wrapped_yaw_{0.0};
};

}  // namespace drone_node