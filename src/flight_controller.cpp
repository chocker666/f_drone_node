
#include "drone_node/flight_controller.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

FlightController::FlightController()
    : Node("flight_controller")
{
    control_frequency_ = this->declare_parameter<double>(
        "control_frequency", 50.0);

    state_timeout_ = this->declare_parameter<double>(
        "state_timeout", 0.2);

    enable_setpoint_stream_ = this->declare_parameter<bool>(
        "enable_setpoint_stream", false);

    if (!std::isfinite(control_frequency_) ||
        control_frequency_ <= 0.0) {
        throw std::runtime_error(
            "control_frequency must be finite and positive");
    }

    if (!std::isfinite(state_timeout_) ||
        state_timeout_ <= 0.0) {
        throw std::runtime_error(
            "state_timeout must be finite and positive");
    }

    // State feedback: keep only the latest available sample.
    vehicle_state_sub_ =
        this->create_subscription<VehicleState>(
            "/px4_interface/out/vehicle_state",
            rclcpp::QoS(1),
            std::bind(
                &FlightController::vehicle_state_callback,
                this,
                std::placeholders::_1));

    // Position setpoint sent to PX4Interface.
    trajectory_setpoint_pub_ =
        this->create_publisher<TrajectorySetpoint>(
            "/px4_interface/in/trajectory_setpoint",
            rclcpp::QoS(1));

    const auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                1.0 / control_frequency_));

    control_timer_ = this->create_wall_timer(
        period,
        std::bind(&FlightController::control_loop, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Flight controller started: %.2f Hz, "
        "setpoint stream %s",
        control_frequency_,
        enable_setpoint_stream_ ? "enabled" : "disabled");
}

void FlightController::vehicle_state_callback(
    const VehicleState::SharedPtr msg)
{
    const auto &p = msg->position;
    const auto &v = msg->velocity;
    const auto &q = msg->orientation;

    const bool finite =
        std::isfinite(p.x) &&
        std::isfinite(p.y) &&
        std::isfinite(p.z) &&
        std::isfinite(v.x) &&
        std::isfinite(v.y) &&
        std::isfinite(v.z) &&
        std::isfinite(q.x) &&
        std::isfinite(q.y) &&
        std::isfinite(q.z) &&
        std::isfinite(q.w);

    const double q_norm_squared =
        q.x * q.x +
        q.y * q.y +
        q.z * q.z +
        q.w * q.w;

    if (!finite ||
        !std::isfinite(q_norm_squared) ||
        q_norm_squared < 1e-6) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Invalid vehicle state received");
        return;
    }

    latest_vehicle_state_ = *msg;
    has_valid_state_ = true;
    last_state_receive_time_ = SteadyClock::now();
}

void FlightController::control_loop()
{
    ++control_cycle_count_;

    if (!enable_setpoint_stream_) {
        return;
    }

    if (!has_valid_state_) {
        return;
    }

    const double state_age =
        std::chrono::duration<double>(
            SteadyClock::now() -
            last_state_receive_time_).count();

    if (state_age > state_timeout_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Vehicle state timeout: %.3f s",
            state_age);
        return;
    }

    if (!target_initialized_) {
        target_position_ = latest_vehicle_state_.position;

        const auto &q = latest_vehicle_state_.orientation;

        // Quaternion is body FLU -> world ENU.
        const double yaw_rad = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        constexpr double RAD_TO_DEG =
            180.0 / 3.14159265358979323846;

        target_yaw_deg_ =
            static_cast<float>(yaw_rad * RAD_TO_DEG);

        target_initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Position target captured: "
            "(%.3f, %.3f, %.3f), yaw %.2f deg",
            target_position_.x,
            target_position_.y,
            target_position_.z,
            target_yaw_deg_);
    }

    TrajectorySetpoint setpoint{};

    setpoint.stamp = this->now();
    setpoint.control_mode = TrajectorySetpoint::POSITION;

    setpoint.position = target_position_;
    setpoint.yaw = target_yaw_deg_;

    trajectory_setpoint_pub_->publish(setpoint);
    ++setpoint_publish_count_;
}
