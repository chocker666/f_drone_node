
#include "drone_node/flight_controller.hpp"
#include "drone_node/algorithm/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

using drone_node::algorithm::ControllerInput;
using drone_node::algorithm::PIDController;
using drone_node::algorithm::PIDParameters;

FlightController::FlightController(
    const rclcpp::NodeOptions & options)
    : Node("flight_controller", options)
{
    control_frequency_ = this->declare_parameter<double>(
        "control_frequency", 50.0);

    state_timeout_ = this->declare_parameter<double>(
        "state_timeout", 0.2);

    enable_setpoint_stream_ =
        this->declare_parameter<bool>(
            "enable_setpoint_stream", false);

    position_tolerance_ = this->declare_parameter<double>(
        "position_tolerance", 0.15);

    velocity_tolerance_ = this->declare_parameter<double>(
        "velocity_tolerance", 0.15);

    arrival_hold_time_ = this->declare_parameter<double>(
        "arrival_hold_time", 0.5);

    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };

    if (!positive(control_frequency_) ||
        !positive(state_timeout_) ||
        !positive(position_tolerance_) ||
        !positive(velocity_tolerance_) ||
        !positive(arrival_hold_time_)) {
        throw std::runtime_error(
            "Invalid flight controller parameters");
    }

    // Algorithm selection: PID for phase 3.
    PIDParameters pid_parameters;

    pid_parameters.kp = this->declare_parameter<double>(
        "pid.kp", 0.8);

    pid_parameters.ki = this->declare_parameter<double>(
        "pid.ki", 0.0);

    pid_parameters.kd = this->declare_parameter<double>(
        "pid.kd", 0.0);

    pid_parameters.max_speed =
        this->declare_parameter<double>(
            "pid.max_speed", 1.0);

    pid_parameters.max_acceleration =
        this->declare_parameter<double>(
            "pid.max_acceleration", 1.0);

    pid_parameters.integral_limit =
        this->declare_parameter<double>(
            "pid.integral_limit", 1.0);

    controller_ =
        std::make_unique<PIDController>(pid_parameters);

    vehicle_state_sub_ =
        this->create_subscription<VehicleState>(
            "/px4_interface/out/vehicle_state",
            rclcpp::QoS(1),
            std::bind(
                &FlightController::vehicle_state_callback,
                this,
                std::placeholders::_1));

    target_position_sub_ =
        this->create_subscription<TargetPosition>(
            "/flight_controller/in/target_position",
            rclcpp::QoS(1),
            std::bind(
                &FlightController::target_position_callback,
                this,
                std::placeholders::_1));

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
        std::bind(
            &FlightController::control_loop,
            this));

    RCLCPP_INFO(
        this->get_logger(),
        "Flight controller started: %.1f Hz, "
        "algorithm PID, output %s",
        control_frequency_,
        enable_setpoint_stream_ ? "enabled" : "disabled");
}

void FlightController::vehicle_state_callback(
    const VehicleState::SharedPtr msg)
{
    const auto & p = msg->position;
    const auto & v = msg->velocity;
    const auto & q = msg->orientation;

    const bool valid =
        std::isfinite(p.x) &&
        std::isfinite(p.y) &&
        std::isfinite(p.z) &&
        std::isfinite(v.x) &&
        std::isfinite(v.y) &&
        std::isfinite(v.z) &&
        std::isfinite(q.x) &&
        std::isfinite(q.y) &&
        std::isfinite(q.z) &&
        std::isfinite(q.w) &&
        std::isfinite(
            q.x*q.x + q.y*q.y +
            q.z*q.z + q.w*q.w) &&
        (q.x*q.x + q.y*q.y +
         q.z*q.z + q.w*q.w) > 1e-6;

    if (!valid) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Invalid vehicle state");
        return;
    }

    latest_vehicle_state_ = *msg;
    has_valid_state_ = true;
    last_state_receive_time_ = SteadyClock::now();
}

void FlightController::target_position_callback(
    const TargetPosition::SharedPtr msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN(
            this->get_logger(),
            "Target rejected: expected frame_id 'map'");
        return;
    }

    const auto & p = msg->point;

    if (!std::isfinite(p.x) ||
        !std::isfinite(p.y) ||
        !std::isfinite(p.z)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Target rejected: invalid coordinates");
        return;
    }

    if (!enable_setpoint_stream_ || !has_valid_state_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Target rejected: controller disabled "
            "or no vehicle state");
        return;
    }

    const double state_age =
        std::chrono::duration<double>(
            SteadyClock::now() -
            last_state_receive_time_).count();

    if (state_age > state_timeout_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Target rejected: stale feedback");
        return;
    }

    target_position_ = Eigen::Vector3d(
        p.x, p.y, p.z);

    has_target_ = true;
    arrival_timer_active_ = false;
    has_last_control_time_ = false;

    controller_->reset();
    change_motion_state(MotionState::MOVING);

    RCLCPP_INFO(
        this->get_logger(),
        "Target accepted: %.3f %.3f %.3f",
        p.x, p.y, p.z);
}

void FlightController::change_motion_state(
    MotionState next_state)
{
    if (motion_state_ == next_state) {
        return;
    }

    motion_state_ = next_state;

    switch (next_state) {
    case MotionState::IDLE:
        RCLCPP_INFO(this->get_logger(), "Motion: IDLE");
        break;
    case MotionState::MOVING:
        RCLCPP_INFO(this->get_logger(), "Motion: MOVING");
        break;
    case MotionState::HOLD:
        RCLCPP_INFO(this->get_logger(), "Motion: HOLD");
        break;
    case MotionState::FAULT:
        RCLCPP_ERROR(this->get_logger(), "Motion: FAULT");
        break;
    }
}

void FlightController::publish_velocity_setpoint(
    const Eigen::Vector3d & velocity)
{
    TrajectorySetpoint setpoint{};

    setpoint.stamp = this->now();
    setpoint.control_mode = TrajectorySetpoint::VELOCITY;

    setpoint.velocity.x = velocity.x();
    setpoint.velocity.y = velocity.y();
    setpoint.velocity.z = velocity.z();

    setpoint.yawspeed = 0.0f;

    trajectory_setpoint_pub_->publish(setpoint);
}

void FlightController::control_loop()
{
    if (!enable_setpoint_stream_ || !has_target_) {
        return;
    }

    const auto now = SteadyClock::now();

    if (!has_valid_state_ ||
        std::chrono::duration<double>(
            now - last_state_receive_time_).count()
            > state_timeout_) {
        change_motion_state(MotionState::FAULT);
        has_target_ = false;
        arrival_timer_active_ = false;
        has_last_control_time_ = false;
        controller_->reset();
        return;
    }

    if (motion_state_ == MotionState::HOLD) {
        publish_velocity_setpoint(Eigen::Vector3d::Zero());
        return;
    }

    if (motion_state_ != MotionState::MOVING) {
        return;
    }

    double dt = 1.0 / control_frequency_;

    if (has_last_control_time_) {
        dt = std::chrono::duration<double>(
            now - last_control_time_).count();

        dt = std::clamp(
            dt, 0.001, 2.0 / control_frequency_);
    }

    last_control_time_ = now;
    has_last_control_time_ = true;

    const auto & p = latest_vehicle_state_.position;
    const auto & v = latest_vehicle_state_.velocity;

    ControllerInput input;

    input.position = Eigen::Vector3d(
        p.x, p.y, p.z);

    input.velocity = Eigen::Vector3d(
        v.x, v.y, v.z);

    input.target_position = target_position_;
    input.dt = dt;

    const double position_error =
        (input.target_position - input.position).norm();

    const double actual_speed = input.velocity.norm();

    if (position_error <= position_tolerance_ &&
        actual_speed <= velocity_tolerance_) {

        if (!arrival_timer_active_) {
            arrival_start_time_ = now;
            arrival_timer_active_ = true;
        }

        const double stable_time =
            std::chrono::duration<double>(
                now - arrival_start_time_).count();

        if (stable_time >= arrival_hold_time_) {
            change_motion_state(MotionState::HOLD);
            controller_->reset();
            publish_velocity_setpoint(
                Eigen::Vector3d::Zero());
            return;
        }
    } else {
        arrival_timer_active_ = false;
    }

    // FlightController only dispatches the algorithm.
    const auto output = controller_->update(input);

    publish_velocity_setpoint(output.velocity_setpoint);
}
