#include "drone_node/ctrl/flight_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

using drone_node::algorithm::PIDParameters;
using PositionPID = drone_node::algorithm::PIDController<3>;
using PositionInput = drone_node::algorithm::ControllerInput<3>;
using YawInput = drone_node::algorithm::ControllerInput<1>;

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double ANGLE_EPS = 1e-10;

}

FlightController::FlightController(
    const rclcpp::NodeOptions& options)
    : Node("flight_controller", options)
{
    control_frequency_ =
        declare_parameter<double>(
            "control_frequency", 50.0);

    status_frequency_ =
        declare_parameter<double>(
            "status_frequency", 10.0);

    state_timeout_ =
        declare_parameter<double>(
            "state_timeout", 0.2);

    enable_setpoint_stream_ =
        declare_parameter<bool>(
            "enable_setpoint_stream", false);

    const auto positive = [](double x) {
        return std::isfinite(x) && x > 0.0;
    };

    if (!positive(control_frequency_) ||
        !positive(status_frequency_) ||
        !positive(state_timeout_)) {
        throw std::runtime_error(
            "Invalid FlightController parameters");
    }

    controller_ =
        std::make_unique<PositionPID>(PIDParameters{});

    vehicle_state_sub_ =
        create_subscription<VehicleState>(
            "/px4_interface/out/vehicle_state",
            rclcpp::QoS(1),
            std::bind(
                &FlightController::vehicle_state_callback,
                this,
                std::placeholders::_1));

    trajectory_sub_ =
        create_subscription<ControlTrajectory>(
            "/flight_controller/in/trajectory",
            rclcpp::QoS(1).reliable(),
            std::bind(
                &FlightController::trajectory_callback,
                this,
                std::placeholders::_1));

    trajectory_setpoint_pub_ =
        create_publisher<TrajectorySetpoint>(
            "/px4_interface/in/trajectory_setpoint",
            rclcpp::QoS(1));

    control_status_pub_ =
        create_publisher<ControlStatus>(
            "/flight_controller/out/control_status",
            rclcpp::QoS(10).reliable());

    const auto control_period =
        std::chrono::duration_cast<
            std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                1.0 / control_frequency_));

    const auto status_period =
        std::chrono::duration_cast<
            std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                1.0 / status_frequency_));

    control_timer_ = create_wall_timer(
        control_period,
        std::bind(
            &FlightController::control_loop, this));

    status_timer_ = create_wall_timer(
        status_period,
        std::bind(
            &FlightController::status_loop, this));

    RCLCPP_INFO(
        get_logger(),
        "FlightController started: %.1f Hz, "
        "status %.1f Hz, output %s",
        control_frequency_,
        status_frequency_,
        enable_setpoint_stream_ ?
            "enabled" : "disabled");
}

double FlightController::wrap_pi(double angle)
{
    // (-pi, pi], with +pi selected at the tie.
    double result = std::remainder(angle, TWO_PI);

    if (result <= -PI) {
        result += TWO_PI;
    }

    return result;
}

double FlightController::yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion& q)
{
    const double norm_squared =
        q.w * q.w + q.x * q.x +
        q.y * q.y + q.z * q.z;

    const double siny =
        2.0 * (q.w * q.z + q.x * q.y) /
        norm_squared;

    const double cosy =
        1.0 -
        2.0 * (q.y * q.y + q.z * q.z) /
        norm_squared;

    return std::atan2(siny, cosy);
}

double FlightController::select_yaw_delta(
    double current_yaw,
    double target_yaw,
    uint8_t direction)
{
    double delta =
        wrap_pi(target_yaw - current_yaw);

    if (std::abs(std::abs(delta) - PI) < ANGLE_EPS) {
        delta = PI;
    }

    using FixedTrajectory =
        drone_node::msg::FixedTrajectory;

    switch (direction) {
    case FixedTrajectory::SHORTEST:
        break;

    case FixedTrajectory::CW:
        if (delta > 0.0) {
            delta -= TWO_PI;
        }
        break;

    case FixedTrajectory::CCW:
        if (delta < 0.0) {
            delta += TWO_PI;
        }
        break;

    default:
        throw std::invalid_argument(
            "Invalid yaw direction");
    }

    return delta;
}

void FlightController::update_unwrapped_yaw(
    double measured_yaw)
{
    const auto receive_time = SteadyClock::now();

    if (!has_yaw_measurement_) {
        previous_wrapped_yaw_ = measured_yaw;
        unwrapped_yaw_ = measured_yaw;
        measured_yaw_rate_ = 0.0;
        last_yaw_receive_time_ = receive_time;
        has_yaw_measurement_ = true;
        return;
    }

    const double dt =
        std::chrono::duration<double>(
            receive_time -
            last_yaw_receive_time_).count();

    if (dt <= 0.0) {
        return;
    }

    const double delta =
        wrap_pi(measured_yaw - previous_wrapped_yaw_);

    // Feedback interruption invalidates continuous yaw.
    // Reinitializing during an active CW/CCW trajectory
    // could silently change its requested turn.
    if (dt > state_timeout_ ||
        std::abs(delta) >= PI - ANGLE_EPS) {
        has_yaw_measurement_ = false;
        measured_yaw_rate_ = 0.0;

        if (trajectory_manager_.has_trajectory()) {
            enter_fault(
                ControlStatus::FAULT_INVALID_STATE);
        }
        return;
    }

    unwrapped_yaw_ += delta;

    measured_yaw_rate_ = delta / dt;

    previous_wrapped_yaw_ = measured_yaw;
    last_yaw_receive_time_ = receive_time;
}

void FlightController::initialize_yaw_reference(
    double target_yaw)
{
    if (!has_yaw_measurement_) {
        throw std::runtime_error(
            "Yaw measurement unavailable");
    }

    const double delta = select_yaw_delta(
        unwrapped_yaw_,
        target_yaw,
        trajectory_manager_.yaw_direction());

    target_unwrapped_yaw_ =
        unwrapped_yaw_ + delta;

    yaw_controller_.reset();
    yaw_reference_initialized_ = true;

    RCLCPP_INFO(
        get_logger(),
        "Yaw target: current=%.2f deg, "
        "target=%.2f deg, delta=%.2f deg, "
        "direction=%u",
        unwrapped_yaw_ * RAD_TO_DEG,
        target_unwrapped_yaw_ * RAD_TO_DEG,
        delta * RAD_TO_DEG,
        static_cast<unsigned int>(
            trajectory_manager_.yaw_direction()));
}

void FlightController::vehicle_state_callback(
    const VehicleState::SharedPtr msg)
{
    const auto& p = msg->position;
    const auto& v = msg->velocity;
    const auto& q = msg->orientation;

    const double q_norm_squared =
        q.x * q.x + q.y * q.y +
        q.z * q.z + q.w * q.w;

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
        std::isfinite(q_norm_squared) &&
        q_norm_squared > 1e-6;

    if (!valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Invalid vehicle state");
        return;
    }

    latest_vehicle_state_ = *msg;
    has_valid_state_ = true;
    last_state_receive_time_ = SteadyClock::now();

    const double yaw = yaw_from_quaternion(q);
    update_unwrapped_yaw(yaw);
}

bool FlightController::state_is_fresh() const
{
    if (!has_valid_state_ || !has_yaw_measurement_) {
        return false;
    }

    const double age =
        std::chrono::duration<double>(
            SteadyClock::now() -
            last_state_receive_time_).count();

    return age <= state_timeout_;
}

void FlightController::trajectory_callback(
    const ControlTrajectory::SharedPtr msg)
{
    if (!enable_setpoint_stream_) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: output disabled");
        return;
    }

    if (!state_is_fresh()) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: no fresh state");
        return;
    }

    std::string reason;

    const auto result =
        trajectory_manager_.accept(*msg, now(), reason);

    if (result ==
        drone_node::TrajectoryManager::AcceptResult::REJECTED) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory %lu rejected: %s",
            static_cast<unsigned long>(
                msg->trajectory_id),
            reason.c_str());
        return;
    }

    if (result ==
        drone_node::TrajectoryManager::AcceptResult::DUPLICATE) {
        return;
    }

    try {
        auto* position_pid =
            dynamic_cast<PositionPID*>(controller_.get());

        if (!position_pid) {
            throw std::runtime_error(
                "Position PID unavailable");
        }

        position_pid->set_parameters(
            trajectory_manager_.position_pid_parameters());

        yaw_controller_.set_parameters(
            trajectory_manager_.yaw_pid_parameters());

        yaw_reference_initialized_ = false;
        has_last_control_time_ = false;
        has_reference_ = false;

        fault_code_ = ControlStatus::FAULT_NONE;

        change_control_state(
            trajectory_manager_.is_started(now()) ?
                ControlStatus::TRACKING :
                ControlStatus::WAITING);

        RCLCPP_INFO(
            get_logger(),
            "Trajectory %lu accepted",
            static_cast<unsigned long>(
                trajectory_manager_.trajectory_id()));

    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "Controller configuration failed: %s",
            e.what());

        enter_fault(
            ControlStatus::FAULT_CONTROLLER_ERROR);
    }
}

void FlightController::change_control_state(
    uint8_t state)
{
    if (control_state_ == state) {
        return;
    }

    control_state_ = state;

    RCLCPP_INFO(
        get_logger(),
        "Control state changed to %u",
        static_cast<unsigned int>(state));
}

void FlightController::enter_fault(
    uint16_t fault_code)
{
    fault_code_ = fault_code;

    trajectory_manager_.clear();

    controller_->reset();
    yaw_controller_.reset();

    yaw_reference_initialized_ = false;
    has_reference_ = false;
    has_last_control_time_ = false;

    change_control_state(ControlStatus::FAULT);

    RCLCPP_ERROR(
        get_logger(),
        "FlightController fault: %u",
        static_cast<unsigned int>(fault_code));
}

void FlightController::publish_velocity_setpoint(
    const Eigen::Vector3d& velocity,
    double yaw_rate)
{
    TrajectorySetpoint setpoint{};

    setpoint.stamp = now();
    setpoint.control_mode = TrajectorySetpoint::VELOCITY;

    setpoint.velocity.x = velocity.x();
    setpoint.velocity.y = velocity.y();
    setpoint.velocity.z = velocity.z();

    // Internal rad/s -> message deg/s.
    setpoint.yawspeed =
        static_cast<float>(yaw_rate * RAD_TO_DEG);

    trajectory_setpoint_pub_->publish(setpoint);
}

void FlightController::control_loop()
{
    if (!enable_setpoint_stream_ ||
        !trajectory_manager_.has_trajectory()) {
        return;
    }

    const auto steady_now = SteadyClock::now();
    const auto ros_now = now();

    if (!state_is_fresh()) {
        enter_fault(
            ControlStatus::FAULT_STATE_TIMEOUT);
        return;
    }

    if (!trajectory_manager_.is_started(ros_now)) {
        change_control_state(ControlStatus::WAITING);
        return;
    }

    if (control_state_ != ControlStatus::TRACKING) {
        has_last_control_time_ = false;
        change_control_state(ControlStatus::TRACKING);
    }

    double dt = 1.0 / control_frequency_;

    if (has_last_control_time_) {
        dt = std::chrono::duration<double>(
            steady_now - last_control_time_).count();

        dt = std::clamp(
            dt, 0.001, 2.0 / control_frequency_);
    }

    last_control_time_ = steady_now;
    has_last_control_time_ = true;

    try {
        latest_reference_ =
            trajectory_manager_.sample(ros_now);

        has_reference_ = true;

        if (!yaw_reference_initialized_) {
            initialize_yaw_reference(
                latest_reference_.yaw);
        }

        const auto& p = latest_vehicle_state_.position;
        const auto& v = latest_vehicle_state_.velocity;

        const Eigen::Vector3d current_position(
            p.x, p.y, p.z);

        const Eigen::Vector3d current_velocity(
            v.x, v.y, v.z);

        // Position PID: m -> m/s.
        PositionInput position_input;

        position_input.error =
            latest_reference_.position - current_position;

        position_input.error_rate =
            latest_reference_.velocity - current_velocity;

        position_input.dt = dt;

        const auto position_output =
            controller_->update(position_input);

        // Yaw PID: rad -> rad/s.
        YawInput yaw_input;

        // Do not wrap this error.
        yaw_input.error(0) =
            target_unwrapped_yaw_ - unwrapped_yaw_;

        yaw_input.error_rate(0) =
            latest_reference_.yaw_rate -
            measured_yaw_rate_;

        yaw_input.dt = dt;

        const auto yaw_output =
            yaw_controller_.update(yaw_input);

        if (!position_output.command.allFinite() ||
            !yaw_output.command.allFinite()) {
            throw std::runtime_error(
                "Non-finite controller output");
        }

        publish_velocity_setpoint(
            position_output.command,
            yaw_output.command(0));

    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "Control update failed: %s",
            e.what());

        enter_fault(
            ControlStatus::FAULT_CONTROLLER_ERROR);
    }
}

void FlightController::status_loop()
{
    ControlStatus status{};

    status.stamp = now();
    status.trajectory_id =
        trajectory_manager_.trajectory_id();

    status.state = control_state_;
    status.fault_code = fault_code_;

    if (trajectory_manager_.has_trajectory()) {
        status.trajectory_time =
            trajectory_manager_.elapsed(now());
    }

    if (has_valid_state_) {
        status.actual_position =
            latest_vehicle_state_.position;

        status.actual_velocity =
            latest_vehicle_state_.velocity;
    }

    if (has_reference_ && has_valid_state_) {
        status.reference_position.x =
            latest_reference_.position.x();
        status.reference_position.y =
            latest_reference_.position.y();
        status.reference_position.z =
            latest_reference_.position.z();

        status.reference_velocity.x =
            latest_reference_.velocity.x();
        status.reference_velocity.y =
            latest_reference_.velocity.y();
        status.reference_velocity.z =
            latest_reference_.velocity.z();

        const Eigen::Vector3d p(
            status.actual_position.x,
            status.actual_position.y,
            status.actual_position.z);

        const Eigen::Vector3d v(
            status.actual_velocity.x,
            status.actual_velocity.y,
            status.actual_velocity.z);

        const Eigen::Vector3d ep =
            latest_reference_.position - p;

        const Eigen::Vector3d ev =
            latest_reference_.velocity - v;

        status.position_error.x = ep.x();
        status.position_error.y = ep.y();
        status.position_error.z = ep.z();

        status.velocity_error.x = ev.x();
        status.velocity_error.y = ev.y();
        status.velocity_error.z = ev.z();

        status.position_error_norm = ep.norm();
        status.velocity_error_norm = ev.norm();
    }

    control_status_pub_->publish(status);
}