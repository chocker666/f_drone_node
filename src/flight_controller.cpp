
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

    // Default PID parameters are only used for
    // initial controller construction.
    // Each accepted trajectory supplies its own gains.
    PIDParameters defaults;

    controller_ =
        std::make_unique<PIDController>(defaults);

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
}

bool FlightController::state_is_fresh() const
{
    if (!has_valid_state_) {
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

    const auto result = trajectory_manager_.accept(
        *msg, now(), reason);

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
        RCLCPP_DEBUG(
            get_logger(),
            "Duplicate trajectory %lu ignored",
            static_cast<unsigned long>(
                msg->trajectory_id));
        return;
    }

    // Apply the parameters of the accepted trajectory.
    // The PID implementation and its mathematical
    // update rule remain unchanged.
    auto* pid =
        dynamic_cast<PIDController*>(controller_.get());

    if (!pid) {
        enter_fault(
            ControlStatus::FAULT_CONTROLLER_ERROR);
        return;
    }

    pid->set_parameters(
        trajectory_manager_.pid_parameters());

    has_last_control_time_ = false;
    has_reference_ = false;

    fault_code_ = ControlStatus::FAULT_NONE;

    change_control_state(
        trajectory_manager_.is_started(now()) ?
            ControlStatus::TRACKING :
            ControlStatus::WAITING);

    RCLCPP_INFO(
        get_logger(),
        "Trajectory %lu accepted, PID "
        "kp=%.3f ki=%.3f kd=%.3f",
        static_cast<unsigned long>(
            trajectory_manager_.trajectory_id()),
        trajectory_manager_.pid_parameters().kp,
        trajectory_manager_.pid_parameters().ki,
        trajectory_manager_.pid_parameters().kd);
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

    has_reference_ = false;
    has_last_control_time_ = false;

    change_control_state(ControlStatus::FAULT);

    RCLCPP_ERROR(
        get_logger(),
        "FlightController fault: %u",
        static_cast<unsigned int>(fault_code));
}

void FlightController::publish_velocity_setpoint(
    const Eigen::Vector3d& velocity)
{
    TrajectorySetpoint setpoint{};

    setpoint.stamp = now();
    setpoint.control_mode = TrajectorySetpoint::VELOCITY;

    setpoint.velocity.x = velocity.x();
    setpoint.velocity.y = velocity.y();
    setpoint.velocity.z = velocity.z();

    // Keep existing behavior: no yaw-rate control.
    setpoint.yawspeed = 0.0f;

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

        const auto& p = latest_vehicle_state_.position;
        const auto& v = latest_vehicle_state_.velocity;

        ControllerInput input;

        input.position = Eigen::Vector3d(
            p.x, p.y, p.z);

        input.velocity = Eigen::Vector3d(
            v.x, v.y, v.z);

        input.target_position =
            latest_reference_.position;

        input.dt = dt;

        const auto output =
            controller_->update(input);

        if (!output.velocity_setpoint.allFinite()) {
            throw std::runtime_error(
                "Non-finite controller output");
        }

        publish_velocity_setpoint(
            output.velocity_setpoint);

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
