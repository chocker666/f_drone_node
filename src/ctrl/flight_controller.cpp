
#include "drone_node/ctrl/flight_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>


namespace
{

drone_node::algorithm::PIDParameters load_pid_parameters(
    rclcpp::Node& node,
    const std::string& prefix,
    drone_node::algorithm::PIDLimitMode limit_mode)
{
    drone_node::algorithm::PIDParameters params;

    params.kp = node.declare_parameter<double>(
        prefix + ".kp", params.kp);

    params.ki = node.declare_parameter<double>(
        prefix + ".ki", params.ki);

    params.kd = node.declare_parameter<double>(
        prefix + ".kd", params.kd);

    params.max_output = node.declare_parameter<double>(
        prefix + ".max_output", params.max_output);

    params.max_output_rate = node.declare_parameter<double>(
        prefix + ".max_output_rate",
        params.max_output_rate);

    params.integral_limit = node.declare_parameter<double>(
        prefix + ".integral_limit",
        params.integral_limit);

    params.limit_mode = limit_mode;
    return params;
}

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double ANGLE_EPS = 1e-10;

}  // namespace

FlightController::FlightController(
    const rclcpp::NodeOptions& options)
    : Node("flight_controller", options)
{
    control_frequency_ = declare_parameter<double>(
        "control_frequency", 50.0);

    status_frequency_ = declare_parameter<double>(
        "status_frequency", 10.0);

    state_timeout_ = declare_parameter<double>(
        "state_timeout", 0.2);

    enable_setpoint_stream_ = declare_parameter<bool>(
        "enable_setpoint_stream", false);

    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };

    if (!positive(control_frequency_) ||
        !positive(status_frequency_) ||
        !positive(state_timeout_)) {
        throw std::runtime_error(
            "Invalid FlightController parameters");
    }

    const auto position_pid_params = load_pid_parameters(
        *this,
        "position_pid",
        drone_node::algorithm::PIDLimitMode::NORM);

    const auto yaw_pid_params = load_pid_parameters(
        *this,
        "yaw_pid",
        drone_node::algorithm::PIDLimitMode::PER_AXIS);

    controller_config_.position = position_pid_params;
    controller_config_.yaw = yaw_pid_params;

    controller_ = drone_node::controller::ControllerFactory::create(
        "position_yaw_pid", controller_config_);

    RCLCPP_INFO(
        get_logger(),
        "Position PID: kp=%.3f ki=%.3f kd=%.3f "
        "max_output=%.3f max_output_rate=%.3f",
        position_pid_params.kp,
        position_pid_params.ki,
        position_pid_params.kd,
        position_pid_params.max_output,
        position_pid_params.max_output_rate);

    RCLCPP_INFO(
        get_logger(),
        "Yaw PID: kp=%.3f ki=%.3f kd=%.3f "
        "max_output=%.3f max_output_rate=%.3f",
        yaw_pid_params.kp,
        yaw_pid_params.ki,
        yaw_pid_params.kd,
        yaw_pid_params.max_output,
        yaw_pid_params.max_output_rate);

    const auto feedback_qos = rclcpp::QoS(1).reliable();

    position_sub_ = create_subscription<PointStamped>(
        "/px4_interface/out/position", feedback_qos,
        std::bind(&FlightController::position_callback,
                this, std::placeholders::_1));

    velocity_sub_ = create_subscription<VectorStamped>(
        "/px4_interface/out/velocity", feedback_qos,
        std::bind(&FlightController::velocity_callback,
                this, std::placeholders::_1));

    attitude_sub_ = create_subscription<QuaternionStamped>(
        "/px4_interface/out/attitude", feedback_qos,
        std::bind(&FlightController::attitude_callback,
                this, std::placeholders::_1));

    trajectory_sub_ =
        create_subscription<ControlTrajectory>(
            "/flight_controller/in/trajectory",
            rclcpp::QoS(1).reliable(),
            std::bind(
                &FlightController::trajectory_callback,
                this,
                std::placeholders::_1));

    command_sub_ =
        create_subscription<ControlCommand>(
            "/flight_controller/in/command",
            rclcpp::QoS(10).reliable(),
            std::bind(
                &FlightController::command_callback,
                this,
                std::placeholders::_1));

    output_enable_sub_ =
        create_subscription<std_msgs::msg::Bool>(
            "/flight_controller/in/enable_setpoint_stream",
            rclcpp::QoS(10).reliable(),
            std::bind(
                &FlightController::output_enable_callback,
                this,
                std::placeholders::_1));

    armed_sub_ =
        create_subscription<std_msgs::msg::Bool>(
            "/px4_interface/out/arm",
            rclcpp::QoS(10),
            std::bind(
                &FlightController::armed_callback,
                this,
                std::placeholders::_1));

    trajectory_setpoint_pub_ = create_publisher<Setpoint>(
        "/px4_interface/in/setpoint", rclcpp::QoS(1).reliable());

    control_status_pub_ =
        create_publisher<ControlStatus>(
            "/flight_controller/out/control_status",
            rclcpp::QoS(10).reliable());

    const auto control_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                1.0 / control_frequency_));

    const auto status_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                1.0 / status_frequency_));

    control_timer_ = create_wall_timer(
        control_period,
        std::bind(&FlightController::control_loop, this));

    status_timer_ = create_wall_timer(
        status_period,
        std::bind(&FlightController::status_loop, this));

    RCLCPP_INFO(
        get_logger(),
        "FlightController started: control=%.1f Hz, "
        "status=%.1f Hz, state=IDLE, output=%s",
        control_frequency_,
        status_frequency_,
        enable_setpoint_stream_ ? "enabled" : "disabled");
}

double FlightController::wrap_pi(double angle)
{
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
            receive_time - last_yaw_receive_time_).count();

    if (dt <= 0.0) {
        return;
    }

    const double delta =
        wrap_pi(measured_yaw - previous_wrapped_yaw_);

    if (dt > state_timeout_ ||
        std::abs(delta) >= PI - ANGLE_EPS) {

        has_yaw_measurement_ = false;
        measured_yaw_rate_ = 0.0;

        if (control_state_ == ControlStatus::TRACKING ||
            control_state_ == ControlStatus::HOVER) {
            enter_fault(ControlStatus::FAULT_INVALID_STATE);
        }

        return;
    }

    unwrapped_yaw_ += delta;
    measured_yaw_rate_ = delta / dt;

    previous_wrapped_yaw_ = measured_yaw;
    last_yaw_receive_time_ = receive_time;
}

void FlightController::position_callback(const PointStamped::SharedPtr msg)
{
    const auto &p = msg->point;
    if (!std::isfinite(p.x) ||
        !std::isfinite(p.y) ||
        !std::isfinite(p.z))
        return;

    latest_vehicle_state_.position = p;
    feedback_received_[0] = true;
    feedback_time_[0] = SteadyClock::now();
}

void FlightController::velocity_callback(const VectorStamped::SharedPtr msg)
{
    const auto &v = msg->vector;
    if (!std::isfinite(v.x) ||
        !std::isfinite(v.y) ||
        !std::isfinite(v.z))
        return;

    latest_vehicle_state_.velocity = v;
    feedback_received_[1] = true;
    feedback_time_[1] = SteadyClock::now();
}

void FlightController::attitude_callback(const QuaternionStamped::SharedPtr msg)
{
    auto q = msg->quaternion;
    const double norm2 =
        q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!std::isfinite(norm2) || norm2 <= 1e-12)
        return;

    const double norm = std::sqrt(norm2);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;

    latest_vehicle_state_.orientation = q;
    feedback_received_[2] = true;
    feedback_time_[2] = SteadyClock::now();
    update_unwrapped_yaw(yaw_from_quaternion(q));
}

bool FlightController::state_is_fresh() const
{
    if (!has_yaw_measurement_)
        return false;

    const auto now = SteadyClock::now();
    for (size_t i = 0; i < feedback_received_.size(); ++i) {
        if (!feedback_received_[i] ||
            std::chrono::duration<double>(
                now - feedback_time_[i]).count() > state_timeout_)
            return false;
    }
    return true;
}

FlightController::FixedTrajectory
FlightController::make_current_fixed_reference() const
{
    if (!state_is_fresh()) {
        throw std::runtime_error(
            "Cannot create fixed reference: stale vehicle state");
    }

    FixedTrajectory fixed{};

    fixed.position = latest_vehicle_state_.position;
    fixed.yaw = yaw_from_quaternion(
        latest_vehicle_state_.orientation);

    fixed.yaw_direction = FixedTrajectory::SHORTEST;

    return fixed;
}

void FlightController::reset_control_runtime()
{
    if (controller_) {
        controller_->reset();
    }

    controller_initialized_ = false;
    has_last_control_time_ = false;
    has_reference_ = false;

    position_error_ = std::numeric_limits<double>::quiet_NaN();
    yaw_error_ = std::numeric_limits<double>::quiet_NaN();
}

drone_node::PhysicalControllerInput
FlightController::make_controller_input(
    const drone_node::ReferenceState& reference,
    double dt) const
{
    drone_node::PhysicalControllerInput input;
    input.reference = reference;
    input.reference_validity = {true, true, true, true, true};
    input.dt = dt;

    auto& state = input.state;
    state.captured_at = SteadyClock::now();

    const auto& p = latest_vehicle_state_.position;
    const auto& v = latest_vehicle_state_.velocity;
    const auto& q = latest_vehicle_state_.orientation;

    state.position = Eigen::Vector3d(p.x, p.y, p.z);
    state.velocity = Eigen::Vector3d(v.x, v.y, v.z);
    state.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);

    state.position_feedback = {
        feedback_received_[0], feedback_time_[0]};
    state.velocity_feedback = {
        feedback_received_[1], feedback_time_[1]};
    state.attitude_feedback = {
        feedback_received_[2], feedback_time_[2]};

    state.unwrapped_yaw = unwrapped_yaw_;
    state.measured_yaw_rate = measured_yaw_rate_;
    state.yaw_valid = has_yaw_measurement_;

    return input;
}

void FlightController::change_control_state(uint8_t state)
{
    if (control_state_ == state) {
        return;
    }

    const auto previous = control_state_;
    control_state_ = state;

    RCLCPP_INFO(
        get_logger(),
        "Control state: %u -> %u",
        static_cast<unsigned>(previous),
        static_cast<unsigned>(state));
}

void FlightController::enter_hover()
{
    if (control_state_ != ControlStatus::TRACKING) {
        throw std::logic_error(
            "HOVER is only reachable from TRACKING");
    }

    // Capture the actual pose once. Do not update it each cycle.
    auto new_sampler = std::make_unique<FixedSampler>(
        make_current_fixed_reference());

    hover_sampler_ = std::move(new_sampler);
    trajectory_manager_.clear();

    reset_control_runtime();

    fault_code_ = ControlStatus::FAULT_NONE;
    change_control_state(ControlStatus::HOVER);

    RCLCPP_INFO(
        get_logger(),
        "Entered HOVER at x=%.3f y=%.3f z=%.3f",
        latest_vehicle_state_.position.x,
        latest_vehicle_state_.position.y,
        latest_vehicle_state_.position.z);
}

void FlightController::enter_idle()
{
    trajectory_manager_.clear();
    hover_sampler_.reset();

    reset_control_runtime();

    enable_setpoint_stream_ = false;
    fault_code_ = ControlStatus::FAULT_NONE;

    change_control_state(ControlStatus::IDLE);
}

void FlightController::enter_fault(uint16_t fault_code)
{
    if (control_state_ == ControlStatus::FAULT) {
        return;
    }

    fault_code_ = fault_code;

    trajectory_manager_.clear();
    hover_sampler_.reset();

    reset_control_runtime();
    enable_setpoint_stream_ = false;

    change_control_state(ControlStatus::FAULT);

    RCLCPP_ERROR(
        get_logger(),
        "FlightController fault: %u",
        static_cast<unsigned>(fault_code));
}

void FlightController::armed_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    armed_ = msg->data;
    has_armed_feedback_ = true;
}

void FlightController::output_enable_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data &&
        (control_state_ == ControlStatus::IDLE ||
         control_state_ == ControlStatus::FAULT)) {

        RCLCPP_WARN(
            get_logger(),
            "Cannot enable output in IDLE or FAULT");
        return;
    }

    if (enable_setpoint_stream_ == msg->data) {
        return;
    }

    enable_setpoint_stream_ = msg->data;
    reset_control_runtime();

    RCLCPP_INFO(
        get_logger(),
        "Setpoint output %s",
        enable_setpoint_stream_ ? "enabled" : "disabled");
}

void FlightController::command_callback(
    const ControlCommand::SharedPtr msg)
{
    switch (msg->command) {

    case ControlCommand::CANCEL:
    {
        if (control_state_ != ControlStatus::TRACKING) {
            RCLCPP_WARN(
                get_logger(),
                "CANCEL rejected: not TRACKING");
            return;
        }

        try {
            enter_hover();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                get_logger(),
                "CANCEL failed: %s",
                e.what());

            enter_fault(ControlStatus::FAULT_INVALID_STATE);
        }
        return;
    }

    case ControlCommand::EXIT_IDLE:
    {
        if (control_state_ != ControlStatus::TRACKING) {
            RCLCPP_WARN(
                get_logger(),
                "EXIT_IDLE rejected: not TRACKING");
            return;
        }

        if (!has_armed_feedback_ || armed_) {
            RCLCPP_WARN(
                get_logger(),
                "EXIT_IDLE rejected: disarm not confirmed");
            return;
        }

        enter_idle();
        return;
    }

    case ControlCommand::RESET_FAULT:
    {
        if (control_state_ != ControlStatus::FAULT) {
            RCLCPP_WARN(
                get_logger(),
                "RESET_FAULT rejected: not FAULT");
            return;
        }

        if (!has_armed_feedback_ || armed_) {
            RCLCPP_WARN(
                get_logger(),
                "RESET_FAULT rejected: disarm not confirmed");
            return;
        }

        enter_idle();
        return;
    }

    default:
        RCLCPP_WARN(
            get_logger(),
            "Unknown control command: %u",
            static_cast<unsigned>(msg->command));
        return;
    }
}

void FlightController::trajectory_callback(
    const ControlTrajectory::SharedPtr msg)
{
    if (control_state_ == ControlStatus::FAULT) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: controller in FAULT");
        return;
    }

    if (!state_is_fresh()) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: no fresh vehicle state");
        return;
    }

    if (msg->trajectory_type != ControlTrajectory::POINT ||
        msg->config_name != "default") {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: unsupported type or config");
        return;
    }

    try {
        // Prepare and initialize the candidate before replacement.
        const FixedSampler initial_sampler(msg->point_params);
        const auto input = make_controller_input(
            initial_sampler.sample(0.0),
            1.0 / control_frequency_);

        auto candidate =
            drone_node::controller::ControllerFactory::create(
                msg->controller_id, controller_config_);

        candidate->initialize(
            input,
            static_cast<drone_node::controller::YawDirection>(
                msg->point_params.yaw_direction));

        std::string reason;
        const auto result = trajectory_manager_.accept(
            *msg, now(), reason);

        if (result ==
            drone_node::TrajectoryManager::AcceptResult::REJECTED) {
            RCLCPP_WARN(
                get_logger(),
                "Trajectory %lu rejected: %s",
                static_cast<unsigned long>(msg->trajectory_id),
                reason.c_str());
            return;
        }

        hover_sampler_.reset();
        reset_control_runtime();

        controller_ = std::move(candidate);
        controller_initialized_ = true;

        fault_code_ = ControlStatus::FAULT_NONE;
        change_control_state(ControlStatus::TRACKING);

        RCLCPP_INFO(
            get_logger(),
            "Trajectory %lu accepted",
            static_cast<unsigned long>(
                trajectory_manager_.trajectory_id()));
    } catch (const std::exception& e) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory preparation rejected: %s",
            e.what());
    }
}

void FlightController::publish_velocity_setpoint(
    const Eigen::Vector3d& velocity,
    double yaw_rate)
{
    Setpoint setpoint{};
    setpoint.stamp = now();
    setpoint.control_mode = Setpoint::VELOCITY;

    setpoint.velocity.x = velocity.x();
    setpoint.velocity.y = velocity.y();
    setpoint.velocity.z = velocity.z();

    // Internal rad/s -> ROS message deg/s.
    setpoint.yawspeed =
        static_cast<float>(yaw_rate * RAD_TO_DEG);

    trajectory_setpoint_pub_->publish(setpoint);
}

void FlightController::control_loop()
{
    if (control_state_ == ControlStatus::IDLE ||
        control_state_ == ControlStatus::FAULT) {
        return;
    }

    // Removed in the FSM migration stage.
    if (!enable_setpoint_stream_) {
        return;
    }

    const auto steady_now = SteadyClock::now();
    const auto ros_now = now();

    if (!state_is_fresh()) {
        enter_fault(ControlStatus::FAULT_STATE_TIMEOUT);
        return;
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
        if (!controller_) {
            throw std::runtime_error(
                "Missing physical controller");
        }

        auto direction =
            drone_node::controller::YawDirection::SHORTEST;

        if (control_state_ == ControlStatus::TRACKING) {
            if (!trajectory_manager_.has_trajectory()) {
                throw std::runtime_error(
                    "TRACKING without an active trajectory");
            }

            latest_reference_ = trajectory_manager_.sample(ros_now);
            direction =
                static_cast<drone_node::controller::YawDirection>(
                    trajectory_manager_.yaw_direction());
        } else if (control_state_ == ControlStatus::HOVER) {
            if (!hover_sampler_) {
                throw std::runtime_error(
                    "HOVER without reference sampler");
            }

            latest_reference_ = hover_sampler_->sample(0.0);
        } else {
            return;
        }

        has_reference_ = true;

        const auto input = make_controller_input(
            latest_reference_, dt);

        if (!controller_initialized_) {
            controller_->initialize(input, direction);
            controller_initialized_ = true;
        }

        const auto result = controller_->update(input);

        if (result.output.mode != drone_node::ControlMode::VELOCITY ||
            !result.output.velocity.allFinite() ||
            !std::isfinite(result.output.yaw_rate) ||
            !std::isfinite(result.errors.position_error) ||
            !std::isfinite(result.errors.yaw_error)) {
            throw std::runtime_error(
                "Invalid physical controller result");
        }

        position_error_ = result.errors.position_error;
        yaw_error_ = result.errors.yaw_error;

        publish_velocity_setpoint(
            result.output.velocity,
            result.output.yaw_rate);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "Control update failed: %s",
            e.what());

        enter_fault(ControlStatus::FAULT_CONTROLLER_ERROR);
    }
}

void FlightController::status_loop()
{
    ControlStatus status{};

    status.stamp = now();

    status.trajectory_id =
        control_state_ == ControlStatus::TRACKING ?
        trajectory_manager_.trajectory_id() : 0;

    status.state = control_state_;
    status.fault_code = fault_code_;

    status.position_error = position_error_;
    status.yaw_error = yaw_error_;

    control_status_pub_->publish(status);
}
