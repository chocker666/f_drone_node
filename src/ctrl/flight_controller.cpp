
#include "drone_node/ctrl/flight_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr double RAD_TO_DEG =
    180.0 / 3.14159265358979323846;

bool positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

std::chrono::nanoseconds timer_period(double frequency)
{
    if (!positive(frequency)) {
        throw std::invalid_argument(
            "Invalid timer frequency");
    }

    const double nanoseconds = 1e9 / frequency;

    if (!std::isfinite(nanoseconds) ||
        nanoseconds < 1.0 ||
        nanoseconds >= static_cast<double>(
            std::numeric_limits<int64_t>::max())) {
        throw std::invalid_argument(
            "Timer period is outside the supported range");
    }

    return std::chrono::nanoseconds(
        static_cast<int64_t>(std::ceil(nanoseconds)));
}

drone_node::algorithm::PIDParameters load_pid_parameters(
    rclcpp::Node& node,
    const std::string& prefix,
    drone_node::algorithm::PIDLimitMode default_mode)
{
    using Mode = drone_node::algorithm::PIDLimitMode;

    drone_node::algorithm::PIDParameters parameters;

    parameters.kp = node.declare_parameter<double>(
        prefix + ".kp", parameters.kp);

    parameters.ki = node.declare_parameter<double>(
        prefix + ".ki", parameters.ki);

    parameters.kd = node.declare_parameter<double>(
        prefix + ".kd", parameters.kd);

    parameters.max_output = node.declare_parameter<double>(
        prefix + ".max_output",
        parameters.max_output);

    parameters.max_output_rate = node.declare_parameter<double>(
        prefix + ".max_output_rate",
        parameters.max_output_rate);

    parameters.integral_limit = node.declare_parameter<double>(
        prefix + ".integral_limit",
        parameters.integral_limit);

    const std::string mode =
        node.declare_parameter<std::string>(
            prefix + ".limit_mode",
            default_mode == Mode::NORM
                ? "norm"
                : "per_axis");

    if (mode == "norm") {
        parameters.limit_mode = Mode::NORM;
    } else if (mode == "per_axis") {
        parameters.limit_mode = Mode::PER_AXIS;
    } else {
        throw std::invalid_argument(
            "Unsupported PID limit_mode at " + prefix);
    }

    return parameters;
}

drone_node::PhysicalControllerInput make_input(
    const drone_node::StateSnapshot& state,
    const drone_node::ReferenceState& reference,
    double dt,
    const drone_node::ReferenceValidity& validity =
        {true, true, true, true, true})
{
    drone_node::PhysicalControllerInput input;

    input.state = state;
    input.reference = reference;
    input.reference_validity = validity;
    input.dt = dt;

    return input;
}

}  // namespace

// ============================================================
// Constructor
// ============================================================

FlightController::FlightController(
    const rclcpp::NodeOptions& options)
    : Node("flight_controller", options)
{
    control_frequency_ = declare_parameter<double>(
        "control_frequency", 100.0);

    status_frequency_ = declare_parameter<double>(
        "status_frequency", 10.0);

    state_timeout_ = declare_parameter<double>(
        "state_timeout", 0.2);

    hover_config_.speed_threshold =
        declare_parameter<double>(
            "hover.speed_threshold", 0.1);

    hover_config_.yaw_rate_threshold =
        declare_parameter<double>(
            "hover.yaw_rate_threshold", 0.1);

    hover_config_.settle_duration =
        declare_parameter<double>(
            "hover.settle_duration", 0.3);

    if (!positive(control_frequency_) ||
        !positive(status_frequency_) ||
        status_frequency_ > 20.0 ||
        !positive(state_timeout_)) {
        throw std::invalid_argument(
            "Invalid scheduling or timeout parameters");
    }

    load_controller_configs();

    // Validate default controller and HOVER configuration.
    {
        auto checked_controller =
            drone_node::controller::ControllerFactory::create(
                "position_yaw_pid",
                controller_config_);

        const drone_node::HoverReference checked_hover(
            hover_config_);

        (void)checked_controller;
        (void)checked_hover;
    }

    state_cache_ =
        std::make_unique<drone_node::StateCache>(
            state_timeout_);

    const auto feedback_qos =
        rclcpp::QoS(1).reliable();

    position_sub_ =
        create_subscription<PointStamped>(
            "/px4_interface/out/position",
            feedback_qos,
            std::bind(
                &FlightController::position_callback,
                this,
                std::placeholders::_1));

    velocity_sub_ =
        create_subscription<VectorStamped>(
            "/px4_interface/out/velocity",
            feedback_qos,
            std::bind(
                &FlightController::velocity_callback,
                this,
                std::placeholders::_1));

    attitude_sub_ =
        create_subscription<QuaternionStamped>(
            "/px4_interface/out/attitude",
            feedback_qos,
            std::bind(
                &FlightController::attitude_callback,
                this,
                std::placeholders::_1));

    // Unified polynomial trajectory input.
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

    setpoint_pub_ =
        create_publisher<Setpoint>(
            "/px4_interface/in/setpoint",
            rclcpp::QoS(1).reliable());

    control_status_pub_ =
        create_publisher<ControlStatus>(
            "/flight_controller/out/control_status",
            rclcpp::QoS(10).reliable());

    set_control_timer(control_frequency_);

    status_timer_ = create_wall_timer(
        timer_period(status_frequency_),
        std::bind(
            &FlightController::status_loop,
            this));

    RCLCPP_INFO(
        get_logger(),
        "FlightController started: control=%.1f Hz, "
        "status=%.1f Hz, state=IDLE",
        active_control_frequency_,
        status_frequency_);
}

// ============================================================
// Timer management
// ============================================================

void FlightController::set_control_timer(double frequency)
{
    const auto period = timer_period(frequency);

    auto next_timer = create_wall_timer(
        period,
        std::bind(
            &FlightController::control_loop,
            this));

    if (control_timer_) {
        control_timer_->cancel();
    }

    control_timer_ = std::move(next_timer);

    active_control_frequency_ = frequency;
    consecutive_overruns_ = 0;
}

// ============================================================
// State feedback callbacks
// ============================================================

void FlightController::position_callback(
    const PointStamped::SharedPtr msg)
{
    const auto& p = msg->point;

    state_cache_->update_position(
        Eigen::Vector3d(p.x, p.y, p.z),
        SteadyClock::now());
}

void FlightController::velocity_callback(
    const VectorStamped::SharedPtr msg)
{
    const auto& v = msg->vector;

    state_cache_->update_velocity(
        Eigen::Vector3d(v.x, v.y, v.z),
        SteadyClock::now());
}

void FlightController::attitude_callback(
    const QuaternionStamped::SharedPtr msg)
{
    const auto& q = msg->quaternion;

    const auto result = state_cache_->update_attitude(
        Eigen::Quaterniond(q.w, q.x, q.y, q.z),
        SteadyClock::now());

    if (result ==
            drone_node::StateCache::AttitudeResult::YAW_DISCONTINUITY &&
        (control_state_ == ControlStatus::TRACKING ||
         control_state_ == ControlStatus::HOVER)) {
        enter_fault(
            ControlStatus::FAULT_INVALID_STATE);
    }
}

// ============================================================
// Control state management
// ============================================================

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

void FlightController::clear_execution()
{
    trajectory_manager_.clear();

    hover_reference_.reset();
    controller_.reset();

    active_controller_id_.clear();
    active_config_name_.clear();

    has_last_control_time_ = false;

    position_error_ =
        std::numeric_limits<double>::quiet_NaN();

    yaw_error_ =
        std::numeric_limits<double>::quiet_NaN();
}

void FlightController::enter_idle()
{
    clear_execution();

    fault_code_ = ControlStatus::FAULT_NONE;

    change_control_state(ControlStatus::IDLE);
}

void FlightController::enter_fault(uint16_t fault_code)
{
    if (control_state_ == ControlStatus::FAULT) {
        return;
    }

    clear_execution();

    fault_code_ = fault_code;

    change_control_state(ControlStatus::FAULT);

    RCLCPP_ERROR(
        get_logger(),
        "FlightController fault: %u",
        static_cast<unsigned>(fault_code_));
}

void FlightController::enter_hover()
{
    if (control_state_ != ControlStatus::TRACKING) {
        throw std::logic_error(
            "HOVER is only reachable from TRACKING");
    }

    const auto state =
        state_cache_->snapshot(SteadyClock::now());

    if (!drone_node::StateCache::usable(state)) {
        throw std::runtime_error(
            "CANCEL requires fresh feedback");
    }

    auto pending_hover =
        std::make_unique<drone_node::HoverReference>(
            hover_config_);

    pending_hover->start(state);

    auto pending_controller =
        drone_node::controller::ControllerFactory::create(
            "position_yaw_pid",
            controller_config_);

    pending_controller->initialize(
        make_input(
            state,
            pending_hover->sample(state),
            1.0 / active_control_frequency_),
        drone_node::controller::YawDirection::SHORTEST);

    // Prepare controller identity before modifying active state.
    std::string next_controller_id = "position_yaw_pid";
    std::string next_config_name = "default";

    clear_execution();

    hover_reference_ = std::move(pending_hover);
    controller_ = std::move(pending_controller);

    active_controller_id_.swap(next_controller_id);
    active_config_name_.swap(next_config_name);

    fault_code_ = ControlStatus::FAULT_NONE;

    change_control_state(ControlStatus::HOVER);
}

// ============================================================
// Control command callback
// ============================================================

void FlightController::command_callback(
    const ControlCommand::SharedPtr msg)
{
    switch (msg->command) {

    case ControlCommand::CANCEL:
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

            enter_fault(
                ControlStatus::FAULT_INVALID_STATE);
        }
        return;

    case ControlCommand::EXIT_IDLE:
        if (control_state_ != ControlStatus::TRACKING) {
            RCLCPP_WARN(
                get_logger(),
                "EXIT_IDLE rejected: not TRACKING");
            return;
        }

        enter_idle();
        return;

    case ControlCommand::RESET_FAULT:
        if (control_state_ != ControlStatus::FAULT) {
            RCLCPP_WARN(
                get_logger(),
                "RESET_FAULT rejected: not FAULT");
            return;
        }

        enter_idle();
        return;

    default:
        RCLCPP_WARN(
            get_logger(),
            "Unknown control command: %u",
            static_cast<unsigned>(msg->command));
        return;
    }
}

// ============================================================
// Unified trajectory callback
// ============================================================

void FlightController::trajectory_callback(
    const ControlTrajectory::SharedPtr msg)
{
    const auto id =
        static_cast<unsigned long>(msg->trajectory_id);

    if (control_state_ == ControlStatus::FAULT) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory %lu rejected: controller in FAULT",
            id);
        return;
    }

    try {
        // ----------------------------------------------------
        // 1. Validate current feedback
        // ----------------------------------------------------

        const auto state_time = SteadyClock::now();

        const auto state =
            state_cache_->snapshot(state_time);

        if (!drone_node::StateCache::usable(state)) {
            throw std::runtime_error(
                "Current state feedback is unavailable");
        }

        // ----------------------------------------------------
        // 2. Prepare trajectory
        // ----------------------------------------------------

        // This validates the polynomial and constructs
        // a candidate sampler without changing the
        // currently active trajectory.
        auto prepared =
            trajectory_manager_.prepare(*msg);

        if (!prepared.sampler) {
            throw std::runtime_error(
                "Prepared trajectory has no sampler");
        }

        // ----------------------------------------------------
        // 3. Resolve controller implementation and profile
        // ----------------------------------------------------

        // Only position_yaw_pid is implemented currently.
        if (msg->controller_id != "position_yaw_pid") {
            throw std::invalid_argument(
                "Unsupported controller_id: " +
                msg->controller_id);
        }

        const auto config_it =
            controller_configs_.find(msg->config_name);

        if (config_it == controller_configs_.end()) {
            throw std::invalid_argument(
                "Unknown controller config_name: " +
                msg->config_name);
        }

        // ----------------------------------------------------
        // 4. Decide whether to reuse controller
        // ----------------------------------------------------

        const bool reuse_controller =
            control_state_ == ControlStatus::TRACKING &&
            controller_ != nullptr &&
            active_controller_id_ == msg->controller_id &&
            active_config_name_ == msg->config_name;

        std::unique_ptr<
            drone_node::controller::ControllerBase>
            candidate_controller;

        if (!reuse_controller) {
            candidate_controller =
                drone_node::controller::ControllerFactory::create(
                    msg->controller_id,
                    config_it->second);

            if (!candidate_controller) {
                throw std::runtime_error(
                    "ControllerFactory returned null");
            }

            // Use the initial reference of the new trajectory
            // to initialize the new physical controller.
            const auto initial =
                prepared.sampler->sample_with_validity(0.0);

            const auto input = make_input(
                state,
                initial.state,
                1.0 / active_control_frequency_,
                initial.validity);

            candidate_controller->initialize(
                input,
                drone_node::controller::YawDirection::SHORTEST);
        }

        // ----------------------------------------------------
        // 5. Prepare controller identity for no-throw commit
        // ----------------------------------------------------

        std::string next_controller_id =
            msg->controller_id;

        std::string next_config_name =
            msg->config_name;

        // ----------------------------------------------------
        // 6. Commit trajectory and controller
        // ----------------------------------------------------

        // This callback must be serialized with control_loop().
        // The default mutually-exclusive ROS callback group
        // provides this when all callbacks use that group.
        const auto activation_time = SteadyClock::now();

        if (!reuse_controller) {
            controller_ = std::move(candidate_controller);
        }

        trajectory_manager_.commit(
            std::move(prepared),
            activation_time);

        active_controller_id_.swap(next_controller_id);
        active_config_name_.swap(next_config_name);

        hover_reference_.reset();

        fault_code_ = ControlStatus::FAULT_NONE;

        change_control_state(ControlStatus::TRACKING);

        RCLCPP_INFO(
            get_logger(),
            "Trajectory %lu accepted: controller=%s, "
            "config=%s, reused=%s",
            id,
            msg->controller_id.c_str(),
            msg->config_name.c_str(),
            reuse_controller ? "true" : "false");

    } catch (const std::exception& e) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory %lu rejected: %s",
            id,
            e.what());
    }
}

// ============================================================
// Setpoint publication
// ============================================================

void FlightController::publish_velocity_setpoint(
    const Eigen::Vector3d& velocity,
    double yaw_rate)
{
    const double degrees_per_second =
        yaw_rate * RAD_TO_DEG;

    if (!velocity.allFinite() ||
        !std::isfinite(degrees_per_second) ||
        std::abs(degrees_per_second) >
            std::numeric_limits<float>::max()) {
        throw std::runtime_error(
            "Setpoint is outside the message numeric range");
    }

    Setpoint setpoint{};

    setpoint.stamp = now();
    setpoint.control_mode = Setpoint::VELOCITY;

    setpoint.velocity.x = velocity.x();
    setpoint.velocity.y = velocity.y();
    setpoint.velocity.z = velocity.z();

    setpoint.yawspeed =
        static_cast<float>(degrees_per_second);

    setpoint_pub_->publish(setpoint);
}

// ============================================================
// Control cycle
// ============================================================

void FlightController::run_control_cycle()
{
    if (control_state_ != ControlStatus::TRACKING &&
        control_state_ != ControlStatus::HOVER) {
        return;
    }

    const auto cycle_time = SteadyClock::now();

    // Exactly one feedback snapshot per control cycle.
    const auto state =
        state_cache_->snapshot(cycle_time);

    if (!drone_node::StateCache::usable(state)) {
        enter_fault(
            ControlStatus::FAULT_STATE_TIMEOUT);
        return;
    }

    double dt = 1.0 / active_control_frequency_;

    if (has_last_control_time_) {
        dt = std::chrono::duration<double>(
            cycle_time - last_control_time_).count();

        dt = std::clamp(
            dt,
            std::min(
                0.001,
                1.0 / active_control_frequency_),
            2.0 / active_control_frequency_);
    }

    last_control_time_ = cycle_time;
    has_last_control_time_ = true;

    try {
        if (!controller_) {
            throw std::runtime_error(
                "Missing physical controller");
        }

        drone_node::ReferenceState reference;

        drone_node::ReferenceValidity validity{
            true, true, true, true, true
        };

        if (control_state_ == ControlStatus::TRACKING) {
            const auto sampled =
                trajectory_manager_.sample_with_validity(
                    cycle_time);

            reference = sampled.state;
            validity = sampled.validity;

        } else {
            if (!hover_reference_) {
                throw std::runtime_error(
                    "Missing HOVER reference");
            }

            reference =
                hover_reference_->sample(state);
        }

        const auto result = controller_->update(
            make_input(
                state,
                reference,
                dt,
                validity));

        if (result.output.mode !=
                drone_node::ControlMode::VELOCITY ||
            !result.output.velocity.allFinite() ||
            !std::isfinite(result.output.yaw_rate) ||
            !std::isfinite(result.errors.position_error) ||
            !std::isfinite(result.errors.yaw_error)) {
            throw std::runtime_error(
                "Invalid physical controller result");
        }

        publish_velocity_setpoint(
            result.output.velocity,
            result.output.yaw_rate);

        position_error_ =
            result.errors.position_error;

        yaw_error_ =
            result.errors.yaw_error;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "Control update failed: %s",
            e.what());

        enter_fault(
            ControlStatus::FAULT_CONTROLLER_ERROR);
    }
}

// ============================================================
// Control timer callback
// ============================================================

void FlightController::control_loop()
{
    const auto begin = SteadyClock::now();

    run_control_cycle();

    if (control_state_ != ControlStatus::TRACKING &&
        control_state_ != ControlStatus::HOVER) {
        consecutive_overruns_ = 0;
        return;
    }

    const double cost =
        std::chrono::duration<double>(
            SteadyClock::now() - begin).count();

    if (cost > 0.8 / active_control_frequency_) {
        ++consecutive_overruns_;
    } else {
        consecutive_overruns_ = 0;
    }

    // Reduce frequency after sustained high computation cost.
    if (consecutive_overruns_ >= 5) {
        const double next_frequency = std::min(
            active_control_frequency_,
            0.8 / cost);

        try {
            set_control_timer(next_frequency);

            RCLCPP_WARN(
                get_logger(),
                "Control frequency reduced to %.2f Hz",
                active_control_frequency_);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                get_logger(),
                "Control scheduling failed: %s",
                e.what());

            enter_fault(
                ControlStatus::FAULT_CONTROLLER_ERROR);
        }
    }
}

// ============================================================
// Status publication
// ============================================================

void FlightController::status_loop()
{
    ControlStatus status{};

    status.stamp = now();

    status.trajectory_id =
        control_state_ == ControlStatus::TRACKING
            ? trajectory_manager_.trajectory_id()
            : 0;

    status.state = control_state_;
    status.fault_code = fault_code_;

    const bool fresh =
        drone_node::StateCache::usable(
            state_cache_->snapshot(SteadyClock::now()));

    status.position_error = fresh
        ? position_error_
        : std::numeric_limits<double>::quiet_NaN();

    status.yaw_error = fresh
        ? yaw_error_
        : std::numeric_limits<double>::quiet_NaN();

    control_status_pub_->publish(status);
}

// ============================================================
// Controller configuration loading
// ============================================================

void FlightController::load_controller_configs()
{
    const auto names =
        declare_parameter<std::vector<std::string>>(
            "controllers.position_yaw_pid.config_names",
            std::vector<std::string>{"default"});

    if (names.empty()) {
        throw std::invalid_argument(
            "No position_yaw_pid configurations defined");
    }

    for (const auto& name : names) {
        if (name.empty() ||
            name.find('.') != std::string::npos) {
            throw std::invalid_argument(
                "Configuration names must be nonempty "
                "and contain no dots");
        }

        if (controller_configs_.find(name) !=
            controller_configs_.end()) {
            throw std::invalid_argument(
                "Duplicate controller configuration: " +
                name);
        }

        const std::string prefix =
            "controllers.position_yaw_pid.configs." + name;

        drone_node::controller::PositionYawPIDConfig config;

        config.position = load_pid_parameters(
            *this,
            prefix + ".position_pid",
            drone_node::algorithm::PIDLimitMode::NORM);

        config.yaw = load_pid_parameters(
            *this,
            prefix + ".yaw_pid",
            drone_node::algorithm::PIDLimitMode::PER_AXIS);

        // Validate every profile during startup.
        auto checked =
            drone_node::controller::ControllerFactory::create(
                "position_yaw_pid",
                config);

        (void)checked;

        controller_configs_.emplace(
            name,
            config);
    }

    const auto default_config =
        controller_configs_.find("default");

    if (default_config == controller_configs_.end()) {
        throw std::invalid_argument(
            "The default controller configuration is required");
    }

    controller_config_ = default_config->second;
}
