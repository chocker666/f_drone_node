
#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "drone_node/msg/setpoint.hpp"
#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/msg/control_status.hpp"
#include "drone_node/msg/control_command.hpp"

#include "drone_node/ctrl/controller/controller_factory.hpp"
#include "drone_node/ctrl/trajectory_manager.hpp"
#include "drone_node/ctrl/state_cache.hpp"
#include "drone_node/ctrl/hover_reference.hpp"

class FlightController : public rclcpp::Node
{
public:
    explicit FlightController(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ROS message types.
    using Setpoint = drone_node::msg::Setpoint;
    using PointStamped = geometry_msgs::msg::PointStamped;
    using VectorStamped = geometry_msgs::msg::Vector3Stamped;
    using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
    using ControlTrajectory = drone_node::msg::ControlTrajectory;
    using ControlStatus = drone_node::msg::ControlStatus;
    using ControlCommand = drone_node::msg::ControlCommand;

    using SteadyClock = drone_node::SteadyClock;

    // ---------------------------------------------------------
    // ROS callbacks
    // ---------------------------------------------------------

    void position_callback(
        const PointStamped::SharedPtr msg);

    void velocity_callback(
        const VectorStamped::SharedPtr msg);

    void attitude_callback(
        const QuaternionStamped::SharedPtr msg);

    // Unified trajectory input.
    // Accepts a piecewise quintic polynomial trajectory and
    // its requested physical controller configuration.
    //
    // The new trajectory is either activated immediately
    // after successful preparation, or rejected entirely.
    // A rejected trajectory must not affect active execution.
    void trajectory_callback(
        const ControlTrajectory::SharedPtr msg);

    void command_callback(
        const ControlCommand::SharedPtr msg);

    // ---------------------------------------------------------
    // Control scheduling
    // ---------------------------------------------------------

    void control_loop();

    void run_control_cycle();

    void status_loop();

    void set_control_timer(double frequency);

    // ---------------------------------------------------------
    // Control output
    // ---------------------------------------------------------

    void publish_velocity_setpoint(
        const Eigen::Vector3d& velocity,
        double yaw_rate);

    // ---------------------------------------------------------
    // Control state management
    // ---------------------------------------------------------

    void change_control_state(uint8_t state);

    void enter_hover();

    void enter_idle();

    void enter_fault(uint16_t fault_code);

    void clear_execution();

    // ---------------------------------------------------------
    // Controller configuration
    // ---------------------------------------------------------

    void load_controller_configs();

    // ---------------------------------------------------------
    // Scheduling parameters
    // ---------------------------------------------------------

    double control_frequency_{100.0};

    double active_control_frequency_{100.0};

    double status_frequency_{10.0};

    double state_timeout_{0.2};

    unsigned consecutive_overruns_{0};

    // ---------------------------------------------------------
    // Physical controller configurations
    // ---------------------------------------------------------

    // Default PID configuration.
    // Also used by the existing HOVER implementation.
    drone_node::controller::PositionYawPIDConfig
        controller_config_{};

    // Named PID parameter profiles.
    std::map<
        std::string,
        drone_node::controller::PositionYawPIDConfig>
        controller_configs_;

    // HOVER reference generator configuration.
    drone_node::HoverReference::Config hover_config_{};

    // ---------------------------------------------------------
    // Runtime objects
    // ---------------------------------------------------------

    std::unique_ptr<drone_node::StateCache>
        state_cache_;

    // Active physical controller instance.
    std::unique_ptr<
        drone_node::controller::ControllerBase>
        controller_;

    // Identity of the active physical controller.
    //
    // When controller_id and config_name are unchanged
    // during TRACKING, the existing controller instance
    // can be reused without resetting its internal state.
    //
    // When either changes, a new controller instance
    // must be prepared before the trajectory is committed.
    std::string active_controller_id_;

    std::string active_config_name_;

    // HOVER reference generator.
    std::unique_ptr<drone_node::HoverReference>
        hover_reference_;

    // Active trajectory manager.
    // Owns the PolynomialSampler and trajectory timing.
    drone_node::TrajectoryManager trajectory_manager_;

    // ---------------------------------------------------------
    // Control state and fault status
    // ---------------------------------------------------------

    uint8_t control_state_{ControlStatus::IDLE};

    uint16_t fault_code_{ControlStatus::FAULT_NONE};

    double position_error_{
        std::numeric_limits<double>::quiet_NaN()};

    double yaw_error_{
        std::numeric_limits<double>::quiet_NaN()};

    // ---------------------------------------------------------
    // Control timing
    // ---------------------------------------------------------

    drone_node::SteadyTime last_control_time_{};

    bool has_last_control_time_{false};

    // ---------------------------------------------------------
    // ROS subscriptions
    // ---------------------------------------------------------

    rclcpp::Subscription<PointStamped>::SharedPtr
        position_sub_;

    rclcpp::Subscription<VectorStamped>::SharedPtr
        velocity_sub_;

    rclcpp::Subscription<QuaternionStamped>::SharedPtr
        attitude_sub_;

    // Unified ControlTrajectory subscription.
    rclcpp::Subscription<ControlTrajectory>::SharedPtr
        trajectory_sub_;

    rclcpp::Subscription<ControlCommand>::SharedPtr
        command_sub_;

    // ---------------------------------------------------------
    // ROS publishers
    // ---------------------------------------------------------

    rclcpp::Publisher<Setpoint>::SharedPtr
        setpoint_pub_;

    rclcpp::Publisher<ControlStatus>::SharedPtr
        control_status_pub_;

    // ---------------------------------------------------------
    // ROS timers
    // ---------------------------------------------------------

    rclcpp::TimerBase::SharedPtr
        control_timer_;

    rclcpp::TimerBase::SharedPtr
        status_timer_;
};
