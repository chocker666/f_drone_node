
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "drone_node/msg/vehicle_state.hpp"
#include "drone_node/msg/trajectory_setpoint.hpp"
#include "drone_node/algorithm/controller_base.hpp"

class FlightController : public rclcpp::Node
{
public:
    explicit FlightController(
        const rclcpp::NodeOptions & options =
            rclcpp::NodeOptions());

private:
    using VehicleState = drone_node::msg::VehicleState;
    using TrajectorySetpoint =
        drone_node::msg::TrajectorySetpoint;
    using TargetPosition = geometry_msgs::msg::PointStamped;
    using SteadyClock = std::chrono::steady_clock;

    enum class MotionState
    {
        IDLE,
        MOVING,
        HOLD,
        FAULT
    };

    void vehicle_state_callback(
        const VehicleState::SharedPtr msg);

    void target_position_callback(
        const TargetPosition::SharedPtr msg);

    void control_loop();

    void publish_velocity_setpoint(
        const Eigen::Vector3d & velocity);

    void change_motion_state(MotionState next_state);

    // Scheduling and safety
    double control_frequency_{50.0};
    double state_timeout_{0.2};
    bool enable_setpoint_stream_{false};

    double position_tolerance_{0.15};
    double velocity_tolerance_{0.15};
    double arrival_hold_time_{0.5};

    // Algorithm selection
    std::unique_ptr<drone_node::algorithm::ControllerBase>
        controller_;

    // Feedback
    VehicleState latest_vehicle_state_{};
    bool has_valid_state_{false};
    SteadyClock::time_point last_state_receive_time_{};

    // Target and state
    Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
    bool has_target_{false};
    MotionState motion_state_{MotionState::IDLE};

    SteadyClock::time_point last_control_time_{};
    bool has_last_control_time_{false};

    SteadyClock::time_point arrival_start_time_{};
    bool arrival_timer_active_{false};

    // ROS entities
    rclcpp::Subscription<VehicleState>::SharedPtr
        vehicle_state_sub_;

    rclcpp::Subscription<TargetPosition>::SharedPtr
        target_position_sub_;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
};
