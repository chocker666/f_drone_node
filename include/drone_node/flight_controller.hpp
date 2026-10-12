
#pragma once

#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include "drone_node/msg/vehicle_state.hpp"
#include "drone_node/msg/trajectory_setpoint.hpp"

class FlightController : public rclcpp::Node
{
public:
    // FlightController();
    explicit FlightController(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    using VehicleState = drone_node::msg::VehicleState;
    using TrajectorySetpoint = drone_node::msg::TrajectorySetpoint;
    using SteadyClock = std::chrono::steady_clock;

    void vehicle_state_callback(
        const VehicleState::SharedPtr msg);

    void control_loop();

    // Parameters
    double control_frequency_{50.0};
    double state_timeout_{0.2};
    bool enable_setpoint_stream_{false};

    // Latest feedback
    VehicleState latest_vehicle_state_{};
    bool has_valid_state_{false};
    SteadyClock::time_point last_state_receive_time_{};

    // Fixed position target for phase-2 testing
    bool target_initialized_{false};
    geometry_msgs::msg::Point target_position_{};
    float target_yaw_deg_{0.0f};

    // ROS entities
    rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_sub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Diagnostics
    std::uint64_t control_cycle_count_{0};
    std::uint64_t setpoint_publish_count_{0};
};
