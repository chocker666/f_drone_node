#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "drone_node/msg/vehicle_state.hpp"
#include "drone_node/msg/trajectory_setpoint.hpp"
#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/msg/control_status.hpp"

#include "drone_node/ctrl/algorithm/controller_base.hpp"
#include "drone_node/ctrl/algorithm/pid_controller.hpp"
#include "drone_node/ctrl/trajectory_manager.hpp"
#include "drone_node/types.hpp"

#include <limits>

class FlightController : public rclcpp::Node
{
public:
    explicit FlightController(
        const rclcpp::NodeOptions& options =
            rclcpp::NodeOptions());

private:
    using VehicleState = drone_node::msg::VehicleState;
    using TrajectorySetpoint =
        drone_node::msg::TrajectorySetpoint;
    using ControlTrajectory =
        drone_node::msg::ControlTrajectory;
    using ControlStatus =
        drone_node::msg::ControlStatus;

    using SteadyClock = std::chrono::steady_clock;

    void vehicle_state_callback(
        const VehicleState::SharedPtr msg);

    void trajectory_callback(
        const ControlTrajectory::SharedPtr msg);

    void control_loop();
    void status_loop();

    void publish_velocity_setpoint(
        const Eigen::Vector3d& velocity,
        double yaw_rate);

    void change_control_state(uint8_t state);
    void enter_fault(uint16_t fault_code);

    bool state_is_fresh() const;

    // Angular utilities.
    static double wrap_pi(double angle);

    static double yaw_from_quaternion(
        const geometry_msgs::msg::Quaternion& q);

    static double select_yaw_delta(
        double current_yaw,
        double target_yaw,
        uint8_t direction);

    void update_unwrapped_yaw(double measured_yaw);

    void initialize_yaw_reference(double target_yaw);

    // Scheduling and safety.
    double control_frequency_{50.0};
    double status_frequency_{10.0};
    double state_timeout_{0.2};
    bool enable_setpoint_stream_{false};

    // Independent controller instances.
    std::unique_ptr<
        drone_node::algorithm::ControllerBase<3>>
        controller_;

    drone_node::algorithm::PIDController<1>
        yaw_controller_;

    drone_node::TrajectoryManager trajectory_manager_;

    // Vehicle feedback.
    VehicleState latest_vehicle_state_{};
    bool has_valid_state_{false};

    SteadyClock::time_point last_state_receive_time_{};

    // Continuous yaw measurement.
    bool has_yaw_measurement_{false};
    double previous_wrapped_yaw_{0.0};
    double unwrapped_yaw_{0.0};

    // Estimated yaw rate, rad/s.
    double measured_yaw_rate_{0.0};
    SteadyClock::time_point last_yaw_receive_time_{};

    // Continuous yaw target.
    bool yaw_reference_initialized_{false};
    double target_unwrapped_yaw_{0.0};

    // Current control status.
    uint8_t control_state_{ControlStatus::IDLE};
    uint16_t fault_code_{ControlStatus::FAULT_NONE};

    drone_node::ReferenceState latest_reference_{};
    bool has_reference_{false};
    // Latest tracking errors calculated by control_loop().
    // NaN means no valid tracking error is available.
    double position_error_{
        std::numeric_limits<double>::quiet_NaN()};

    double yaw_error_{
        std::numeric_limits<double>::quiet_NaN()};

    SteadyClock::time_point last_control_time_{};
    bool has_last_control_time_{false};

    // ROS entities.
    rclcpp::Subscription<VehicleState>::SharedPtr
        vehicle_state_sub_;

    rclcpp::Subscription<ControlTrajectory>::SharedPtr
        trajectory_sub_;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;

    rclcpp::Publisher<ControlStatus>::SharedPtr
        control_status_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};