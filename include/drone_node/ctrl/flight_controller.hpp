#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "drone_node/msg/setpoint.hpp"
#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/msg/control_status.hpp"
#include "drone_node/msg/control_command.hpp"
#include "drone_node/msg/fixed_trajectory.hpp"

#include "drone_node/ctrl/controller/controller_factory.hpp"
#include "drone_node/ctrl/trajectory_manager.hpp"
#include "drone_node/ctrl/trajectory_sampler/fixed_sampler.hpp"
#include "drone_node/types.hpp"

class FlightController : public rclcpp::Node
{
public:
    explicit FlightController(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using Setpoint = drone_node::msg::Setpoint;
    using PointStamped = geometry_msgs::msg::PointStamped;
    using VectorStamped = geometry_msgs::msg::Vector3Stamped;
    using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
    using ControlTrajectory = drone_node::msg::ControlTrajectory;
    using ControlStatus = drone_node::msg::ControlStatus;
    using ControlCommand = drone_node::msg::ControlCommand;
    using FixedTrajectory = drone_node::msg::FixedTrajectory;
    using FixedSampler = drone_node::trajectory_sampler::FixedSampler;
    using SteadyClock = std::chrono::steady_clock;

    struct Feedback
    {
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        geometry_msgs::msg::Quaternion orientation;
    };

    void position_callback(const PointStamped::SharedPtr msg);
    void velocity_callback(const VectorStamped::SharedPtr msg);
    void attitude_callback(const QuaternionStamped::SharedPtr msg);
    void trajectory_callback(const ControlTrajectory::SharedPtr msg);
    void command_callback(const ControlCommand::SharedPtr msg);

    // Removed later in the FSM migration stage.
    void output_enable_callback(
        const std_msgs::msg::Bool::SharedPtr msg);
    void armed_callback(
        const std_msgs::msg::Bool::SharedPtr msg);

    void control_loop();
    void status_loop();

    void publish_velocity_setpoint(
        const Eigen::Vector3d& velocity,
        double yaw_rate);

    void change_control_state(uint8_t state);
    void enter_hover();
    void enter_idle();
    void enter_fault(uint16_t fault_code);
    void reset_control_runtime();

    FixedTrajectory make_current_fixed_reference() const;

    drone_node::PhysicalControllerInput make_controller_input(
        const drone_node::ReferenceState& reference,
        double dt) const;

    bool state_is_fresh() const;

    // Feedback estimation moves to StateCache in its migration stage.
    static double wrap_pi(double angle);
    static double yaw_from_quaternion(
        const geometry_msgs::msg::Quaternion& q);
    void update_unwrapped_yaw(double measured_yaw);

    double control_frequency_{50.0};
    double status_frequency_{10.0};
    double state_timeout_{0.2};

    bool enable_setpoint_stream_{false};

    drone_node::controller::PositionYawPIDConfig controller_config_{};
    std::unique_ptr<drone_node::controller::ControllerBase> controller_;
    bool controller_initialized_{false};

    drone_node::TrajectoryManager trajectory_manager_;
    std::unique_ptr<FixedSampler> hover_sampler_;

    Feedback latest_vehicle_state_{};
    std::array<bool, 3> feedback_received_{};
    std::array<SteadyClock::time_point, 3> feedback_time_{};

    bool armed_{false};
    bool has_armed_feedback_{false};

    bool has_yaw_measurement_{false};
    double previous_wrapped_yaw_{0.0};
    double unwrapped_yaw_{0.0};
    double measured_yaw_rate_{0.0};
    SteadyClock::time_point last_yaw_receive_time_{};

    uint8_t control_state_{ControlStatus::IDLE};
    uint16_t fault_code_{ControlStatus::FAULT_NONE};

    drone_node::ReferenceState latest_reference_{};
    bool has_reference_{false};

    double position_error_{
        std::numeric_limits<double>::quiet_NaN()};
    double yaw_error_{
        std::numeric_limits<double>::quiet_NaN()};

    SteadyClock::time_point last_control_time_{};
    bool has_last_control_time_{false};

    rclcpp::Subscription<PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<VectorStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<ControlTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<ControlCommand>::SharedPtr command_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr output_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;

    rclcpp::Publisher<Setpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<ControlStatus>::SharedPtr control_status_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};