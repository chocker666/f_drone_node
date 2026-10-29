#pragma once

#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "drone_node/msg/setpoint.hpp"
#include "drone_node/msg/discrete_traj.hpp"
#include "drone_node/msg/polynomial_traj.hpp"
#include "drone_node/msg/control_status.hpp"
#include "drone_node/msg/control_command.hpp"

#include "drone_node/ctrl/controller/controller_factory.hpp"
#include "drone_node/ctrl/trajectory_manager.hpp"
#include "drone_node/ctrl/state_cache.hpp"
#include "drone_node/ctrl/hover_reference.hpp"

#include <map>
#include <string>

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
    using DiscreteTraj = drone_node::msg::DiscreteTraj;
    using PolynomialTraj = drone_node::msg::PolynomialTraj;
    using ControlStatus = drone_node::msg::ControlStatus;
    using ControlCommand = drone_node::msg::ControlCommand;
    using SteadyClock = drone_node::SteadyClock;

    void position_callback(const PointStamped::SharedPtr msg);
    void velocity_callback(const VectorStamped::SharedPtr msg);
    void attitude_callback(const QuaternionStamped::SharedPtr msg);
    void discrete_trajectory_callback(
        const DiscreteTraj::SharedPtr msg);
    void polynomial_trajectory_callback(
        const PolynomialTraj::SharedPtr msg);
    void report_trajectory_preparation(
        uint64_t trajectory_id,
        drone_node::TrajectoryManager::AcceptResult result,
        const std::string& reason);
    void command_callback(const ControlCommand::SharedPtr msg);

    void control_loop();
    void run_control_cycle();
    void status_loop();
    void set_control_timer(double frequency);

    void publish_velocity_setpoint(
        const Eigen::Vector3d& velocity,
        double yaw_rate);

    void change_control_state(uint8_t state);
    void enter_hover();
    void enter_idle();
    void enter_fault(uint16_t fault_code);
    void clear_execution();

    double control_frequency_{100.0};
    double active_control_frequency_{100.0};
    double status_frequency_{10.0};
    double state_timeout_{0.2};

    unsigned consecutive_overruns_{0};

    drone_node::controller::PositionYawPIDConfig controller_config_{};
    std::map<
        std::string,
        drone_node::controller::PositionYawPIDConfig>
        controller_configs_;
    drone_node::HoverReference::Config hover_config_{};

    std::unique_ptr<drone_node::StateCache> state_cache_;
    std::unique_ptr<drone_node::controller::ControllerBase> controller_;
    std::unique_ptr<drone_node::HoverReference> hover_reference_;

    drone_node::TrajectoryManager trajectory_manager_;

    uint8_t control_state_{ControlStatus::IDLE};
    uint16_t fault_code_{ControlStatus::FAULT_NONE};

    double position_error_{
        std::numeric_limits<double>::quiet_NaN()};
    double yaw_error_{
        std::numeric_limits<double>::quiet_NaN()};

    drone_node::SteadyTime last_control_time_{};
    bool has_last_control_time_{false};

    rclcpp::Subscription<PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<VectorStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<DiscreteTraj>::SharedPtr
        discrete_trajectory_sub_;
    rclcpp::Subscription<PolynomialTraj>::SharedPtr
        polynomial_trajectory_sub_;
    rclcpp::Subscription<ControlCommand>::SharedPtr command_sub_;

    rclcpp::Publisher<Setpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<ControlStatus>::SharedPtr control_status_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    void load_controller_configs();
};