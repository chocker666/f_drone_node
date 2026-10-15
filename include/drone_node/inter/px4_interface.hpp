#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <string>
#include "drone_node/msg/vehicle_state.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include "drone_node/msg/trajectory_setpoint.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>





enum class ControlMode : uint8_t
{
  POSITION = 0,
  VELOCITY = 1,
  ACCELERATION = 2,
  ATTITUDE = 3,
  BODY_RATE = 4
};


class PX4Interface : public rclcpp::Node
{
public:
    // PX4Interface();
    explicit PX4Interface(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
private:
    //  configs
    std::string vehicle_status_topic_;
    double heartbeat_timeout_{1.0};
    double status_publish_rate_{10.0};
    
    std::string vehicle_command_topic_;
    
    std::string vehicle_local_position_topic_;
    std::string vehicle_attitude_topic_;
    
    ControlMode current_control_mode_{ControlMode::POSITION};
    rclcpp::TimerBase::SharedPtr offboard_mode_timer_;
    double offboard_mode_publish_rate_{20.0};
    std::string px4_trajectory_setpoint_topic_;
    std::string offboard_control_mode_topic_;
    
    //  states
    bool armed_{false};
    bool connected_{false};
    rclcpp::Time last_heartbeat_time_;
    
    Eigen::Vector3d position_enu_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_enu_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond attitude_enu_flu_{Eigen::Quaterniond::Identity()};


    //  ROS entity
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_command_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Publisher<drone_node::msg::VehicleState>::SharedPtr vehicle_state_pub_;
    
    rclcpp::Subscription<drone_node::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

    //  callbacks
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    
    void trajectory_setpoint_callback(const drone_node::msg::TrajectorySetpoint::SharedPtr msg);
  
    //  funcitons
    void update_status();  // 10Hz pub DDS_heartbeat and arm
    void publish_offboard_control_mode();  // 20Hz pub offboard_mode
    bool update_control_mode(uint8_t control_mode);  // save current given offboard_mode
    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void arm_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
};
