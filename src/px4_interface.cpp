#include "drone_node/px4_interface.hpp"
#include <stdexcept>
#include "drone_node/frame_transforms.hpp"
#include <functional>
#include <limits>
#include <cmath>

namespace ft = drone_node::frame_transforms;


// PX4Interface::PX4Interface() : Node("px4_interface")
PX4Interface::PX4Interface(
    const rclcpp::NodeOptions & options)
    : Node("px4_interface", options)
{
    RCLCPP_INFO(this->get_logger(), "PX4 interface started");
    
    // DDS_heartbeat and arm
    this->declare_parameter<double>("status_publish_rate", 10.0);
    status_publish_rate_ = this->get_parameter("status_publish_rate").as_double();
    if (status_publish_rate_ <= 0.0)
        throw std::runtime_error("status_publish_rate must be greater than 0");
    this->declare_parameter<double>("heartbeat_timeout", 1.0);
    heartbeat_timeout_ = this->get_parameter("heartbeat_timeout").as_double();
    this->declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
    
    const auto status_period = std::chrono::duration<double>(1.0 / status_publish_rate_);
    status_timer_ = this->create_wall_timer( 
        status_period, 
        std::bind(
            &PX4Interface::update_status, 
            this));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &PX4Interface::vehicle_status_callback,
            this,
            std::placeholders::_1));
    armed_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/px4_interface/out/armed", 10);
    connected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/px4_interface/out/connected", 10);

    // arm signal
    arm_command_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/px4_interface/in/arm_cmd",
        10,
        std::bind(
          &PX4Interface::arm_command_callback,
          this,
          std::placeholders::_1));
    vehicle_command_topic_ = 
        this->declare_parameter<std::string>(
        "vehicle_command_topic", "/fmu/in/vehicle_command");
    vehicle_command_pub_ = 
        this->create_publisher<px4_msgs::msg::VehicleCommand>(
        vehicle_command_topic_, 10);

    // position and attitude
    this->declare_parameter<std::string>("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    vehicle_local_position_topic_ = this->get_parameter("vehicle_local_position_topic").as_string();
    this->declare_parameter<std::string>("vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    vehicle_attitude_topic_ = this->get_parameter("vehicle_attitude_topic").as_string();
    
    vehicle_local_position_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            vehicle_local_position_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &PX4Interface::vehicle_local_position_callback,
                this,
                std::placeholders::_1));
    vehicle_attitude_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            vehicle_attitude_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &PX4Interface::vehicle_attitude_callback,
                this,
                std::placeholders::_1));
    vehicle_state_pub_ =
      this->create_publisher<drone_node::msg::VehicleState>(
        "/px4_interface/out/vehicle_state", 10);

    // set_point and control_mode
    offboard_mode_publish_rate_ = this->declare_parameter<double>("offboard_mode_publish_rate", 20.0);
    if (offboard_mode_publish_rate_ <= 0.0) 
        throw std::runtime_error("offboard_mode_publish_rate must be greater than 0");
    this->declare_parameter<std::string>("px4_trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    px4_trajectory_setpoint_topic_ = this->get_parameter("px4_trajectory_setpoint_topic").as_string();
    this->declare_parameter<std::string>("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    offboard_control_mode_topic_ = this->get_parameter("offboard_control_mode_topic").as_string();

    const auto offboard_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / offboard_mode_publish_rate_));
    offboard_mode_timer_ = this->create_wall_timer(
        offboard_period,
        std::bind(&PX4Interface::publish_offboard_control_mode, this));
    trajectory_setpoint_sub_ =
      this->create_subscription<drone_node::msg::TrajectorySetpoint>(
        "/px4_interface/in/trajectory_setpoint",
        rclcpp::QoS(10),
        std::bind(
          &PX4Interface::trajectory_setpoint_callback,
          this,
          std::placeholders::_1));
    px4_trajectory_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_trajectory_setpoint_topic_,
        rclcpp::QoS(10));
    offboard_control_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        offboard_control_mode_topic_,
        rclcpp::QoS(10));

}


//  GET dds_heartbeat_alive and arm_status
void PX4Interface::vehicle_status_callback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    last_heartbeat_time_ = this->now();
    armed_ =
        msg->arming_state ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}


//  basic status publish
void PX4Interface::update_status()
{
    if (last_heartbeat_time_.nanoseconds() == 0)
        connected_ = false;
    else 
        connected_ = (this->now() - last_heartbeat_time_).seconds() < heartbeat_timeout_;

    std_msgs::msg::Bool armed_msg;
    armed_msg.data = armed_;
    armed_pub_->publish(armed_msg);

    std_msgs::msg::Bool connected_msg;
    connected_msg.data = connected_;
    connected_pub_->publish(connected_msg);
}


//  GET position && position and attitude publish
void PX4Interface::vehicle_local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    Eigen::Vector3d position_ned(msg->x, msg->y, msg->z);
    Eigen::Vector3d velocity_ned(msg->vx, msg->vy, msg->vz);
    position_enu_ = ft::ned_to_enu_local_frame(position_ned);
    velocity_enu_ = ft::ned_to_enu_local_frame(velocity_ned);
    
    // auto state = std::make_unique<drone_node::msg::VehicleState>();
    // state->stamp = this->now();

    // state->position.x = position_enu_.x();
    // state->position.y = position_enu_.y();
    // state->position.z = position_enu_.z();

    // state->velocity.x = velocity_enu_.x();
    // state->velocity.y = velocity_enu_.y();
    // state->velocity.z = velocity_enu_.z();

    // state->orientation.x = attitude_enu_flu_.x();
    // state->orientation.y = attitude_enu_flu_.y();
    // state->orientation.z = attitude_enu_flu_.z();
    // state->orientation.w = attitude_enu_flu_.w();

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "PUB address: %p",
    //     static_cast<const void *>(state.get()));

    // vehicle_state_pub_->publish(std::move(state));
    drone_node::msg::VehicleState state;
    state.stamp = this->now();

    // Position: ENU
    state.position.x = position_enu_.x();
    state.position.y = position_enu_.y();
    state.position.z = position_enu_.z();

    // Velocity: ENU
    state.velocity.x = velocity_enu_.x();
    state.velocity.y = velocity_enu_.y();
    state.velocity.z = velocity_enu_.z();

    // Orientation: FLU body -> ENU world
    state.orientation.x = attitude_enu_flu_.x();
    state.orientation.y = attitude_enu_flu_.y();
    state.orientation.z = attitude_enu_flu_.z();
    state.orientation.w = attitude_enu_flu_.w();

    vehicle_state_pub_->publish(state);
}


//  GET attitude
void PX4Interface::vehicle_attitude_callback(
    const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    const Eigen::Quaterniond q_px4 = ft::utils::quaternion::array_to_eigen_quat(msg->q);
    attitude_enu_flu_ = ft::px4_to_ros_orientation(q_px4);
}


// GET setpoint && SET setpoint
void PX4Interface::trajectory_setpoint_callback(
    const drone_node::msg::TrajectorySetpoint::SharedPtr msg)
{
    if (!update_control_mode(msg->control_mode)) return;
    publish_offboard_control_mode();

    const float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    
    // ROS time [ns] -> PX4 timestamp [us]
    const uint64_t timestamp =
      static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);

    // PX4 trajectory setpoint
    px4_msgs::msg::TrajectorySetpoint px4_setpoint{};
    px4_setpoint.timestamp = timestamp;
    px4_setpoint.position = {nan, nan, nan};
    px4_setpoint.velocity = {nan, nan, nan};
    px4_setpoint.acceleration = {nan, nan, nan};
    px4_setpoint.jerk = {nan, nan, nan};
    px4_setpoint.yaw = nan;
    px4_setpoint.yawspeed = nan;
    
    switch (current_control_mode_) {

    case ControlMode::POSITION:
    {
        // ROS ENU position -> PX4 NED position
        const Eigen::Vector3d position_enu(
          msg->position.x,
          msg->position.y,
          msg->position.z);

        const Eigen::Vector3d position_ned =
          ft::enu_to_ned_local_frame(position_enu);

        px4_setpoint.position[0] = static_cast<float>(position_ned.x());
        px4_setpoint.position[1] = static_cast<float>(position_ned.y());
        px4_setpoint.position[2] = static_cast<float>(position_ned.z());

        // ROS ENU/FLU yaw -> PX4 NED/FRD yaw
        const double yaw_enu_rad = static_cast<double>(msg->yaw) * DEG_TO_RAD;
        const Eigen::Quaterniond q_ros = ft::utils::quaternion::quaternion_from_euler( 0.0, 0.0, yaw_enu_rad);
        const Eigen::Quaterniond q_px4 = ft::ros_to_px4_orientation(q_ros);
        px4_setpoint.yaw = static_cast<float>( ft::utils::quaternion::quaternion_get_yaw(q_px4));

        break;
    }

    case ControlMode::VELOCITY:
    {
        // ROS ENU velocity -> PX4 NED velocity
        const Eigen::Vector3d velocity_enu(
          msg->velocity.x,
          msg->velocity.y,
          msg->velocity.z);

        const Eigen::Vector3d velocity_ned =
          ft::enu_to_ned_local_frame(velocity_enu);

        px4_setpoint.velocity[0] = static_cast<float>(velocity_ned.x());
        px4_setpoint.velocity[1] = static_cast<float>(velocity_ned.y());
        px4_setpoint.velocity[2] = static_cast<float>(velocity_ned.z());
        
        // ROS ENU yaw rate -> PX4 NED yaw rate
        px4_setpoint.yawspeed = -static_cast<float>(static_cast<double>(msg->yawspeed) * DEG_TO_RAD);

        break;
    }
    
    case ControlMode::ACCELERATION:
    {
        // ROS ENU acceleration -> PX4 NED acceleration
        const Eigen::Vector3d acceleration_enu(
          msg->acceleration.x,
          msg->acceleration.y,
          msg->acceleration.z);

        const Eigen::Vector3d acceleration_ned =
          ft::enu_to_ned_local_frame(acceleration_enu);

        px4_setpoint.acceleration[0] =
          static_cast<float>(acceleration_ned.x());
        px4_setpoint.acceleration[1] =
          static_cast<float>(acceleration_ned.y());
        px4_setpoint.acceleration[2] =
          static_cast<float>(acceleration_ned.z());

        // ROS ENU jerk -> PX4 NED jerk
        if (msg->use_jerk) {
            const Eigen::Vector3d jerk_enu(
              msg->jerk.x,
              msg->jerk.y,
              msg->jerk.z);

            const Eigen::Vector3d jerk_ned =
              ft::enu_to_ned_local_frame(jerk_enu);

            px4_setpoint.jerk[0] = static_cast<float>(jerk_ned.x());
            px4_setpoint.jerk[1] = static_cast<float>(jerk_ned.y());
            px4_setpoint.jerk[2] = static_cast<float>(jerk_ned.z());
        }

        // ROS ENU yaw rate -> PX4 NED yaw rate
        px4_setpoint.yawspeed =
          -static_cast<float>(
            static_cast<double>(msg->yawspeed) * DEG_TO_RAD);

        break;
    }
    default:
        RCLCPP_WARN(this->get_logger(), "Unsupported trajectory setpoint control mode");
        return;
    }

    px4_trajectory_setpoint_pub_->publish(px4_setpoint);
}


//  heartbeat offboard_mode publish
void PX4Interface::publish_offboard_control_mode()
{
  px4_msgs::msg::OffboardControlMode msg{};

  // PX4 timestamp，单位：us
  msg.timestamp =
    static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);

  // 默认全部关闭
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;

  // 根据本地控制模式，只开启对应的一项
  switch (current_control_mode_) {
    case ControlMode::POSITION:
      msg.position = true;
      break;

    case ControlMode::VELOCITY:
      msg.velocity = true;
      break;

    case ControlMode::ACCELERATION:
      msg.acceleration = true;
      break;

    case ControlMode::ATTITUDE:
      msg.attitude = true;
      break;

    case ControlMode::BODY_RATE:
      msg.body_rate = true;
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown control mode, OffboardControlMode not published");
      return;
  }

  offboard_control_mode_pub_->publish(msg);
}


// convert offboard_mode msg to local
bool PX4Interface::update_control_mode(uint8_t control_mode)
{
  switch (control_mode) {
    case drone_node::msg::TrajectorySetpoint::POSITION:
      current_control_mode_ = ControlMode::POSITION;
      break;

    case drone_node::msg::TrajectorySetpoint::VELOCITY:
      current_control_mode_ = ControlMode::VELOCITY;
      break;

    case drone_node::msg::TrajectorySetpoint::ACCELERATION:
      current_control_mode_ = ControlMode::ACCELERATION;
      break;

    case drone_node::msg::TrajectorySetpoint::ATTITUDE:
      current_control_mode_ = ControlMode::ATTITUDE;
      break;

    case drone_node::msg::TrajectorySetpoint::BODY_RATE:
      current_control_mode_ = ControlMode::BODY_RATE;
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(),
        "Received invalid control mode: %u",
        static_cast<unsigned int>(control_mode));
      return false;
  }

  return true;
}


// publish arm/disarm signal
void PX4Interface::publish_vehicle_command(
  uint16_t command,
  float param1,
  float param2)
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.param1 = param1;
  msg.param2 = param2;

  msg.command = command;

  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  msg.timestamp =
    static_cast<uint64_t>(
      this->get_clock()->now().nanoseconds() / 1000);

  vehicle_command_pub_->publish(msg);
}

//  GET arm/disarm command from higher layer
void PX4Interface::arm_command_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        // 1. 请求进入 Offboard 模式
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,
            6.0f);

        // 2. 请求解锁
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f);

        RCLCPP_INFO(
            this->get_logger(),
            "Offboard mode and ARM requested");
    }
    else {
        // 请求上锁
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0f);

        RCLCPP_INFO(
            this->get_logger(),
            "DISARM requested");
    }
}
