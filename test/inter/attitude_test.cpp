#include <cmath>
#include <iomanip>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "drone_node/msg/vehicle_state.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>


class AttitudeTest : public rclcpp::Node
{
public:
  AttitudeTest()
  : Node("attitude_test")
  {
    px4_attitude_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        rclcpp::SensorDataQoS(),
        std::bind(
          &AttitudeTest::px4_attitude_callback,
          this,
          std::placeholders::_1));

    vehicle_state_sub_ =
      this->create_subscription<drone_node::msg::VehicleState>(
        "/px4_interface/vehicle_state",
        10,
        std::bind(
          &AttitudeTest::vehicle_state_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Attitude test node started.");
  }

private:

  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  // Eigen quaternion -> roll/pitch/yaw
  static Eigen::Vector3d quaternion_to_rpy(
    const Eigen::Quaterniond & q)
  {
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    const double roll =
      std::atan2(R(2, 1), R(2, 2));

    const double pitch =
      std::asin(std::clamp(-R(2, 0), -1.0, 1.0));

    const double yaw =
      std::atan2(R(1, 0), R(0, 0));

    return {roll, pitch, yaw};
  }


  void px4_attitude_callback(
    const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    // PX4 VehicleAttitude:
    // q[0] = w
    // q[1] = x
    // q[2] = y
    // q[3] = z

    Eigen::Quaterniond q(
      msg->q[0],
      msg->q[1],
      msg->q[2],
      msg->q[3]);

    const Eigen::Vector3d rpy = quaternion_to_rpy(q);

    RCLCPP_INFO(
      this->get_logger(),
      "PX4 NED/FRD | "
      "q[wxyz]=[% .4f % .4f % .4f % .4f] | "
      "RPY=[% 7.2f % 7.2f % 7.2f] deg",
      msg->q[0],
      msg->q[1],
      msg->q[2],
      msg->q[3],
      rpy.x() * RAD_TO_DEG,
      rpy.y() * RAD_TO_DEG,
      rpy.z() * RAD_TO_DEG);
  }


  void vehicle_state_callback(
    const drone_node::msg::VehicleState::SharedPtr msg)
  {
    // geometry_msgs Quaternion:
    // msg: x y z w
    // Eigen constructor: w x y z

    Eigen::Quaterniond q(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);

    const Eigen::Vector3d rpy = quaternion_to_rpy(q);

    RCLCPP_INFO(
      this->get_logger(),
      "ROS ENU/FLU | "
      "q[xyzw]=[% .4f % .4f % .4f % .4f] | "
      "RPY=[% 7.2f % 7.2f % 7.2f] deg",
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w,
      rpy.x() * RAD_TO_DEG,
      rpy.y() * RAD_TO_DEG,
      rpy.z() * RAD_TO_DEG);
  }


  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
    px4_attitude_sub_;

  rclcpp::Subscription<drone_node::msg::VehicleState>::SharedPtr
    vehicle_state_sub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AttitudeTest>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
