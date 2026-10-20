#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

class AttitudeMonitor : public rclcpp::Node
{
public:
    AttitudeMonitor() : Node("attitude_test")
    {
        subscription_ =
            create_subscription<geometry_msgs::msg::QuaternionStamped>(
                "/px4_interface/out/attitude", rclcpp::QoS(1).reliable(),
                [this](
                    geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
                    const auto &q = msg->quaternion;
                    const double norm2 =
                        q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "frame=%s q=[%.4f %.4f %.4f %.4f] norm=%.4f",
                        msg->header.frame_id.c_str(),
                        q.w, q.x, q.y, q.z, std::sqrt(norm2));
                });
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeMonitor>());
    rclcpp::shutdown();
    return 0;
}