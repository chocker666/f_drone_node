#include <rclcpp/rclcpp.hpp>
#include "drone_node/px4_interface.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PX4Interface>());

    rclcpp::shutdown();
    return 0;
}
