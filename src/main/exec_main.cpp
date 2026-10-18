#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_node/exec/motion_executor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<drone_node::exec::MotionExecutor>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}