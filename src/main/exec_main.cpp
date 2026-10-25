#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_node/exec/motion_executor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int result = 0;

    try {
        auto motion_executor =
            std::make_shared<drone_node::exec::MotionExecutor>();

        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2);

        executor.add_node(motion_executor);
        executor.spin();
    }
    catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("exec_main"),
            "Exec failed: %s",
            error.what());
        result = 1;
    }

    rclcpp::shutdown();
    return result;
}