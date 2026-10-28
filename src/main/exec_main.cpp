#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_node/exec/exec_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int result = 0;

    try {
        using namespace drone_node::exec;

        auto exec_node = std::make_shared<ExecNode>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2);
        executor.add_node(exec_node);
        executor.spin();
        exec_node->shutdown();
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