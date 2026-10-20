#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "drone_node/inter/px4_interface.hpp"
#include "drone_node/ctrl/flight_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto inter = std::make_shared<PX4Interface>(options);
    auto controller = std::make_shared<FlightController>(options);

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);
    executor.add_node(inter);
    executor.add_node(controller);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}