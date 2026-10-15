
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "drone_node/inter/px4_interface.hpp"
#include "drone_node/ctrl/flight_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Enable ROS2 intra-process communication for both nodes.
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);

    // Instantiate both nodes in the same process.
    auto px4_interface =
        std::make_shared<PX4Interface>(node_options);

    auto flight_controller =
        std::make_shared<FlightController>(node_options);

    // Each node has its own executor.
    rclcpp::executors::SingleThreadedExecutor
        px4_interface_executor;

    rclcpp::executors::SingleThreadedExecutor
        flight_controller_executor;

    px4_interface_executor.add_node(px4_interface);
    flight_controller_executor.add_node(flight_controller);

    // Run the two executors on independent threads.
    std::thread px4_interface_thread([&]() {
        px4_interface_executor.spin();
    });

    std::thread flight_controller_thread([&]() {
        flight_controller_executor.spin();
    });

    RCLCPP_INFO(
        rclcpp::get_logger("drone_node"),
        "Drone node started with two independent executors");

    // Wait until ROS2 shutdown.
    px4_interface_thread.join();
    flight_controller_thread.join();

    px4_interface_executor.remove_node(px4_interface);
    flight_controller_executor.remove_node(flight_controller);

    rclcpp::shutdown();
    return 0;
}
