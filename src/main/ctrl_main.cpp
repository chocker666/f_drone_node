#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "drone_node/ctrl/flight_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<FlightController>());

    rclcpp::shutdown();
    return 0;
}