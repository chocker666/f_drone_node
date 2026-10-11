#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "drone_node/flight_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<FlightController>());

    rclcpp::shutdown();
    return 0;
}