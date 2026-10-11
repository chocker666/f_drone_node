#include "drone_node/flight_controller.hpp"

FlightController::FlightController()
    : Node("flight_controller")
{
    RCLCPP_INFO(
        this->get_logger(),
        "Flight controller node started");
}