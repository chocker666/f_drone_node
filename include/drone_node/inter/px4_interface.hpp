#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

class PX4Interface : public rclcpp::Node
{
public:
    explicit PX4Interface(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PX4Interface() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};