#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace drone_node::exec
{
class FlightStateMachine;
class MotionExecutor;
class ActionManager;

class ExecNode final : public rclcpp::Node
{
public:
    explicit ExecNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ExecNode() override;
    // Call only after executor.spin() has returned and workers have joined.
    void shutdown() noexcept;

private:
    rclcpp::CallbackGroup::SharedPtr action_group_;
    rclcpp::CallbackGroup::SharedPtr motion_group_;
    std::shared_ptr<FlightStateMachine> fsm_;
    std::shared_ptr<MotionExecutor> motion_executor_;
    std::shared_ptr<ActionManager> action_manager_;
};
}  // namespace drone_node::exec
