#include "drone_node/exec/exec_node.hpp"
#include <exception>
#include "drone_node/exec/action_manager.hpp"
#include "drone_node/exec/flight_state_machine.hpp"
#include "drone_node/exec/motion_executor.hpp"

namespace drone_node::exec
{
ExecNode::ExecNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("exec_node", options)
{
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    motion_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    fsm_ = std::make_shared<FlightStateMachine>();
    motion_executor_ = std::make_shared<MotionExecutor>(*this, motion_group_);
    action_manager_ = std::make_shared<ActionManager>(
        *this, action_group_, motion_executor_, fsm_);
    std::weak_ptr<ActionManager> weak_manager = action_manager_;
    motion_executor_->setExecutionCallback(
        [weak_manager](const MotionExecutor::ExecutionEvent& event) {
            if (auto manager = weak_manager.lock())
                manager->enqueueExecutionEvent(event);
        });
}

ExecNode::~ExecNode()
{
    shutdown();
}

void ExecNode::shutdown() noexcept
{
    if (!motion_executor_)
        return;
    try {
        action_manager_->stop();
        motion_executor_->stop();
    }
    catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Exec shutdown request failed: %s", error.what());
    }
    motion_executor_->setExecutionCallback({});
    // No wait: executor callbacks have stopped. This is not a landing guarantee.
    action_manager_.reset();
    motion_executor_.reset();
    fsm_.reset();
}
}  // namespace drone_node::exec
