#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "drone_node/action/go_to.hpp"
#include "drone_node/action/takeoff.hpp"
#include "drone_node/action/land.hpp"
#include "drone_node/action/arm.hpp"
#include "drone_node/action/disarm.hpp"
#include "drone_node/action/stop.hpp"

namespace drone_node::exec
{

class MotionExecutor : public rclcpp::Node
{
public:
    explicit MotionExecutor(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using GoalUUID = rclcpp_action::GoalUUID;

    using GoTo = drone_node::action::GoTo;
    using Takeoff = drone_node::action::Takeoff;
    using Land = drone_node::action::Land;
    using Arm = drone_node::action::Arm;
    using Disarm = drone_node::action::Disarm;
    using Stop = drone_node::action::Stop;

    template<typename ActionT>
    using GoalHandle =
        rclcpp_action::ServerGoalHandle<ActionT>;

    template<typename ActionT>
    using Server =
        rclcpp_action::Server<ActionT>;

    template<typename ActionT>
    void create_action_server(
        const std::string& name,
        typename Server<ActionT>::SharedPtr& server);

    template<typename ActionT>
    rclcpp_action::GoalResponse handle_goal(
        const GoalUUID& uuid,
        std::shared_ptr<const typename ActionT::Goal> goal);

    template<typename ActionT>
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle<ActionT>> goal_handle);

    template<typename ActionT>
    void handle_accepted(
        const std::shared_ptr<GoalHandle<ActionT>> goal_handle);

    template<typename ActionT>
    bool validate_goal(
        const typename ActionT::Goal& goal) const;

    template<typename ActionT>
    void start_mock_task(
        const std::shared_ptr<GoalHandle<ActionT>> goal_handle);

    void update_mock_task();

    struct MockTask
    {
        std::string name;
        std::chrono::steady_clock::time_point start_time;
        double duration_seconds{0.0};

        std::function<void(double)> publish_feedback;
        std::function<bool()> is_canceling;
        std::function<void(bool)> finish;
    };

    typename Server<GoTo>::SharedPtr goto_server_;
    typename Server<Takeoff>::SharedPtr takeoff_server_;
    typename Server<Land>::SharedPtr land_server_;
    typename Server<Arm>::SharedPtr arm_server_;
    typename Server<Disarm>::SharedPtr disarm_server_;
    typename Server<Stop>::SharedPtr stop_server_;

    rclcpp::TimerBase::SharedPtr mock_timer_;

    std::mutex task_mutex_;
    std::shared_ptr<MockTask> active_task_;

    // Prevent a second Goal being accepted before
    // the first accepted callback starts its task.
    bool goal_reserved_{false};

    double mock_duration_{3.0};
    double max_descent_speed_{1.0};
};

}  // namespace drone_node::exec