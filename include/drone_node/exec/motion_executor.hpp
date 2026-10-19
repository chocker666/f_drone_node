
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "drone_node/action/go_to.hpp"
#include "drone_node/action/takeoff.hpp"
#include "drone_node/action/land.hpp"
#include "drone_node/action/arm.hpp"
#include "drone_node/action/disarm.hpp"
#include "drone_node/action/stop.hpp"

#include "drone_node/exec/action_manager.hpp"
#include "drone_node/exec/flight_state_machine.hpp"
#include "drone_node/exec/planner/motion_planner.hpp"

namespace drone_node::exec
{

class MotionExecutor : public rclcpp::Node
{
public:
    explicit MotionExecutor(
        const rclcpp::NodeOptions& options =
            rclcpp::NodeOptions());

private:
    using GoalUUID = rclcpp_action::GoalUUID;
    using Clock = std::chrono::steady_clock;

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
    using Server = rclcpp_action::Server<ActionT>;

    struct GoalBinding
    {
        ActionId id{0};
        ActionType type{ActionType::STOP};

        bool attached{false};
        Clock::time_point started_at{};

        std::function<void(double)> feedback;

        std::function<void(
            ActionState,
            const std::string&,
            bool)> finish;
    };

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
        const std::shared_ptr<GoalHandle<ActionT>> handle);

    template<typename ActionT>
    void handle_accepted(
        const std::shared_ptr<GoalHandle<ActionT>> handle);

    template<typename ActionT>
    bool validate_goal(
        const typename ActionT::Goal& goal) const;

    template<typename ActionT>
    static constexpr ActionType action_type();

    template<typename ActionT>
    void attach_goal(
        ActionId id,
        const std::shared_ptr<GoalHandle<ActionT>>& handle);

    static std::string uuid_key(const GoalUUID& uuid);

    // 第三阶段：GoTo Action -> MotionPlanner。
    bool plan_goto(
        ActionId id,
        const GoTo::Goal& goal);

    void update_mock();
    void process_events();

    void apply_mock_confirmation(
        ActionId id,
        ActionType type,
        bool canceling);

    void publish_mock_feedback(
        ActionId id,
        double progress);

    typename Server<GoTo>::SharedPtr goto_server_;
    typename Server<Takeoff>::SharedPtr takeoff_server_;
    typename Server<Land>::SharedPtr land_server_;
    typename Server<Arm>::SharedPtr arm_server_;
    typename Server<Disarm>::SharedPtr disarm_server_;
    typename Server<Stop>::SharedPtr stop_server_;

    std::shared_ptr<FlightStateMachine> fsm_;
    std::shared_ptr<ActionManager> manager_;

    // 第三阶段：统一规划入口。
    std::unique_ptr<MotionPlanner> planner_;

    rclcpp::TimerBase::SharedPtr mock_timer_;

    // UUID -> ActionId。
    std::unordered_map<std::string, ActionId> uuid_to_id_;

    // ActionId -> GoalHandle 适配器。
    std::unordered_map<ActionId, GoalBinding> bindings_;

    // 已通过校验并被接受的 GoTo Goal。
    // 采用值拷贝，避免 Action 生命周期悬空引用。
    std::unordered_map<ActionId, GoTo::Goal> goto_goals_;

    // 规划成功的轨迹缓存，供第四阶段 Coordinator 使用。
    std::unordered_map<ActionId, PlannedMotion> planned_motions_;

    // 保护 Action 和规划缓存。
    std::mutex mutex_;

    double mock_duration_{3.0};
    double mock_stop_duration_{1.0};
    double max_descent_speed_{1.0};
    double action_timeout_{30.0};

    std::vector<ActionEvent> deferred_events_;

    // Mock 状态不代表真实 PX4 观测。
    VehicleSnapshot mock_snapshot_{};
};

}  // namespace drone_node::exec
