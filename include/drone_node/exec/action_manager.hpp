#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "drone_node/action/arm.hpp"
#include "drone_node/action/disarm.hpp"
#include "drone_node/action/go_to.hpp"
#include "drone_node/action/land.hpp"
#include "drone_node/action/stop.hpp"
#include "drone_node/action/takeoff.hpp"

#include "drone_node/exec/flight_state_machine.hpp"
#include "drone_node/exec/motion_executor.hpp"

namespace drone_node::exec
{

class ActionManager : public rclcpp::Node
{
public:
    explicit ActionManager(
        std::shared_ptr<MotionExecutor> motion_executor,
        std::shared_ptr<FlightStateMachine> flight_state_machine,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ActionManager() override;

    // MotionExecutor 可能从另一个节点的回调线程调用。
    // 这里只入队，不直接操作 GoalHandle 或活动任务。
    void enqueueExecutionEvent(
        const MotionExecutor::ExecutionEvent& event);

private:
    using Clock = std::chrono::steady_clock;
    using GoalUUID = rclcpp_action::GoalUUID;

    using Arm = drone_node::action::Arm;
    using Disarm = drone_node::action::Disarm;
    using Takeoff = drone_node::action::Takeoff;
    using GoTo = drone_node::action::GoTo;
    using Land = drone_node::action::Land;
    using Stop = drone_node::action::Stop;

    enum class ArmPhase : std::uint8_t
    {
        WAIT_SERVICE = 1,
        WAIT_RESPONSE = 2,
        WAIT_OBSERVATION = 3
    };

    enum class ErrorCode : std::uint16_t
    {
        NONE = 0,
        CANCELED = 1,
        SERVICE_UNAVAILABLE = 2,
        TIMEOUT = 3,
        SERVICE_FAILED = 4,
        STATE_REJECTED = 5,
        VEHICLE_FAULT = 6,
        INTERNAL_ERROR = 7
    };

    struct ArmOperation
    {
        GoalUUID uuid{};
        bool desired_arm{false};
        bool attached{false};
        bool cancel_requested{false};
        bool request_sent{false};

        ArmPhase phase{ArmPhase::WAIT_SERVICE};

        Clock::time_point started_at{};
        Clock::time_point sent_at{};
        Clock::time_point response_at{};
        Clock::time_point next_feedback_at{};

        std::optional<MotionExecutor::ArmRequest> request;

        std::function<void(std::uint8_t, const std::string&)> feedback;

        std::function<void(
            ActionState,
            ErrorCode,
            const std::string&)> finish;
    };

    template<typename ActionT>
    typename rclcpp_action::Server<ActionT>::SharedPtr createServer(
        const std::string& name);

    rclcpp_action::GoalResponse handleArmGoal(
        const GoalUUID& uuid,
        bool desired_arm);

    rclcpp_action::CancelResponse handleArmCancel(
        const GoalUUID& uuid);

    template<typename ActionT>
    void attachArmGoal(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<ActionT>>& handle);

    void tick();
    void tickArm();

    void refreshSnapshot();
    void processExecutionEvents();

    void publishFeedback();
    void clearPendingRequest();

    void finishActive(
        ActionState state,
        ErrorCode error,
        const std::string& message);

    void latchFault(const std::string& reason);

    std::shared_ptr<MotionExecutor> motion_executor_;
    std::shared_ptr<FlightStateMachine> fsm_;

    rclcpp_action::Server<Arm>::SharedPtr arm_server_;
    rclcpp_action::Server<Disarm>::SharedPtr disarm_server_;
    rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
    rclcpp_action::Server<GoTo>::SharedPtr goto_server_;
    rclcpp_action::Server<Land>::SharedPtr land_server_;
    rclcpp_action::Server<Stop>::SharedPtr stop_server_;

    rclcpp::TimerBase::SharedPtr timer_;

    // 所有 Action 回调和管理定时器都使用本节点默认互斥回调组。
    // 因而 active_ 不会被这些回调并发访问。
    std::optional<ArmOperation> active_;
    FlightPhase logged_phase_{FlightPhase::UNKNOWN};

    // 跨节点执行事件单独通过互斥队列传递。
    std::mutex event_mutex_;
    std::deque<MotionExecutor::ExecutionEvent> execution_events_;

    double service_wait_timeout_{3.0};
    double response_timeout_{6.0};
    double observation_timeout_{2.5};
    double action_timeout_{15.0};
    double feedback_period_{0.2};
};

}  // namespace drone_node::exec