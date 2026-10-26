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
#include "drone_node/exec/planner/motion_planner.hpp"

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

    // 跨节点回调只负责入队。
    // GoalHandle 和活动任务由本节点的互斥回调组处理。
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
        INTERNAL_ERROR = 7,
        MOTION_FAILED = 8
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

        std::function<void(
            std::uint8_t stage,
            const std::string& message)> feedback;

        std::function<void(
            ActionState,
            ErrorCode,
            const std::string&)> finish;
    };

    struct MotionOperation
    {
        GoalUUID uuid{};
        ActionType type{ActionType::TAKEOFF};

        bool attached{false};
        bool cancel_requested{false};
        bool stop_requested{false};
        bool fsm_started{false};

        // Takeoff 的相对高度目标及起始高度。
        double height{0.0};
        double start_z{0.0};

        // GoTo 保存固定目标，初始距离用于计算真实位置进度。
        PlanningRequest request;
        double initial_distance{0.0};

        PlannedMotion motion;
        MotionExecutor::ExecutionConstraints constraints;
        double confirmation_timeout{2.0};

        std::optional<std::uint64_t> trajectory_id;
        std::optional<MotionExecutor::ExecutionEvent> terminal_event;

        Clock::time_point dispatched_at{};
        Clock::time_point event_received_at{};
        Clock::time_point next_feedback_at{};

        // 非空时优先以失败结束，即使后续停止成功。
        std::string failure;

        // value：Takeoff 为相对高度，GoTo 为剩余距离。
        std::function<void(
            const std::string& phase,
            float progress,
            double value)> feedback;

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

    rclcpp_action::GoalResponse handleTakeoffGoal(
        const GoalUUID& uuid,
        const Takeoff::Goal& goal);

    rclcpp_action::GoalResponse handleGoToGoal(
        const GoalUUID& uuid,
        const GoTo::Goal& goal);

    rclcpp_action::CancelResponse handleMotionCancel(
        const GoalUUID& uuid);

    template<typename ActionT>
    void attachMotionGoal(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<ActionT>>& handle);

    void tick();
    void tickArm();

    void startMotion();
    void tickMotion();

    void refreshSnapshot();
    void processExecutionEvents();

    void publishFeedback();
    void publishMotionFeedback();

    bool motionAtTarget() const;

    void clearPendingRequest();

    void finishActive(
        ActionState state,
        ErrorCode error,
        const std::string& message);

    void finishMotion(
        ActionState state,
        ErrorCode error,
        const std::string& message);

    void latchFault(const std::string& reason);

    static double measuredYaw(
        const geometry_msgs::msg::QuaternionStamped& attitude);

    std::shared_ptr<MotionExecutor> motion_executor_;
    std::shared_ptr<FlightStateMachine> fsm_;

    MotionPlanner planner_;

    rclcpp_action::Server<Arm>::SharedPtr arm_server_;
    rclcpp_action::Server<Disarm>::SharedPtr disarm_server_;
    rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
    rclcpp_action::Server<GoTo>::SharedPtr goto_server_;
    rclcpp_action::Server<Land>::SharedPtr land_server_;
    rclcpp_action::Server<Stop>::SharedPtr stop_server_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Action 回调和管理定时器使用本节点默认互斥回调组。
    // 所有接纳入口共同检查这两个任务槽，保证单活动任务。
    std::optional<ArmOperation> active_;
    std::optional<MotionOperation> motion_;

    FlightPhase logged_phase_{FlightPhase::UNKNOWN};

    // MotionExecutor 的跨节点事件通过此队列传递。
    std::mutex event_mutex_;
    std::deque<MotionExecutor::ExecutionEvent> execution_events_;

    double service_wait_timeout_{3.0};
    double response_timeout_{6.0};
    double observation_timeout_{2.5};
    double action_timeout_{15.0};
    double feedback_period_{0.2};

    double takeoff_default_height_{1.5};
    double takeoff_confirmation_timeout_{2.0};
    MotionExecutor::ExecutionConstraints takeoff_constraints_;

    double goto_confirmation_timeout_{2.0};
    MotionExecutor::ExecutionConstraints goto_constraints_;
};

}  // namespace drone_node::exec