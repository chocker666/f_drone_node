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
#include "drone_node/action/takeoff.hpp"
#include "drone_node/action/go_to.hpp"
#include "drone_node/action/land.hpp"
#include "drone_node/action/stop.hpp"
#include "drone_node/action/continue.hpp"
#include "drone_node/action/hover.hpp"

#include "drone_node/exec/action_types.hpp"
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

    // 跨节点回调只入队。
    // GoalHandle 和任务状态由本节点默认互斥回调组处理。
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
    using Continue = drone_node::action::Continue;
    using Hover = drone_node::action::Hover;

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
        MOTION_FAILED = 8,
        PREEMPTED = 9
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

        // type 决定当前 FSM 的开始和完成事件。
        ActionType type{ActionType::TAKEOFF};

        // source_type 保留原始任务类型。
        ActionType source_type{ActionType::TAKEOFF};

        bool attached{false};
        bool cancel_requested{false};
        bool stop_requested{false};
        bool fsm_started{false};

        // Continue 使用保存的绝对目标，不重新解释相对高度。
        bool is_continue{false};
        bool resumed_target{false};

        double height{0.0};
        double start_z{0.0};

        PlanningRequest request;
        PlannedMotion motion;

        MotionExecutor::ExecutionConstraints constraints;
        double confirmation_timeout{2.0};

        // 新一段执行的初始距离，以及恢复前的任务级进度。
        double initial_distance{0.0};
        double progress_base{0.0};

        bool is_hover{false};
        bool holding{false};

        // 当前保持段的剩余时间、原始总时间和此前已完成时间。
        double hover_duration{0.0};
        double hover_total_duration{0.0};
        double hover_elapsed_before{0.0};

        Clock::time_point hold_started_at{};

        std::optional<std::uint64_t> trajectory_id;
        std::optional<MotionExecutor::ExecutionEvent> terminal_event;

        Clock::time_point dispatched_at{};
        Clock::time_point event_received_at{};
        Clock::time_point next_feedback_at{};

        // 非空时故障优先，即使后续安全停止也不报告正常完成。
        std::string failure;

        // value 的含义：
        // Takeoff：相对高度。
        // GoTo/Continue：剩余距离。
        // Hover：已完成保持时间。
        std::function<void(
            const std::string& phase,
            float progress,
            double value)> feedback;

        std::function<void(
            ActionState,
            ErrorCode,
            const std::string&)> finish;
    };

    struct PausedTask
    {
        ActionType source_type{ActionType::GOTO};

        // 保存任务描述，不保存旧轨迹 ID、ControlTrajectory 或 GoalHandle。
        PlanningRequest request;

        MotionExecutor::ExecutionConstraints constraints;
        double confirmation_timeout{2.0};

        double height{0.0};
        double start_z{0.0};
        double progress{0.0};

        bool is_hover{false};

        double hover_remaining{0.0};
        double hover_total_duration{0.0};
        double hover_elapsed{0.0};
    };

    struct StopOperation
    {
        GoalUUID uuid{};

        bool attached{false};
        bool started{false};
        bool cancel_requested{false};

        // Hover 保持期间的客户端取消复用停止确认，
        // 但不创建额外的 ROS Action Goal。
        bool internal_cancel{false};

        Clock::time_point requested_at{};
        std::int64_t status_stamp_floor{0};

        std::optional<Clock::time_point> stable_since;

        std::function<void(
            std::uint8_t stage,
            const std::string& message)> feedback;

        std::function<void(
            ActionState,
            ErrorCode,
            const std::string&)> finish;
    };

    // Action Server 创建及回调路由。
    template<typename ActionT>
    typename rclcpp_action::Server<ActionT>::SharedPtr createServer(
        const std::string& name);

    // Goal 准入。
    rclcpp_action::GoalResponse handleArmGoal(
        const GoalUUID& uuid,
        bool desired_arm);

    rclcpp_action::GoalResponse handleTakeoffGoal(
        const GoalUUID& uuid,
        const Takeoff::Goal& goal);

    rclcpp_action::GoalResponse handleGoToGoal(
        const GoalUUID& uuid,
        const GoTo::Goal& goal);

    rclcpp_action::GoalResponse handleStopGoal(
        const GoalUUID& uuid);

    rclcpp_action::GoalResponse handleContinueGoal(
        const GoalUUID& uuid);

    rclcpp_action::GoalResponse handleHoverGoal(
        const GoalUUID& uuid,
        const Hover::Goal& goal);

    // Cancel 准入。
    rclcpp_action::CancelResponse handleArmCancel(
        const GoalUUID& uuid);

    rclcpp_action::CancelResponse handleMotionCancel(
        const GoalUUID& uuid);

    rclcpp_action::CancelResponse handleStopCancel(
        const GoalUUID& uuid);

    // GoalHandle 绑定。
    template<typename ActionT>
    void attachArmGoal(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<ActionT>>& handle);

    template<typename ActionT>
    void attachMotionGoal(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<ActionT>>& handle);

    void attachStopGoal(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<Stop>>& handle);

    // 定时驱动。
    void tick();
    void tickArm();
    void tickMotion();
    void tickStop();
    void tickHoverHold();

    // 真实观测及执行事件。
    void refreshSnapshot();
    void processExecutionEvents();
    void latchFault(const std::string& reason);

    // 规划及执行确认。
    void startMotion();
    bool motionAtTarget() const;
    bool stopObservationConfirmed() const;

    // 真实反馈和任务级进度。
    void publishFeedback();
    void publishMotionFeedback();
    double motionProgress() const;

    // 暂停任务快照。
    std::optional<PausedTask> capturePausedTask() const;

    // 服务请求清理及 Action 终态。
    void clearPendingRequest();

    void finishActive(
        ActionState state,
        ErrorCode error,
        const std::string& message);

    void finishMotion(
        ActionState state,
        ErrorCode error,
        const std::string& message);

    void finishStop(
        ActionState state,
        ErrorCode error,
        const std::string& message);

    void failStop(const std::string& reason);

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
    rclcpp_action::Server<Continue>::SharedPtr continue_server_;
    rclcpp_action::Server<Hover>::SharedPtr hover_server_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Action 回调和管理定时器使用默认互斥回调组。
    // active_ 与 motion_ 不同时占用。
    std::optional<ArmOperation> active_;
    std::optional<MotionOperation> motion_;

    // Stop 是对当前运动的停止事务，不创建第二个活动轨迹。
    std::optional<StopOperation> stop_;

    // 唯一暂停槽。
    std::optional<PausedTask> paused_;

    FlightPhase logged_phase_{FlightPhase::UNKNOWN};

    // 跨节点事件仅通过此队列传递。
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

    double stop_timeout_{8.0};
    double stop_settle_duration_{0.5};
};

}  // namespace drone_node::exec