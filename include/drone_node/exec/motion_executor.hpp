#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "drone_node/msg/control_command.hpp"
#include "drone_node/msg/control_status.hpp"
#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/srv/set_arming.hpp"

#include "drone_node/exec/planner/planner_types.hpp"
#include "drone_node/exec/action_types.hpp"


namespace drone_node::exec
{

class MotionExecutor : public rclcpp::Node
{
public:
    using Clock = std::chrono::steady_clock;
    using ControlTrajectory = drone_node::msg::ControlTrajectory;
    using ControlCommand = drone_node::msg::ControlCommand;
    using ControlStatus = drone_node::msg::ControlStatus;
    using SetArming = drone_node::srv::SetArming;
    using ArmClient = rclcpp::Client<SetArming>;
    using ArmRequest = ArmClient::FutureAndRequestId;

    template<typename MessageT>
    struct CachedState
    {
        MessageT message{};
        Clock::time_point received_at{};
        bool received{false};
        bool valid{false};
    };

    struct StateSnapshot
    {
        CachedState<geometry_msgs::msg::PointStamped> position;
        CachedState<geometry_msgs::msg::Vector3Stamped> velocity;
        CachedState<geometry_msgs::msg::QuaternionStamped> attitude;
        CachedState<std_msgs::msg::Bool> armed;
        CachedState<std_msgs::msg::Bool> landed;
        CachedState<std_msgs::msg::Bool> connected;
        CachedState<ControlStatus> control_status;
    };

        enum class ExecutionPhase
    {
        IDLE,
        WAITING_TRACKING,
        TRACKING,
        WAITING_HOVER
    };

    enum class StopReason
    {
        COMPLETED,
        STOP,
        CLIENT_CANCEL,
        FAILURE
    };

    enum class ExecutionOutcome
    {
        COMPLETED,
        STOPPED,
        CANCELED,
        FAILED
    };

    struct ExecutionConstraints
    {
        double position_tolerance{0.15};  // m
        double yaw_tolerance{0.10};       // rad
        double velocity_tolerance{0.15};  // m/s
        double settle_duration{0.5};      // s
        double timeout{30.0};             // s
        // Takeoff must confirm actual airborne state before success.
        // Other tasks leave this disabled.
        bool require_airborne{false};
    };



    struct ExecutionEvent
    {
        std::uint64_t trajectory_id{0};
        ExecutionOutcome outcome{ExecutionOutcome::FAILED};
        StopReason stop_reason{StopReason::FAILURE};

        // true 只表示已经确认新鲜 HOVER 和稳定状态。
        bool safe_stop_confirmed{false};
        std::string message;
    };

    struct ExecutionSnapshot
    {
        ExecutionPhase phase{ExecutionPhase::IDLE};
        std::uint64_t trajectory_id{0};
        StopReason stop_reason{StopReason::COMPLETED};
        std::string failure;
    };

    using ExecutionCallback =
        std::function<void(const ExecutionEvent&)>;

    explicit MotionExecutor(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // 仅发布，不等待 Ctrl 确认或轨迹执行完成。
    void sendTrajectory(const ControlTrajectory& trajectory);
    void sendCommand(const ControlCommand& command);

    // 返回缓存副本；调用方不持有内部锁或缓存引用。
    StateSnapshot getStateSnapshot() const;

    bool armServiceReady() const;

    // 服务不可用时返回 nullopt；请求发出后立即返回 future 和请求 ID。
    // future 中的响应仅表示服务结果，不代替实际 armed 状态确认。
    std::optional<ArmRequest> requestArm(bool arm);

    // 请求被上层放弃或超时后，清理本地 pending request。
    // 这不会撤销已经发送给 Inter 的请求。
    bool removePendingArmRequest(std::int64_t request_id);

    // 第一版只接受 map/ENU POINT。
    // 成功时分配新的轨迹 ID，并立即发布；返回不代表执行成功。
    std::optional<std::uint64_t> startExecution(
        ControlTrajectory trajectory,
        const ExecutionConstraints& constraints,
        std::string& rejection_reason);

    // 接收规划结果，转换为 Ctrl 消息后复用现有执行入口。
    std::optional<std::uint64_t> startExecution(
        const PlannedMotion& motion,
        const ExecutionConstraints& constraints,
        std::string& rejection_reason);

    // 仅允许 STOP 或 CLIENT_CANCEL。
    // 必须提供当前轨迹 ID，避免迟到取消影响下一项任务。
    bool requestStop(
        std::uint64_t trajectory_id,
        StopReason reason);

    ExecutionSnapshot getExecutionSnapshot() const;

    // 回调在内部锁释放后执行；回调应短小，不阻塞。
    void setExecutionCallback(ExecutionCallback callback);

    // 面向 FSM 的真实观测快照，不改变 FSM 状态。
    VehicleSnapshot getVehicleSnapshot() const;
private:

    struct ExecutionContext
    {
        ControlTrajectory trajectory;
        ExecutionConstraints constraints;

        ExecutionPhase phase{ExecutionPhase::WAITING_TRACKING};
        StopReason stop_reason{StopReason::COMPLETED};
        std::string failure;

        Clock::time_point started_at{};
        Clock::time_point phase_started_at{};
        std::int64_t status_stamp_floor{0};

        bool hover_confirmed{false};
        bool stability_started{false};
        Clock::time_point stable_since{};

        geometry_msgs::msg::Point hover_position;
        double hover_yaw{0.0};
    };

    template<typename MessageT>
    bool fresh(
        const CachedState<MessageT>& cache,
        Clock::time_point time) const
    {
        return cache.received &&
               cache.valid &&
               time >= cache.received_at &&
               std::chrono::duration<double>(
                   time - cache.received_at).count() <= state_timeout_;
    }

    static std::int64_t statusStamp(const ControlStatus& status);
    static double yawOf(
        const geometry_msgs::msg::QuaternionStamped& attitude);
    static double yawDistance(double first, double second);
    static bool validConstraints(
        const ExecutionConstraints& constraints);

    std::string stateProblem(
        const StateSnapshot& state,
        Clock::time_point time) const;

    bool statusAfter(
        const StateSnapshot& state,
        const ExecutionContext& context) const;

    bool settled(
        ExecutionContext& context,
        const StateSnapshot& state,
        Clock::time_point time,
        bool hover);

    void beginStopping(
        ExecutionContext& context,
        const StateSnapshot& state,
        Clock::time_point time,
        StopReason reason,
        const std::string& failure = {});

    ExecutionEvent finishExecution(bool safe_stop_confirmed);
    void updateExecution();

    mutable std::mutex execution_mutex_;
    std::optional<ExecutionContext> execution_;
    ExecutionCallback execution_callback_;
    rclcpp::TimerBase::SharedPtr execution_timer_;
    template<typename MessageT>
    bool statusFresh(
        const CachedState<MessageT>& cache,
        Clock::time_point time) const
    {
        return cache.received &&
               cache.valid &&
               time >= cache.received_at &&
               std::chrono::duration<double>(
                   time - cache.received_at).count() <= status_timeout_;
    }

    // 以下两个方法在 execution_mutex_ 保护下使用。
    VehicleSnapshot buildVehicleSnapshot(
        const StateSnapshot& state,
        Clock::time_point time) const;

    void updateHoverObservation(
        const StateSnapshot& state,
        Clock::time_point time);

    double status_timeout_{1.5};

    double hover_position_tolerance_{0.15};
    double hover_yaw_tolerance_{0.10};
    double hover_velocity_tolerance_{0.15};
    double hover_settle_duration_{0.5};

    std::optional<Clock::time_point> hover_stable_since_;
    geometry_msgs::msg::Point hover_anchor_position_;
    double hover_anchor_yaw_{0.0};
    bool observed_hover_confirmed_{false};

    double state_timeout_{0.3};
    double tracking_ack_timeout_{1.0};
    double hover_ack_timeout_{1.0};
    double stop_timeout_{5.0};

    template<typename MessageT>
    void updateCache(
        CachedState<MessageT>& cache,
        const MessageT& message,
        bool valid)
    {
        const auto received_at = Clock::now();
        std::lock_guard<std::mutex> lock(state_mutex_);
        cache.message = message;
        cache.received_at = received_at;
        cache.received = true;
        cache.valid = valid;
    }

    mutable std::mutex state_mutex_;
    StateSnapshot state_;

    rclcpp::Publisher<ControlTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<ControlCommand>::SharedPtr command_pub_;

    rclcpp::Subscription<ControlStatus>::SharedPtr control_status_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connected_sub_;

    ArmClient::SharedPtr arm_client_;
};

}  // namespace drone_node::exec