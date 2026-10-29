
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
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

#include "drone_node/exec/action_types.hpp"

namespace drone_node::exec
{

class MotionExecutor
{
public:
    using Clock = std::chrono::steady_clock;

    using ControlTrajectory =
        drone_node::msg::ControlTrajectory;

    using ControlCommand =
        drone_node::msg::ControlCommand;

    using ControlStatus =
        drone_node::msg::ControlStatus;

    using SetArming =
        drone_node::srv::SetArming;

    using ArmClient =
        rclcpp::Client<SetArming>;

    using ArmRequest =
        ArmClient::FutureAndRequestId;

    // ========================================================
    // State cache
    // ========================================================

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
        CachedState<
            geometry_msgs::msg::PointStamped> position;

        CachedState<
            geometry_msgs::msg::Vector3Stamped> velocity;

        CachedState<
            geometry_msgs::msg::QuaternionStamped> attitude;

        CachedState<
            std_msgs::msg::Bool> armed;

        CachedState<
            std_msgs::msg::Bool> landed;

        CachedState<
            std_msgs::msg::Bool> connected;

        CachedState<ControlStatus> control_status;
    };

    // ========================================================
    // Execution state
    // ========================================================

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

    // ========================================================
    // Execution constraints
    // ========================================================

    struct ExecutionConstraints
    {
        double position_tolerance{0.15};  // m
        double yaw_tolerance{0.10};       // rad
        double velocity_tolerance{0.15};  // m/s

        double settle_duration{0.5};      // s
        double timeout{30.0};             // s

        // Takeoff / GoTo:
        // Normal completion requires airborne observation.
        bool require_airborne{false};

        // Land:
        // Complete based on ground-contact observation,
        // rather than reaching the polynomial endpoint.
        bool complete_on_landed{false};
    };

    // ========================================================
    // Execution result
    // ========================================================

    struct ExecutionEvent
    {
        std::uint64_t trajectory_id{0};

        ExecutionOutcome outcome{
            ExecutionOutcome::FAILED
        };

        StopReason stop_reason{
            StopReason::FAILURE
        };

        // True only when fresh HOVER feedback and
        // stable vehicle state have been confirmed.
        bool safe_stop_confirmed{false};

        std::string message;
    };

    struct ExecutionSnapshot
    {
        ExecutionPhase phase{
            ExecutionPhase::IDLE
        };

        std::uint64_t trajectory_id{0};

        StopReason stop_reason{
            StopReason::COMPLETED
        };

        std::string failure;
    };

    using ExecutionCallback =
        std::function<void(const ExecutionEvent&)>;

    // ========================================================
    // Construction and lifecycle
    // ========================================================

    explicit MotionExecutor(
        rclcpp::Node& node,
        rclcpp::CallbackGroup::SharedPtr motion_group);

    // Executor workers must be stopped before calling.
    // Idempotent; does not wait for active operations.
    void stop();

    // ========================================================
    // CTRL communication
    // ========================================================

    // Publish a complete polynomial trajectory.
    //
    // This function only publishes the message.
    // It does not wait for CTRL acceptance or completion.
    void sendTrajectory(
        const ControlTrajectory& trajectory);

    void sendCommand(
        const ControlCommand& command);

    // ========================================================
    // Vehicle state and arming
    // ========================================================

    // Return a copy of cached state.
    // Callers do not retain references to internal storage.
    StateSnapshot getStateSnapshot() const;

    bool armServiceReady() const;

    // Return nullopt if the service is unavailable.
    //
    // A service response does not replace verification
    // of the actual armed state.
    std::optional<ArmRequest> requestArm(bool arm);

    // Remove a pending local service request.
    // Does not retract a request already sent to INTER.
    bool removePendingArmRequest(
        std::int64_t request_id);

    // ========================================================
    // Trajectory execution
    // ========================================================

    // Accept a complete ControlTrajectory generated by
    // MotionPlanner / QuinticPlanner.
    //
    // Responsibilities:
    // 1. Validate trajectory structure and constraints.
    // 2. Allocate a unique trajectory ID.
    // 3. Extract terminal position and yaw.
    // 4. Record total trajectory duration.
    // 5. Publish the trajectory to CTRL.
    //
    // Returning an ID means the request was submitted.
    // It does not mean CTRL accepted or executed it.
    std::optional<std::uint64_t> startExecution(
        ControlTrajectory trajectory,
        const ExecutionConstraints& constraints,
        std::string& rejection_reason);

    // Request STOP or CLIENT_CANCEL for the specified
    // trajectory ID.
    //
    // The ID prevents stale callers from canceling a
    // different local execution.
    bool requestStop(
        std::uint64_t trajectory_id,
        StopReason reason);

    ExecutionSnapshot getExecutionSnapshot() const;

    // Callback is invoked after releasing internal locks.
    // It should not perform long-running operations.
    void setExecutionCallback(
        ExecutionCallback callback);

    // Read actual vehicle observations for the FSM.
    // Does not change FSM state.
    VehicleSnapshot getVehicleSnapshot() const;

private:
    // ========================================================
    // Internal dependencies
    // ========================================================

    rclcpp::Node& node_;

    rclcpp::CallbackGroup::SharedPtr motion_group_;

    // ========================================================
    // Execution context
    // ========================================================

    struct ExecutionContext
    {
        // Complete polynomial trajectory sent to CTRL.
        ControlTrajectory trajectory;

        ExecutionConstraints constraints;

        // Terminal reference extracted from the final
        // polynomial segment.
        //
        // Used by normal completion and HOVER checks.
        geometry_msgs::msg::Point target_position;
        double target_yaw{0.0};

        // Sum of all polynomial segment durations.
        double trajectory_duration{0.0};

        ExecutionPhase phase{
            ExecutionPhase::WAITING_TRACKING
        };

        StopReason stop_reason{
            StopReason::COMPLETED
        };

        std::string failure;

        // Local time when execution was submitted.
        Clock::time_point started_at{};

        // Local time when the current phase began.
        Clock::time_point phase_started_at{};

        // Local time when matching CTRL TRACKING status
        // was first confirmed.
        //
        // This is not the exact CTRL activation time.
        Clock::time_point tracking_confirmed_at{};

        // Reject CTRL status observations older than
        // the current phase transition.
        std::int64_t status_stamp_floor{0};

        bool hover_confirmed{false};

        bool stability_started{false};

        Clock::time_point stable_since{};

        // HOVER confirmation target.
        //
        // Normal completion:
        //   use trajectory terminal reference.
        //
        // STOP / CLIENT_CANCEL:
        //   use the actual stopping position and yaw.
        //
        // Land:
        //   use the observed ground-contact position.
        geometry_msgs::msg::Point hover_position;

        double hover_yaw{0.0};
    };

    // ========================================================
    // State freshness helpers
    // ========================================================

    template<typename MessageT>
    bool fresh(
        const CachedState<MessageT>& cache,
        Clock::time_point time) const
    {
        return cache.received &&
               cache.valid &&
               time >= cache.received_at &&
               std::chrono::duration<double>(
                   time - cache.received_at).count()
                   <= state_timeout_;
    }

    template<typename MessageT>
    bool statusFresh(
        const CachedState<MessageT>& cache,
        Clock::time_point time) const
    {
        return cache.received &&
               cache.valid &&
               time >= cache.received_at &&
               std::chrono::duration<double>(
                   time - cache.received_at).count()
                   <= status_timeout_;
    }

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

    // ========================================================
    // Static validation and conversion helpers
    // ========================================================

    static std::int64_t statusStamp(
        const ControlStatus& status);

    static double yawOf(
        const geometry_msgs::msg::QuaternionStamped&
            attitude);

    static double yawDistance(
        double first,
        double second);

    static bool validConstraints(
        const ExecutionConstraints& constraints);

    // ========================================================
    // Execution-state helpers
    // ========================================================

    std::string stateProblem(
        const StateSnapshot& state,
        Clock::time_point time,
        bool allow_ground_disarmed = false) const;

    bool statusAfter(
        const StateSnapshot& state,
        const ExecutionContext& context) const;

    // Check whether actual position, yaw and speed
    // remain within tolerance for the required duration.
    //
    // hover == false:
    //   compare against polynomial terminal reference.
    //
    // hover == true:
    //   compare against HOVER confirmation reference.
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

    ExecutionEvent finishExecution(
        bool safe_stop_confirmed);

    void updateExecution();

    // ========================================================
    // Vehicle observation helpers
    // ========================================================

    // Called with execution_mutex_ held.
    VehicleSnapshot buildVehicleSnapshot(
        const StateSnapshot& state,
        Clock::time_point time) const;

    // Called with execution_mutex_ held.
    void updateHoverObservation(
        const StateSnapshot& state,
        Clock::time_point time);

    // ========================================================
    // Execution synchronization
    // ========================================================

    mutable std::mutex execution_mutex_;

    std::optional<ExecutionContext> execution_;

    ExecutionCallback execution_callback_;

    rclcpp::TimerBase::SharedPtr execution_timer_;

    // ========================================================
    // HOVER observation
    // ========================================================

    double status_timeout_{1.5};

    double hover_position_tolerance_{0.15};
    double hover_yaw_tolerance_{0.10};
    double hover_velocity_tolerance_{0.15};
    double hover_settle_duration_{0.5};

    std::optional<Clock::time_point>
        hover_stable_since_;

    geometry_msgs::msg::Point
        hover_anchor_position_;

    double hover_anchor_yaw_{0.0};

    bool observed_hover_confirmed_{false};

    // ========================================================
    // Execution timing
    // ========================================================

    double state_timeout_{0.3};

    double tracking_ack_timeout_{1.0};
    double hover_ack_timeout_{1.0};
    double stop_timeout_{5.0};

    // ========================================================
    // Cached vehicle state
    // ========================================================

    mutable std::mutex state_mutex_;

    StateSnapshot state_;

    // ========================================================
    // ROS publishers
    // ========================================================

    rclcpp::Publisher<ControlTrajectory>::SharedPtr
        trajectory_pub_;

    rclcpp::Publisher<ControlCommand>::SharedPtr
        command_pub_;

    // ========================================================
    // ROS subscriptions
    // ========================================================

    rclcpp::Subscription<ControlStatus>::SharedPtr
        control_status_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped>::SharedPtr
        position_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::Vector3Stamped>::SharedPtr
        velocity_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::QuaternionStamped>::SharedPtr
        attitude_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Bool>::SharedPtr
        armed_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Bool>::SharedPtr
        landed_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Bool>::SharedPtr
        connected_sub_;

    // ========================================================
    // INTER service client
    // ========================================================

    ArmClient::SharedPtr arm_client_;
};

}  // namespace drone_node::exec
