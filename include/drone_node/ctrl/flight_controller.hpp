
#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include "drone_node/msg/setpoint.hpp"

#include "drone_node/msg/control_trajectory.hpp"
#include "drone_node/msg/control_status.hpp"
#include "drone_node/msg/control_command.hpp"
#include "drone_node/msg/fixed_trajectory.hpp"

#include "drone_node/ctrl/algorithm/controller_base.hpp"
#include "drone_node/ctrl/algorithm/pid_controller.hpp"

#include "drone_node/ctrl/trajectory_manager.hpp"
#include "drone_node/ctrl/trajectory_sampler/fixed_sampler.hpp"

#include "drone_node/types.hpp"


class FlightController : public rclcpp::Node
{
public:
    explicit FlightController(
        const rclcpp::NodeOptions& options =
            rclcpp::NodeOptions());

private:
    // ========================================================
    // Type aliases
    // ========================================================

    using Setpoint = drone_node::msg::Setpoint;
    using PointStamped = geometry_msgs::msg::PointStamped;
    using VectorStamped = geometry_msgs::msg::Vector3Stamped;
    using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;



    using ControlTrajectory =
        drone_node::msg::ControlTrajectory;

    using ControlStatus =
        drone_node::msg::ControlStatus;

    using ControlCommand =
        drone_node::msg::ControlCommand;

    using FixedTrajectory =
        drone_node::msg::FixedTrajectory;

    using FixedSampler =
        drone_node::trajectory_sampler::FixedSampler;

    using SteadyClock =
        std::chrono::steady_clock;

    struct Feedback
    {
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        geometry_msgs::msg::Quaternion orientation;
    };

    // ========================================================
    // ROS callbacks
    // ========================================================

    // Vehicle position, velocity and attitude feedback.
    void position_callback(const PointStamped::SharedPtr msg);
    void velocity_callback(const VectorStamped::SharedPtr msg);
    void attitude_callback(const QuaternionStamped::SharedPtr msg);

    // Receive and replace the active trajectory.
    void trajectory_callback(
        const ControlTrajectory::SharedPtr msg);

    // CANCEL / EXIT_IDLE / RESET_FAULT.
    void command_callback(
        const ControlCommand::SharedPtr msg);

    // Runtime setpoint output permission.
    void output_enable_callback(
        const std_msgs::msg::Bool::SharedPtr msg);

    // PX4 arming status feedback.
    void armed_callback(
        const std_msgs::msg::Bool::SharedPtr msg);


    // ========================================================
    // Periodic loops
    // ========================================================

    // Main position and yaw control loop.
    void control_loop();

    // Controller status feedback.
    void status_loop();


    // ========================================================
    // Setpoint output
    // ========================================================

    // Velocity: ENU, m/s.
    // Yaw rate: ENU, rad/s.
    void publish_velocity_setpoint(
        const Eigen::Vector3d& velocity,
        double yaw_rate);


    // ========================================================
    // State machine
    // ========================================================

    // IDLE <-> TRACKING <-> HOVER
    //
    // IDLE / TRACKING / HOVER -> FAULT
    // FAULT -> IDLE (safe reset only)

    void change_control_state(uint8_t state);

    // Enter fixed-position hover from TRACKING.
    void enter_hover();

    // Clear references and return to IDLE.
    void enter_idle();

    // Enter fault protection.
    void enter_fault(uint16_t fault_code);

    // Reset controller runtime states without changing
    // the configured PID parameters.
    void reset_control_runtime();


    // ========================================================
    // Reference management
    // ========================================================

    // Capture the current position and yaw once.
    // Used to construct the internal hover sampler.
    FixedTrajectory make_current_fixed_reference() const;


    // ========================================================
    // Vehicle state validation
    // ========================================================

    bool state_is_fresh() const;


    // ========================================================
    // Yaw utilities
    // ========================================================

    // Wrap angle into (-pi, pi].
    static double wrap_pi(double angle);

    // Extract ENU yaw from the vehicle quaternion.
    static double yaw_from_quaternion(
        const geometry_msgs::msg::Quaternion& q);

    // Select the signed angular displacement according
    // to the requested rotation direction.
    static double select_yaw_delta(
        double current_yaw,
        double target_yaw,
        uint8_t direction);

    // Maintain continuous measured yaw across +/-pi.
    void update_unwrapped_yaw(double measured_yaw);

    // Initialize the continuous yaw reference.
    // TRACKING uses the active trajectory direction.
    // HOVER uses SHORTEST.
    void initialize_yaw_reference(
        double target_yaw,
        uint8_t yaw_direction);


    // ========================================================
    // Scheduling and safety parameters
    // ========================================================

    double control_frequency_{50.0};

    double status_frequency_{10.0};

    double state_timeout_{0.2};

    // Initial value loaded from ROS parameters.
    // Can subsequently be updated through the
    // runtime output-enable topic.
    bool enable_setpoint_stream_{false};


    // ========================================================
    // Controllers
    // ========================================================

    // Position PID: position error -> velocity command.
    std::unique_ptr<
        drone_node::algorithm::ControllerBase<3>>
        controller_;

    // Yaw PID: yaw error -> yaw-rate command.
    drone_node::algorithm::PIDController<1>
        yaw_controller_;


    // ========================================================
    // Reference sources
    // ========================================================

    // Upper-layer trajectory reference.
    // Used in TRACKING.
    drone_node::TrajectoryManager trajectory_manager_;

    // Internal fixed reference.
    // Used in HOVER.
    //
    // The reference is captured only when entering HOVER.
    // It must not follow the current vehicle position
    // during subsequent control cycles.
    std::unique_ptr<FixedSampler> hover_sampler_;


    // ========================================================
    // Vehicle feedback
    // ========================================================

    Feedback latest_vehicle_state_{};
    std::array<bool, 3> feedback_received_{};
    std::array<SteadyClock::time_point, 3> feedback_time_{};


    // ========================================================
    // PX4 arming feedback
    // ========================================================

    bool armed_{false};

    bool has_armed_feedback_{false};


    // ========================================================
    // Continuous yaw measurement
    // ========================================================

    bool has_yaw_measurement_{false};

    double previous_wrapped_yaw_{0.0};

    double unwrapped_yaw_{0.0};

    // Estimated yaw rate, rad/s.
    double measured_yaw_rate_{0.0};

    SteadyClock::time_point last_yaw_receive_time_{};


    // ========================================================
    // Continuous yaw reference
    // ========================================================

    bool yaw_reference_initialized_{false};

    double target_unwrapped_yaw_{0.0};


    // ========================================================
    // Controller state
    // ========================================================

    // Initial state: IDLE.
    uint8_t control_state_{ControlStatus::IDLE};

    uint16_t fault_code_{ControlStatus::FAULT_NONE};


    // ========================================================
    // Latest reference and tracking errors
    // ========================================================

    drone_node::ReferenceState latest_reference_{};

    bool has_reference_{false};

    // Position tracking error norm, m.
    // NaN means unavailable.
    double position_error_{
        std::numeric_limits<double>::quiet_NaN()};

    // Continuous signed yaw error, rad.
    // NaN means unavailable.
    double yaw_error_{
        std::numeric_limits<double>::quiet_NaN()};


    // ========================================================
    // Control timing
    // ========================================================

    SteadyClock::time_point last_control_time_{};

    bool has_last_control_time_{false};


    // ========================================================
    // ROS subscriptions
    // ========================================================

    rclcpp::Subscription<PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<VectorStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<QuaternionStamped>::SharedPtr attitude_sub_;

    rclcpp::Subscription<ControlTrajectory>::SharedPtr
        trajectory_sub_;

    rclcpp::Subscription<ControlCommand>::SharedPtr
        command_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        output_enable_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        armed_sub_;


    // ========================================================
    // ROS publishers
    // ========================================================

    rclcpp::Publisher<Setpoint>::SharedPtr trajectory_setpoint_pub_;

    rclcpp::Publisher<ControlStatus>::SharedPtr
        control_status_pub_;


    // ========================================================
    // ROS timers
    // ========================================================

    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::TimerBase::SharedPtr status_timer_;
};
