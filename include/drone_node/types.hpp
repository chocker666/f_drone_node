#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_node
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

// Existing sampler output. Linear quantities use local ENU.
// Yaw uses radians; yaw rate uses radians per second.
struct ReferenceState
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // m
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // m/s
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  // m/s^2

    double yaw{0.0};
    double yaw_rate{0.0};
};

// Describes which reference quantities are supplied.
// Zero is a valid reference value; it does not mean unavailable.
struct ReferenceValidity
{
    bool position{false};
    bool velocity{false};
    bool acceleration{false};
    bool yaw{false};
    bool yaw_rate{false};
};

struct FeedbackValidity
{
    bool valid{false};
    SteadyTime received_at{};
};

// Value copied from StateCache once per control cycle.
// This structure itself performs no locking or state estimation.
struct StateSnapshot
{
    SteadyTime captured_at{};

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // ENU, m
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};  // ENU, m/s

    // Rotation from FLU body coordinates to ENU world coordinates.
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

    FeedbackValidity position_feedback{};
    FeedbackValidity velocity_feedback{};
    FeedbackValidity attitude_feedback{};

    // Continuous measured yaw and estimated yaw rate.
    // Identity orientation alone does not make these available.
    double unwrapped_yaw{0.0};      // rad
    double measured_yaw_rate{0.0};  // rad/s
    bool yaw_valid{false};
};

enum class ControlMode : std::uint8_t
{
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    ATTITUDE = 3,
    BODY_RATE = 4
};

// Physical controller input; independent of ROS message types.
// The caller supplies elapsed time measured with SteadyClock.
struct PhysicalControllerInput
{
    StateSnapshot state{};
    ReferenceState reference{};
    ReferenceValidity reference_validity{};
    double dt{0.0};  // s; must be positive and finite before update
};

// Only quantities selected by mode are interpreted.
// The publisher performs rad -> deg and rad/s -> deg/s conversion.
struct PhysicalControllerOutput
{
    ControlMode mode{ControlMode::VELOCITY};

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // ENU, m
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // ENU, m/s
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  // ENU, m/s^2

    bool use_jerk{false};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};  // ENU, m/s^3

    double yaw{0.0};       // rad
    double yaw_rate{0.0};  // rad/s

    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};  // FLU, rad/s
    Eigen::Vector3d thrust{Eigen::Vector3d::Zero()};     // FLU, normalized
};

// Signed continuous yaw error; position error is a norm.
// NaN means that the corresponding error is unavailable.
struct TrackingErrors
{
    double position_error{
        std::numeric_limits<double>::quiet_NaN()};
    double yaw_error{
        std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace drone_node