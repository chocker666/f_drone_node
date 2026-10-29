
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "drone_node/msg/control_trajectory.hpp"

namespace drone_node::exec::planner
{

class QuinticPlanner
{
public:
    using ControlTrajectory =
        drone_node::msg::ControlTrajectory;

    using Coefficients = std::array<double, 6>;

    enum class YawDirection
    {
        SHORTEST,
        POSITIVE,
        NEGATIVE
    };

    struct BoundaryState
    {
        Eigen::Vector3d position{
            Eigen::Vector3d::Zero()
        };

        Eigen::Vector3d velocity{
            Eigen::Vector3d::Zero()
        };

        Eigen::Vector3d acceleration{
            Eigen::Vector3d::Zero()
        };

        double yaw{0.0};
        double yaw_rate{0.0};
        double yaw_acceleration{0.0};
    };

    struct Limits
    {
        // Non-positive values disable the corresponding limit.
        double max_velocity{0.0};
        double max_acceleration{0.0};
        double max_jerk{0.0};

        double max_yaw_rate{0.0};
        double max_yaw_acceleration{0.0};
    };

    struct Request
    {
        BoundaryState start;
        BoundaryState goal;

        double duration{0.0};

        YawDirection yaw_direction{
            YawDirection::SHORTEST
        };

        Limits limits;

        std::uint64_t trajectory_id{0};

        std::string frame_id{"map"};

        std::string controller_id{
            "position_yaw_pid"
        };

        std::string config_name{"default"};
    };

    // Construct a single-segment quintic trajectory.
    //
    // Throws std::invalid_argument when the request
    // violates the input or configured motion constraints.
    //
    // Does not modify the request.
    static ControlTrajectory plan(
        const Request& request);

    // Solve:
    // p(0)=p0, p'(0)=v0, p''(0)=a0
    // p(T)=pf, p'(T)=vf, p''(T)=af
    static Coefficients solve(
        double p0,
        double v0,
        double a0,
        double pf,
        double vf,
        double af,
        double duration);

    // Evaluate the polynomial or its derivative.
    // derivative = 0, 1, 2 or 3.
    static double evaluate(
        const Coefficients& coefficients,
        double time,
        unsigned int derivative = 0);

private:
    static double unwrapYaw(
        double start_yaw,
        double target_yaw,
        YawDirection direction);

    static void validateRequest(
        const Request& request);

    static void validateLimits(
        const Request& request,
        const Coefficients& x,
        const Coefficients& y,
        const Coefficients& z,
        const Coefficients& yaw);
};

}  // namespace drone_node::exec::planner
