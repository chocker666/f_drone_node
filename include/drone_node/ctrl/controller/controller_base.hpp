#pragma once

#include <cstdint>

#include "drone_node/types.hpp"

namespace drone_node::controller
{

// ENU positive yaw is counterclockwise when viewed from above.
enum class YawDirection : std::uint8_t
{
    SHORTEST = 0,
    CW = 1,
    CCW = 2
};

struct ControllerResult
{
    PhysicalControllerOutput output{};
    TrackingErrors errors{};
};

// Physical controller interface.
//
// Inputs use ENU linear quantities and rad/rad/s angular quantities.
// Outputs retain these internal units; ROS conversion belongs to
// FlightController's Setpoint publisher.
//
// This interface owns neither ROS communication nor the node FSM.
class ControllerBase
{
public:
    virtual ~ControllerBase() = default;

    // Clear algorithm history and reference initialization.
    // The configured parameters remain unchanged.
    virtual void reset() = 0;

    // Initialize a newly activated reference from the state snapshot.
    // Position/yaw controllers use direction to select the initial
    // continuous yaw target.
    //
    // The caller invokes this after reset and before the first update.
    // Subsequent reference yaw values must remain continuous within
    // the active trajectory.
    //
    // Throws on unsupported or invalid initialization input.
    virtual void initialize(
        const PhysicalControllerInput& input,
        YawDirection direction) = 0;

    // Compute physical control quantities and tracking errors.
    //
    // The caller checks feedback freshness and supplies a positive,
    // finite dt. Implementations validate the quantities they require
    // and throw if computation cannot produce a valid result.
    virtual ControllerResult update(
        const PhysicalControllerInput& input) = 0;
};

}  // namespace drone_node::controller