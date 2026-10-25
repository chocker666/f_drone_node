#include "drone_node/exec/flight_state_machine.hpp"

namespace drone_node::exec
{

bool FlightStateMachine::observation_fresh_locked() const
{
    const auto now = std::chrono::steady_clock::now();

    return snapshot_.captured_at !=
               std::chrono::steady_clock::time_point{} &&
           now >= snapshot_.captured_at &&
           now - snapshot_.captured_at <= snapshot_timeout_;
}

bool FlightStateMachine::basic_observation_valid_locked() const
{
    return observation_fresh_locked() &&
           snapshot_.connected_valid &&
           snapshot_.connected &&
           snapshot_.armed_valid &&
           snapshot_.landed_valid;
}

bool FlightStateMachine::grounded_confirmed_locked() const
{
    return basic_observation_valid_locked() &&
           snapshot_.landed;
}

bool FlightStateMachine::flight_feedback_valid_locked() const
{
    return basic_observation_valid_locked() &&
           snapshot_.armed &&
           snapshot_.position_valid &&
           snapshot_.velocity_valid &&
           snapshot_.attitude_valid &&
           snapshot_.control_status_valid &&
           snapshot_.control_available &&
           !snapshot_.control_fault;
}

bool FlightStateMachine::airborne_control_valid_locked() const
{
    return flight_feedback_valid_locked() &&
           !snapshot_.landed;
}

bool FlightStateMachine::hovering_confirmed_locked() const
{
    return airborne_control_valid_locked() &&
           snapshot_.control_hover &&
           snapshot_.hover_confirmed &&
           !snapshot_.execution_active;
}

void FlightStateMachine::update_snapshot(
    const VehicleSnapshot& snapshot)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 防止较旧的快照覆盖新观测。
    if (snapshot.captured_at < snapshot_.captured_at)
        return;

    snapshot_ = snapshot;

    if (phase_ == FlightPhase::FAULT)
        return;

    const auto fault = [this](const std::string& reason)
    {
        phase_ = FlightPhase::FAULT;
        fault_reason_ = reason;
    };

    if (snapshot_.control_status_valid &&
        snapshot_.control_fault) {
        fault("Ctrl reports FAULT");
        return;
    }

    // 启动阶段等待足够观测，不把尚未收到数据当成故障。
    if (phase_ == FlightPhase::UNKNOWN) {
        if (!snapshot_.execution_active &&
            grounded_confirmed_locked()) {
            phase_ = FlightPhase::GROUNDED;
        }
        else if (hovering_confirmed_locked()) {
            phase_ = FlightPhase::HOVERING;
        }

        return;
    }

    if (!basic_observation_valid_locked()) {
        fault("Connection, armed or landed observation lost");
        return;
    }

    switch (phase_) {
    case FlightPhase::GROUNDED:
        // 没有 TAKEOFF_STARTED，却观测到离地，不能冒充任务起飞。
        if (!snapshot_.landed)
            fault("Unexpected airborne observation while GROUNDED");
        break;

    case FlightPhase::HOVERING:
        if (!airborne_control_valid_locked()) {
            fault("Airborne feedback or control lost");
        }
        else if (!snapshot_.control_hover) {
            fault("Ctrl left HOVER without a flight transition");
        }
        break;

    case FlightPhase::MOVING:
        if (!airborne_control_valid_locked())
            fault("Airborne feedback or control lost while MOVING");
        break;

    case FlightPhase::TAKING_OFF:
    case FlightPhase::LANDING:
        // 起飞和降落过程中允许 landed 变化。
        if (!flight_feedback_valid_locked())
            fault("Flight feedback or control lost");
        break;

    case FlightPhase::STOPPING:
        // 已接地时不要求仍然 armed。
        if (!snapshot_.landed &&
            !airborne_control_valid_locked()) {
            fault("Airborne feedback or control lost while STOPPING");
        }
        break;

    case FlightPhase::UNKNOWN:
    case FlightPhase::FAULT:
        break;
    }
}

FlightPhase FlightStateMachine::phase() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_;
}

VehicleSnapshot FlightStateMachine::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto result = snapshot_;

    if (!observation_fresh_locked()) {
        result.connected_valid = false;
        result.armed_valid = false;
        result.landed_valid = false;
        result.position_valid = false;
        result.velocity_valid = false;
        result.attitude_valid = false;
        result.control_status_valid = false;
        result.control_available = false;
        result.hover_confirmed = false;
    }

    return result;
}

std::string FlightStateMachine::fault_reason() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_reason_;
}

AdmissionResult FlightStateMachine::check_action(
    ActionType action) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return check_action_locked(action);
}

AdmissionResult FlightStateMachine::check_action_locked(
    ActionType action) const
{
    // 允许请求停止不代表已具备安全停止条件。
    if (action == ActionType::STOP)
        return {true, "Stop request permitted"};

    if (phase_ == FlightPhase::FAULT)
        return {false, "Vehicle is in FAULT: " + fault_reason_};

    if (phase_ == FlightPhase::UNKNOWN)
        return {false, "Vehicle phase is UNKNOWN"};

    if (!basic_observation_valid_locked())
        return {false, "Basic vehicle observation is invalid or stale"};

    if (snapshot_.execution_active)
        return {false, "Motion execution is active"};

    switch (action) {
    case ActionType::ARM:
    case ActionType::DISARM:
        if (phase_ != FlightPhase::GROUNDED ||
            !grounded_confirmed_locked()) {
            return {false, "Arm/Disarm requires confirmed GROUNDED"};
        }

        if (action == ActionType::ARM && snapshot_.armed)
            return {false, "Vehicle is already armed"};

        if (action == ActionType::DISARM && !snapshot_.armed)
            return {false, "Vehicle is already disarmed"};

        return {true, "Arm/Disarm permitted"};

    case ActionType::TAKEOFF:
        if (phase_ != FlightPhase::GROUNDED ||
            !grounded_confirmed_locked()) {
            return {false, "Takeoff requires confirmed GROUNDED"};
        }

        if (!flight_feedback_valid_locked() ||
            snapshot_.control_tracking) {
            return {false, "Takeoff requires armed and ready control"};
        }

        return {true, "Takeoff permitted"};

    case ActionType::GOTO:
    case ActionType::LAND:
        if (phase_ != FlightPhase::HOVERING ||
            !hovering_confirmed_locked()) {
            return {false, "GoTo/Land requires confirmed HOVERING"};
        }

        return {true, "Airborne action permitted"};

    case ActionType::STOP:
        return {true, "Stop request permitted"};
    }

    return {false, "Unsupported action"};
}

bool FlightStateMachine::stop_can_preempt(ActionType action)
{
    return action != ActionType::STOP;
}

TransitionResult FlightStateMachine::accept_locked(
    FlightPhase previous,
    const std::string& reason)
{
    return {true, previous, phase_, reason};
}

TransitionResult FlightStateMachine::reject_locked(
    FlightPhase previous,
    const std::string& reason) const
{
    return {false, previous, phase_, reason};
}

TransitionResult FlightStateMachine::process_event(
    FlightEvent event)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto previous = phase_;

    if (event == FlightEvent::FAULT_DETECTED) {
        phase_ = FlightPhase::FAULT;

        if (fault_reason_.empty())
            fault_reason_ = "Execution fault reported";

        return accept_locked(previous, fault_reason_);
    }

    if (event == FlightEvent::STATE_RECOVERED) {
        if (phase_ != FlightPhase::FAULT)
            return reject_locked(previous, "Vehicle is not in FAULT");

        // 仅显式请求恢复；还必须具备真实、可靠的恢复条件。
        if (snapshot_.execution_active ||
            !snapshot_.control_status_valid ||
            !snapshot_.control_available ||
            snapshot_.control_fault ||
            snapshot_.control_tracking) {
            return reject_locked(previous, "Control is not ready for recovery");
        }

        if (grounded_confirmed_locked()) {
            phase_ = FlightPhase::GROUNDED;
        }
        else if (hovering_confirmed_locked()) {
            phase_ = FlightPhase::HOVERING;
        }
        else {
            return reject_locked(previous, "Recovery conditions are not met");
        }

        fault_reason_.clear();
        return accept_locked(previous, "Explicit recovery confirmed");
    }

    if (phase_ == FlightPhase::FAULT) {
        return reject_locked(
            previous, "FAULT requires explicit STATE_RECOVERED");
    }

    switch (event) {
    case FlightEvent::SYNC_GROUNDED:
        if (phase_ == FlightPhase::UNKNOWN &&
            !snapshot_.execution_active &&
            grounded_confirmed_locked()) {
            phase_ = FlightPhase::GROUNDED;
            return accept_locked(previous, "Synchronized GROUNDED");
        }
        break;

    case FlightEvent::SYNC_AIRBORNE:
        if (phase_ == FlightPhase::UNKNOWN &&
            hovering_confirmed_locked()) {
            phase_ = FlightPhase::HOVERING;
            return accept_locked(previous, "Synchronized HOVERING");
        }
        break;

    case FlightEvent::ARM_CONFIRMED:
    case FlightEvent::DISARM_CONFIRMED:
        if (phase_ == FlightPhase::GROUNDED &&
            grounded_confirmed_locked() &&
            !snapshot_.execution_active &&
            snapshot_.armed ==
                (event == FlightEvent::ARM_CONFIRMED)) {
            return accept_locked(previous, "Actual armed state confirmed");
        }
        break;

    case FlightEvent::TAKEOFF_STARTED:
    case FlightEvent::GOTO_STARTED:
    case FlightEvent::LAND_STARTED: {
        const auto action =
            event == FlightEvent::TAKEOFF_STARTED
                ? ActionType::TAKEOFF
                : event == FlightEvent::GOTO_STARTED
                    ? ActionType::GOTO
                    : ActionType::LAND;

        const auto admission = check_action_locked(action);

        if (!admission.accepted)
            return reject_locked(previous, admission.reason);

        phase_ =
            event == FlightEvent::TAKEOFF_STARTED
                ? FlightPhase::TAKING_OFF
                : event == FlightEvent::GOTO_STARTED
                    ? FlightPhase::MOVING
                    : FlightPhase::LANDING;

        return accept_locked(previous, "Flight action started");
    }

    case FlightEvent::TAKEOFF_COMPLETED:
    case FlightEvent::GOTO_COMPLETED: {
        const auto expected =
            event == FlightEvent::TAKEOFF_COMPLETED
                ? FlightPhase::TAKING_OFF
                : FlightPhase::MOVING;

        // 调用方必须先收到对应任务的正常完成事件。
        // 此处再检查真实状态，不能单凭 Ctrl HOVER 完成任务。
        if (phase_ == expected && hovering_confirmed_locked()) {
            phase_ = FlightPhase::HOVERING;
            return accept_locked(previous, "Flight action completed");
        }
        break;
    }

    case FlightEvent::LAND_COMPLETED:
        if (phase_ == FlightPhase::LANDING &&
            !snapshot_.execution_active &&
            grounded_confirmed_locked()) {
            phase_ = FlightPhase::GROUNDED;
            return accept_locked(previous, "Landing confirmed");
        }
        break;

    case FlightEvent::STOP_STARTED:
        phase_ = FlightPhase::STOPPING;
        return accept_locked(previous, "Stop requested");

    case FlightEvent::STOP_HOVER_CONFIRMED:
        if (phase_ == FlightPhase::STOPPING &&
            hovering_confirmed_locked()) {
            phase_ = FlightPhase::HOVERING;
            return accept_locked(previous, "Stopped in confirmed HOVERING");
        }
        break;

    case FlightEvent::STOP_GROUNDED_CONFIRMED:
        if (phase_ == FlightPhase::STOPPING &&
            !snapshot_.execution_active &&
            grounded_confirmed_locked()) {
            phase_ = FlightPhase::GROUNDED;
            return accept_locked(previous, "Stopped on ground");
        }
        break;

    case FlightEvent::FAULT_DETECTED:
    case FlightEvent::STATE_RECOVERED:
        break;
    }

    return reject_locked(
        previous, "Illegal transition or unconfirmed observation");
}

}  // namespace drone_node::exec