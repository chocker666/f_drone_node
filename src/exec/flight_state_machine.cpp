
#include "drone_node/exec/flight_state_machine.hpp"

namespace drone_node::exec
{

void FlightStateMachine::update_snapshot(
    const VehicleSnapshot& snapshot)
{
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = snapshot;
}

FlightPhase FlightStateMachine::phase() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_;
}

VehicleSnapshot FlightStateMachine::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

bool FlightStateMachine::grounded_confirmed_locked() const
{
    return snapshot_.connected &&
           snapshot_.landed_valid &&
           snapshot_.landed;
}

bool FlightStateMachine::airborne_control_valid_locked() const
{
    return snapshot_.connected &&
           snapshot_.armed &&
           snapshot_.position_valid &&
           snapshot_.control_available &&
           snapshot_.landed_valid &&
           !snapshot_.landed;
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
    // Stop 是最高优先级软急停。
    // 即使状态 UNKNOWN 或 FAULT，也允许发出停止请求。
    // 是否能够实际停止，由后续执行模块确认。
    if (action == ActionType::STOP) {
        return {true, "Stop request permitted"};
    }

    if (phase_ == FlightPhase::FAULT) {
        return {false, "Vehicle is in FAULT"};
    }

    if (phase_ == FlightPhase::UNKNOWN) {
        return {false, "Vehicle phase is UNKNOWN"};
    }

    if (!snapshot_.connected) {
        return {false, "PX4 is not connected"};
    }

    switch (action) {
    case ActionType::ARM:
        if (phase_ != FlightPhase::GROUNDED) {
            return {false, "Arm requires GROUNDED"};
        }
        if (!grounded_confirmed_locked()) {
            return {false, "Ground contact is not confirmed"};
        }
        if (snapshot_.armed) {
            return {false, "Vehicle is already armed"};
        }
        return {true, "Arm permitted"};

    case ActionType::DISARM:
        if (phase_ != FlightPhase::GROUNDED) {
            return {false, "Disarm requires GROUNDED"};
        }
        if (!grounded_confirmed_locked()) {
            return {false, "Ground contact is not confirmed"};
        }
        if (!snapshot_.armed) {
            return {false, "Vehicle is already disarmed"};
        }
        return {true, "Disarm permitted"};

    case ActionType::TAKEOFF:
        if (phase_ != FlightPhase::GROUNDED) {
            return {false, "Takeoff requires GROUNDED"};
        }
        if (!grounded_confirmed_locked()) {
            return {false, "Ground contact is not confirmed"};
        }
        if (!snapshot_.armed) {
            return {false, "Takeoff requires prior Arm"};
        }
        if (!snapshot_.position_valid ||
            !snapshot_.control_available) {
            return {false, "Flight control is not ready"};
        }
        return {true, "Takeoff permitted"};

    case ActionType::GOTO:
        if (phase_ != FlightPhase::HOVERING) {
            return {false, "GoTo requires HOVERING"};
        }
        if (!airborne_control_valid_locked()) {
            return {false, "Airborne control is not valid"};
        }
        return {true, "GoTo permitted"};

    case ActionType::LAND:
        // 第一版只允许从 HOVERING 发起普通降落。
        // MOVING 中需要先 Stop，避免普通 Land 绕过仲裁。
        if (phase_ != FlightPhase::HOVERING) {
            return {false, "Land requires HOVERING"};
        }
        if (!airborne_control_valid_locked()) {
            return {false, "Airborne control is not valid"};
        }
        return {true, "Land permitted"};

    case ActionType::STOP:
        return {true, "Stop request permitted"};
    }

    return {false, "Unsupported action"};
}

bool FlightStateMachine::stop_can_preempt(
    ActionType active_action)
{
    // 普通动作均可被 Stop 抢占。
    // 重复 Stop 由 ActionManager 拒绝或合并。
    return active_action != ActionType::STOP;
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

    const FlightPhase previous = phase_;

    // FAULT_DETECTED 可从任意状态触发。
    if (event == FlightEvent::FAULT_DETECTED) {
        phase_ = FlightPhase::FAULT;
        return accept_locked(previous, "Fault detected");
    }

    // STATE_RECOVERED 是唯一允许退出 FAULT 的事件。
    if (event == FlightEvent::STATE_RECOVERED) {
        if (phase_ != FlightPhase::FAULT) {
            return reject_locked(
                previous, "Vehicle is not in FAULT");
        }

        if (grounded_confirmed_locked()) {
            phase_ = FlightPhase::GROUNDED;
            return accept_locked(
                previous, "Recovered to GROUNDED");
        }

        if (airborne_control_valid_locked()) {
            phase_ = FlightPhase::HOVERING;
            return accept_locked(
                previous, "Recovered to HOVERING");
        }

        // 恢复事件到达，但尚不能可靠确认飞机状态。
        // 保持 FAULT，不允许退出。
        return reject_locked(
            previous, "Recovery conditions are not met");
    }

    // FAULT 状态禁止任何普通事件修改主状态。
    if (phase_ == FlightPhase::FAULT) {
        return reject_locked(
            previous,
            "FAULT can only be cleared by STATE_RECOVERED");
    }

    switch (event) {
    case FlightEvent::SYNC_GROUNDED:
        if (phase_ != FlightPhase::UNKNOWN) {
            break;
        }
        if (!grounded_confirmed_locked()) {
            return reject_locked(
                previous, "Grounded state not confirmed");
        }
        phase_ = FlightPhase::GROUNDED;
        return accept_locked(previous, "Synchronized grounded");

    case FlightEvent::SYNC_AIRBORNE:
        if (phase_ != FlightPhase::UNKNOWN) {
            break;
        }
        if (!airborne_control_valid_locked()) {
            return reject_locked(
                previous, "Airborne state not confirmed");
        }
        phase_ = FlightPhase::HOVERING;
        return accept_locked(previous, "Synchronized airborne");

    case FlightEvent::ARM_CONFIRMED:
        if (phase_ != FlightPhase::GROUNDED ||
            !grounded_confirmed_locked() ||
            !snapshot_.armed) {
            break;
        }
        return accept_locked(previous, "Arm confirmed");

    case FlightEvent::DISARM_CONFIRMED:
        if (phase_ != FlightPhase::GROUNDED ||
            !grounded_confirmed_locked() ||
            snapshot_.armed) {
            break;
        }
        return accept_locked(previous, "Disarm confirmed");

    case FlightEvent::TAKEOFF_STARTED:
        if (!check_action_locked(ActionType::TAKEOFF).accepted) {
            break;
        }
        phase_ = FlightPhase::TAKING_OFF;
        return accept_locked(previous, "Takeoff started");

    case FlightEvent::TAKEOFF_COMPLETED:
        if (phase_ != FlightPhase::TAKING_OFF ||
            !airborne_control_valid_locked()) {
            break;
        }
        // 目标高度及稳定性应由后续 Monitor 验证。
        phase_ = FlightPhase::HOVERING;
        return accept_locked(previous, "Takeoff completed");

    case FlightEvent::GOTO_STARTED:
        if (!check_action_locked(ActionType::GOTO).accepted) {
            break;
        }
        phase_ = FlightPhase::MOVING;
        return accept_locked(previous, "GoTo started");

    case FlightEvent::GOTO_COMPLETED:
        if (phase_ != FlightPhase::MOVING ||
            !airborne_control_valid_locked()) {
            break;
        }
        // 目标位置、速度和姿态容差由 Monitor 验证。
        phase_ = FlightPhase::HOVERING;
        return accept_locked(previous, "GoTo completed");

    case FlightEvent::LAND_STARTED:
        if (!check_action_locked(ActionType::LAND).accepted) {
            break;
        }
        phase_ = FlightPhase::LANDING;
        return accept_locked(previous, "Land started");

    case FlightEvent::LAND_COMPLETED:
        if (phase_ != FlightPhase::LANDING ||
            !grounded_confirmed_locked()) {
            break;
        }
        phase_ = FlightPhase::GROUNDED;
        return accept_locked(previous, "Land completed");

    case FlightEvent::STOP_STARTED:
        // STOP 可在任何非 FAULT 状态提出。
        // UNKNOWN 下允许请求，但不假设已建立悬停。
        if (phase_ == FlightPhase::STOPPING) {
            return accept_locked(
                previous, "Stop already in progress");
        }
        phase_ = FlightPhase::STOPPING;
        return accept_locked(previous, "Stop started");

    case FlightEvent::STOP_HOVER_CONFIRMED:
        if (phase_ != FlightPhase::STOPPING ||
            !airborne_control_valid_locked()) {
            break;
        }
        phase_ = FlightPhase::HOVERING;
        return accept_locked(previous, "Stop hover confirmed");

    case FlightEvent::STOP_GROUNDED_CONFIRMED:
        if (phase_ != FlightPhase::STOPPING ||
            !grounded_confirmed_locked()) {
            break;
        }
        phase_ = FlightPhase::GROUNDED;
        return accept_locked(previous, "Stop grounded confirmed");

    case FlightEvent::FAULT_DETECTED:
    case FlightEvent::STATE_RECOVERED:
        // 已在 switch 之前处理。
        break;
    }

    return reject_locked(
        previous, "Illegal transition or unmet conditions");
}

}  // namespace drone_node::exec
