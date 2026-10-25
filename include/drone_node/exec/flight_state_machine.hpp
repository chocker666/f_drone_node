#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "drone_node/exec/action_types.hpp"

namespace drone_node::exec
{

class FlightStateMachine
{
public:
    FlightStateMachine() = default;

    // 更新真实观测，进行启动同步及故障监督。
    // 不根据观测自动完成正在执行的任务，不自动解除 FAULT。
    void update_snapshot(const VehicleSnapshot& snapshot);

    FlightPhase phase() const;
    VehicleSnapshot snapshot() const;
    std::string fault_reason() const;

    AdmissionResult check_action(ActionType action) const;
    TransitionResult process_event(FlightEvent event);

    static bool stop_can_preempt(ActionType active_action);

private:
    bool observation_fresh_locked() const;
    bool basic_observation_valid_locked() const;
    bool grounded_confirmed_locked() const;
    bool flight_feedback_valid_locked() const;
    bool airborne_control_valid_locked() const;
    bool hovering_confirmed_locked() const;

    AdmissionResult check_action_locked(ActionType action) const;

    TransitionResult accept_locked(
        FlightPhase previous,
        const std::string& reason);

    TransitionResult reject_locked(
        FlightPhase previous,
        const std::string& reason) const;

    mutable std::mutex mutex_;

    FlightPhase phase_{FlightPhase::UNKNOWN};
    VehicleSnapshot snapshot_{};
    std::string fault_reason_;

    // 防止快照更新停止后，上层继续使用旧快照进行准入。
    const std::chrono::milliseconds snapshot_timeout_{250};
};

}  // namespace drone_node::exec