
#pragma once

#include <mutex>
#include <string>

#include "drone_node/exec/action_types.hpp"

namespace drone_node::exec
{

class FlightStateMachine
{
public:
    FlightStateMachine() = default;

    // 更新观测数据，不自动解除 FAULT。
    void update_snapshot(const VehicleSnapshot& snapshot);

    // 查询当前状态。
    FlightPhase phase() const;
    VehicleSnapshot snapshot() const;

    // 判断动作在当前飞行状态下是否合法。
    // 不负责普通动作互斥，互斥由 ActionManager 处理。
    AdmissionResult check_action(ActionType action) const;

    // 处理飞行状态事件。
    TransitionResult process_event(FlightEvent event);

    // Stop 对所有普通动作具有抢占权限。
    static bool stop_can_preempt(ActionType active_action);

private:
    AdmissionResult check_action_locked(ActionType action) const;

    TransitionResult accept_locked(
        FlightPhase previous,
        const std::string& reason);

    TransitionResult reject_locked(
        FlightPhase previous,
        const std::string& reason) const;

    // 判断观测数据是否足以确认飞机在地面。
    bool grounded_confirmed_locked() const;

    // 判断观测数据是否足以确认飞机在空中且可控。
    bool airborne_control_valid_locked() const;

    mutable std::mutex mutex_;

    FlightPhase phase_{FlightPhase::UNKNOWN};
    VehicleSnapshot snapshot_{};
};

}  // namespace drone_node::exec
