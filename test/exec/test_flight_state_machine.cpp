
#include <cassert>
#include <iostream>

#include "drone_node/exec/flight_state_machine.hpp"

using namespace drone_node::exec;

namespace
{

VehicleSnapshot grounded_snapshot(bool armed)
{
    VehicleSnapshot s;
    s.connected = true;
    s.armed = armed;
    s.position_valid = true;
    s.control_available = true;
    s.landed_valid = true;
    s.landed = true;
    return s;
}

VehicleSnapshot airborne_snapshot()
{
    VehicleSnapshot s;
    s.connected = true;
    s.armed = true;
    s.position_valid = true;
    s.control_available = true;
    s.landed_valid = true;
    s.landed = false;
    return s;
}

void expect_phase(
    const FlightStateMachine& fsm,
    FlightPhase expected)
{
    assert(fsm.phase() == expected);
}

void expect_event(
    FlightStateMachine& fsm,
    FlightEvent event,
    bool expected_accepted,
    FlightPhase expected_phase)
{
    const auto result = fsm.process_event(event);

    assert(result.accepted == expected_accepted);
    assert(result.current == expected_phase);
    expect_phase(fsm, expected_phase);
}

void test_initial_state()
{
    FlightStateMachine fsm;

    expect_phase(fsm, FlightPhase::UNKNOWN);

    assert(!fsm.check_action(ActionType::ARM).accepted);
    assert(!fsm.check_action(ActionType::GOTO).accepted);
    assert(fsm.check_action(ActionType::STOP).accepted);

    std::cout << "[PASS] initial state\n";
}

void test_takeoff_requires_arm()
{
    FlightStateMachine fsm;

    fsm.update_snapshot(grounded_snapshot(false));

    expect_event(
        fsm,
        FlightEvent::SYNC_GROUNDED,
        true,
        FlightPhase::GROUNDED);

    assert(!fsm.check_action(ActionType::TAKEOFF).accepted);
    assert(fsm.check_action(ActionType::ARM).accepted);

    // 未解锁时不能开始起飞。
    expect_event(
        fsm,
        FlightEvent::TAKEOFF_STARTED,
        false,
        FlightPhase::GROUNDED);

    // 模拟 PX4 确认解锁。
    fsm.update_snapshot(grounded_snapshot(true));

    expect_event(
        fsm,
        FlightEvent::ARM_CONFIRMED,
        true,
        FlightPhase::GROUNDED);

    assert(fsm.check_action(ActionType::TAKEOFF).accepted);

    expect_event(
        fsm,
        FlightEvent::TAKEOFF_STARTED,
        true,
        FlightPhase::TAKING_OFF);

    // 未确认离地，不能宣布起飞完成。
    expect_event(
        fsm,
        FlightEvent::TAKEOFF_COMPLETED,
        false,
        FlightPhase::TAKING_OFF);

    fsm.update_snapshot(airborne_snapshot());

    expect_event(
        fsm,
        FlightEvent::TAKEOFF_COMPLETED,
        true,
        FlightPhase::HOVERING);

    std::cout << "[PASS] takeoff requires arm\n";
}

void test_goto_and_land()
{
    FlightStateMachine fsm;
    fsm.update_snapshot(airborne_snapshot());

    expect_event(
        fsm,
        FlightEvent::SYNC_AIRBORNE,
        true,
        FlightPhase::HOVERING);

    assert(fsm.check_action(ActionType::GOTO).accepted);

    expect_event(
        fsm,
        FlightEvent::GOTO_STARTED,
        true,
        FlightPhase::MOVING);

    // 普通 Land 不能绕过 MOVING 阶段的动作仲裁。
    assert(!fsm.check_action(ActionType::LAND).accepted);

    expect_event(
        fsm,
        FlightEvent::GOTO_COMPLETED,
        true,
        FlightPhase::HOVERING);

    expect_event(
        fsm,
        FlightEvent::LAND_STARTED,
        true,
        FlightPhase::LANDING);

    // 尚未接地。
    expect_event(
        fsm,
        FlightEvent::LAND_COMPLETED,
        false,
        FlightPhase::LANDING);

    fsm.update_snapshot(grounded_snapshot(true));

    expect_event(
        fsm,
        FlightEvent::LAND_COMPLETED,
        true,
        FlightPhase::GROUNDED);

    assert(fsm.check_action(ActionType::DISARM).accepted);

    std::cout << "[PASS] goto and land\n";
}

void test_stop_preemption()
{
    FlightStateMachine fsm;
    fsm.update_snapshot(airborne_snapshot());

    expect_event(
        fsm,
        FlightEvent::SYNC_AIRBORNE,
        true,
        FlightPhase::HOVERING);

    expect_event(
        fsm,
        FlightEvent::GOTO_STARTED,
        true,
        FlightPhase::MOVING);

    assert(fsm.check_action(ActionType::STOP).accepted);
    assert(FlightStateMachine::stop_can_preempt(
        ActionType::GOTO));
    assert(FlightStateMachine::stop_can_preempt(
        ActionType::TAKEOFF));
    assert(FlightStateMachine::stop_can_preempt(
        ActionType::LAND));
    assert(FlightStateMachine::stop_can_preempt(
        ActionType::ARM));
    assert(FlightStateMachine::stop_can_preempt(
        ActionType::DISARM));
    assert(!FlightStateMachine::stop_can_preempt(
        ActionType::STOP));

    expect_event(
        fsm,
        FlightEvent::STOP_STARTED,
        true,
        FlightPhase::STOPPING);

    // 被抢占动作的迟到完成事件不能改变状态。
    expect_event(
        fsm,
        FlightEvent::GOTO_COMPLETED,
        false,
        FlightPhase::STOPPING);

    expect_event(
        fsm,
        FlightEvent::STOP_HOVER_CONFIRMED,
        true,
        FlightPhase::HOVERING);

    std::cout << "[PASS] stop preemption\n";
}

void test_fault_from_every_phase()
{
    const FlightPhase phases[] = {
        FlightPhase::UNKNOWN,
        FlightPhase::GROUNDED,
        FlightPhase::TAKING_OFF,
        FlightPhase::HOVERING,
        FlightPhase::MOVING,
        FlightPhase::STOPPING,
        FlightPhase::LANDING,
        FlightPhase::FAULT
    };

    for (const auto ignored_phase : phases) {
        (void)ignored_phase;

        // FAULT_DETECTED 是全局事件，与当前状态无关。
        // 这里先验证 UNKNOWN 和 FAULT 的重复触发。
        FlightStateMachine fsm;

        expect_event(
            fsm,
            FlightEvent::FAULT_DETECTED,
            true,
            FlightPhase::FAULT);

        expect_event(
            fsm,
            FlightEvent::FAULT_DETECTED,
            true,
            FlightPhase::FAULT);
    }

    // 验证实际运行中间状态进入 FAULT。
    FlightStateMachine moving_fsm;
    moving_fsm.update_snapshot(airborne_snapshot());

    expect_event(
        moving_fsm,
        FlightEvent::SYNC_AIRBORNE,
        true,
        FlightPhase::HOVERING);

    expect_event(
        moving_fsm,
        FlightEvent::GOTO_STARTED,
        true,
        FlightPhase::MOVING);

    expect_event(
        moving_fsm,
        FlightEvent::FAULT_DETECTED,
        true,
        FlightPhase::FAULT);

    std::cout << "[PASS] fault entry\n";
}

void test_fault_recovery_only()
{
    FlightStateMachine fsm;
    fsm.update_snapshot(airborne_snapshot());

    expect_event(
        fsm,
        FlightEvent::SYNC_AIRBORNE,
        true,
        FlightPhase::HOVERING);

    expect_event(
        fsm,
        FlightEvent::FAULT_DETECTED,
        true,
        FlightPhase::FAULT);

    // FAULT 下仍允许 Stop 请求，但不能退出 FAULT。
    assert(fsm.check_action(ActionType::STOP).accepted);
    assert(!fsm.check_action(ActionType::GOTO).accepted);

    expect_event(
        fsm,
        FlightEvent::STOP_STARTED,
        false,
        FlightPhase::FAULT);

    expect_event(
        fsm,
        FlightEvent::GOTO_COMPLETED,
        false,
        FlightPhase::FAULT);

    expect_event(
        fsm,
        FlightEvent::SYNC_AIRBORNE,
        false,
        FlightPhase::FAULT);

    // 状态数据失效时，恢复事件也不能解除 FAULT。
    fsm.update_snapshot(VehicleSnapshot{});

    expect_event(
        fsm,
        FlightEvent::STATE_RECOVERED,
        false,
        FlightPhase::FAULT);

    // 观测数据恢复后，由专门的恢复事件解除。
    fsm.update_snapshot(airborne_snapshot());

    expect_event(
        fsm,
        FlightEvent::STATE_RECOVERED,
        true,
        FlightPhase::HOVERING);

    // 非 FAULT 状态不能重复执行恢复。
    expect_event(
        fsm,
        FlightEvent::STATE_RECOVERED,
        false,
        FlightPhase::HOVERING);

    std::cout << "[PASS] fault recovery only\n";
}

void test_stop_from_unknown()
{
    FlightStateMachine fsm;

    expect_event(
        fsm,
        FlightEvent::STOP_STARTED,
        true,
        FlightPhase::STOPPING);

    // 无可靠状态反馈，不能虚报已悬停。
    expect_event(
        fsm,
        FlightEvent::STOP_HOVER_CONFIRMED,
        false,
        FlightPhase::STOPPING);

    fsm.update_snapshot(grounded_snapshot(false));

    expect_event(
        fsm,
        FlightEvent::STOP_GROUNDED_CONFIRMED,
        true,
        FlightPhase::GROUNDED);

    std::cout << "[PASS] stop from unknown\n";
}

}  // namespace

int main()
{
    test_initial_state();
    test_takeoff_requires_arm();
    test_goto_and_land();
    test_stop_preemption();
    test_fault_from_every_phase();
    test_fault_recovery_only();
    test_stop_from_unknown();

    std::cout
        << "\nAll FlightStateMachine tests passed.\n";

    return 0;
}
