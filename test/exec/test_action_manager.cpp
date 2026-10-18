
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "drone_node/exec/action_manager.hpp"

using namespace drone_node::exec;
using namespace std::chrono_literals;

namespace
{

VehicleSnapshot grounded(bool armed = false)
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

VehicleSnapshot airborne()
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

std::shared_ptr<FlightStateMachine> make_airborne_fsm()
{
    auto fsm = std::make_shared<FlightStateMachine>();
    fsm->update_snapshot(airborne());

    assert(fsm->process_event(
        FlightEvent::SYNC_AIRBORNE).accepted);

    return fsm;
}

std::shared_ptr<FlightStateMachine> make_grounded_fsm(
    bool armed = false)
{
    auto fsm = std::make_shared<FlightStateMachine>();
    fsm->update_snapshot(grounded(armed));

    assert(fsm->process_event(
        FlightEvent::SYNC_GROUNDED).accepted);

    return fsm;
}

void expect_state(
    ActionManager& manager,
    ActionId id,
    ActionState expected)
{
    const auto record = manager.get(id);

    assert(record.has_value());
    assert(record->state == expected);
}

std::size_t count_terminal_events(
    const std::vector<ActionEvent>& events,
    ActionId id)
{
    std::size_t count = 0;

    for (const auto& event : events) {
        if (event.id != id) {
            continue;
        }

        if (event.kind == ActionEventKind::SUCCEEDED ||
            event.kind == ActionEventKind::CANCELED ||
            event.kind == ActionEventKind::FAILED) {
            ++count;
        }
    }

    return count;
}

void test_admission_and_lifecycle()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto goal = manager.submit(ActionType::GOTO);
    assert(goal.accepted);

    const auto rejected = manager.submit(ActionType::LAND);
    assert(!rejected.accepted);

    expect_state(
        manager, goal.id, ActionState::PENDING);

    assert(manager.start(goal.id));

    expect_state(
        manager, goal.id, ActionState::EXECUTING);

    assert(fsm->phase() == FlightPhase::MOVING);

    assert(manager.complete(goal.id));

    expect_state(
        manager, goal.id, ActionState::SUCCEEDED);

    assert(fsm->phase() == FlightPhase::HOVERING);

    assert(!manager.complete(goal.id));
    assert(!manager.active_action().has_value());

    const auto events = manager.drain_events();

    assert(count_terminal_events(events, goal.id) == 1);

    std::cout << "[PASS] admission and lifecycle\n";
}

void test_takeoff_requires_arm()
{
    auto fsm = make_grounded_fsm(false);
    ActionManager manager(fsm);

    assert(!manager.submit(ActionType::TAKEOFF).accepted);

    const auto arm = manager.submit(ActionType::ARM);
    assert(arm.accepted);
    assert(manager.start(arm.id));

    // 尚未收到解锁确认，不能成功结束。
    assert(!manager.complete(arm.id));

    fsm->update_snapshot(grounded(true));

    assert(manager.complete(arm.id));

    const auto takeoff =
        manager.submit(ActionType::TAKEOFF);

    assert(takeoff.accepted);
    assert(manager.start(takeoff.id));

    assert(fsm->phase() == FlightPhase::TAKING_OFF);

    fsm->update_snapshot(airborne());

    assert(manager.complete(takeoff.id));
    assert(fsm->phase() == FlightPhase::HOVERING);

    std::cout << "[PASS] takeoff requires arm\n";
}

void test_stop_preemption()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto go = manager.submit(ActionType::GOTO);
    assert(go.accepted);
    assert(manager.start(go.id));

    const auto stop = manager.submit(ActionType::STOP);
    assert(stop.accepted);

    expect_state(
        manager, go.id, ActionState::CANCELING);

    expect_state(
        manager, stop.id, ActionState::PENDING);

    assert(manager.active_action().value() == stop.id);

    // 新普通动作不能在抢占期间插入。
    assert(!manager.submit(ActionType::GOTO).accepted);

    // 不接受重复 Stop。
    assert(!manager.submit(ActionType::STOP).accepted);

    assert(manager.start(stop.id));
    assert(fsm->phase() == FlightPhase::STOPPING);

    // 旧 GoTo 的迟到完成事件必须被忽略。
    assert(!manager.complete(go.id));

    assert(manager.complete(stop.id));

    expect_state(
        manager, go.id, ActionState::CANCELED);

    expect_state(
        manager, stop.id, ActionState::SUCCEEDED);

    assert(fsm->phase() == FlightPhase::HOVERING);

    assert(!manager.active_action().has_value());

    const auto events = manager.drain_events();

    assert(count_terminal_events(events, go.id) == 1);
    assert(count_terminal_events(events, stop.id) == 1);

    std::cout << "[PASS] stop preemption\n";
}

void test_cancel_waits_for_confirmation()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto go = manager.submit(ActionType::GOTO);
    assert(go.accepted);
    assert(manager.start(go.id));

    assert(manager.cancel(go.id));

    expect_state(
        manager, go.id, ActionState::CANCELING);

    assert(fsm->phase() == FlightPhase::STOPPING);

    // Cancel 接受后仍然占用动作执行权。
    assert(manager.active_action().value() == go.id);

    assert(!manager.submit(ActionType::LAND).accepted);

    // 模拟执行器确认已经停止并悬停。
    assert(manager.complete(go.id));

    expect_state(
        manager, go.id, ActionState::CANCELED);

    assert(fsm->phase() == FlightPhase::HOVERING);

    std::cout << "[PASS] cancel confirmation\n";
}

void test_stop_takes_over_cancel()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto go = manager.submit(ActionType::GOTO);
    assert(go.accepted);
    assert(manager.start(go.id));

    assert(manager.cancel(go.id));

    const auto stop = manager.submit(ActionType::STOP);
    assert(stop.accepted);
    assert(manager.start(stop.id));

    assert(!manager.complete(go.id));

    assert(manager.complete(stop.id));

    expect_state(
        manager, go.id, ActionState::CANCELED);

    expect_state(
        manager, stop.id, ActionState::SUCCEEDED);

    std::cout << "[PASS] stop takes over cancel\n";
}

void test_stop_cannot_be_canceled()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto stop = manager.submit(ActionType::STOP);
    assert(stop.accepted);
    assert(manager.start(stop.id));

    assert(!manager.cancel(stop.id));

    assert(manager.complete(stop.id));

    expect_state(
        manager, stop.id, ActionState::SUCCEEDED);

    std::cout << "[PASS] stop cannot be canceled\n";
}

void test_fault_aborts_actions()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto go = manager.submit(ActionType::GOTO);
    assert(go.accepted);
    assert(manager.start(go.id));

    manager.report_fault("Position estimate lost");

    assert(fsm->phase() == FlightPhase::FAULT);

    expect_state(
        manager, go.id, ActionState::FAILED);

    assert(!manager.complete(go.id));
    assert(!manager.submit(ActionType::GOTO).accepted);

    // FAULT 下 Stop 仍可作为紧急请求接受，
    // 但不改变 FAULT 状态。
    const auto stop = manager.submit(ActionType::STOP);
    assert(stop.accepted);
    assert(manager.start(stop.id));

    assert(fsm->phase() == FlightPhase::FAULT);

    // FAULT 下不能虚报 Stop 已成功悬停。
    assert(!manager.complete(stop.id));

    manager.report_fault("Stop unavailable");

    expect_state(
        manager, stop.id, ActionState::FAILED);

    assert(fsm->phase() == FlightPhase::FAULT);

    fsm->update_snapshot(airborne());

    assert(manager.recover_flight_state());

    assert(fsm->phase() == FlightPhase::HOVERING);

    std::cout << "[PASS] fault and recovery\n";
}

void test_failure_and_timeout()
{
    {
        auto fsm = make_airborne_fsm();
        ActionManager manager(fsm);

        const auto go = manager.submit(ActionType::GOTO);
        assert(go.accepted);
        assert(manager.start(go.id));

        assert(manager.fail(go.id, "Controller fault"));

        expect_state(
            manager, go.id, ActionState::FAILED);

        assert(fsm->phase() == FlightPhase::FAULT);
    }

    {
        auto fsm = make_grounded_fsm(false);
        ActionManager manager(fsm);

        const auto arm = manager.submit(
            ActionType::ARM, 1ms);

        assert(arm.accepted);
        assert(manager.start(arm.id));

        std::this_thread::sleep_for(5ms);
        manager.tick();

        expect_state(
            manager, arm.id, ActionState::FAILED);

        // Arm 超时不直接推断飞机处于 FAULT。
        assert(fsm->phase() == FlightPhase::GROUNDED);
    }

    std::cout << "[PASS] failure and timeout\n";
}

void test_late_events_do_not_affect_new_action()
{
    auto fsm = make_airborne_fsm();
    ActionManager manager(fsm);

    const auto first = manager.submit(ActionType::GOTO);
    assert(first.accepted);
    assert(manager.start(first.id));
    assert(manager.complete(first.id));

    const auto second = manager.submit(ActionType::GOTO);
    assert(second.accepted);
    assert(manager.start(second.id));

    assert(!manager.complete(first.id));
    assert(!manager.fail(first.id, "Late failure"));

    expect_state(
        manager, second.id, ActionState::EXECUTING);

    assert(fsm->phase() == FlightPhase::MOVING);

    assert(manager.complete(second.id));

    std::cout << "[PASS] stale events\n";
}

}  // namespace

int main()
{
    test_admission_and_lifecycle();
    test_takeoff_requires_arm();
    test_stop_preemption();
    test_cancel_waits_for_confirmation();
    test_stop_takes_over_cancel();
    test_stop_cannot_be_canceled();
    test_fault_aborts_actions();
    test_failure_and_timeout();
    test_late_events_do_not_affect_new_action();

    std::cout
        << "\nAll ActionManager tests passed.\n";

    return 0;
}
