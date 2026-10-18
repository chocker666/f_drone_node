
#include "drone_node/exec/action_manager.hpp"

#include <stdexcept>
#include <utility>

namespace drone_node::exec
{

ActionManager::ActionManager(
    std::shared_ptr<FlightStateMachine> flight_state_machine)
    : fsm_(std::move(flight_state_machine))
{
    if (!fsm_) {
        throw std::invalid_argument(
            "ActionManager requires FlightStateMachine");
    }
}

bool ActionManager::terminal(ActionState state)
{
    return state == ActionState::SUCCEEDED ||
           state == ActionState::CANCELED ||
           state == ActionState::FAILED;
}

bool ActionManager::motion_action(ActionType type)
{
    return type == ActionType::TAKEOFF ||
           type == ActionType::LAND ||
           type == ActionType::GOTO ||
           type == ActionType::STOP;
}

void ActionManager::emit_locked(
    const ActionRecord& record,
    ActionEventKind kind,
    const std::string& reason)
{
    events_.push_back({
        record.id,
        record.type,
        kind,
        record.state,
        reason
    });
}

void ActionManager::finish_locked(
    ActionRecord& record,
    ActionState state,
    const std::string& reason)
{
    if (terminal(record.state)) {
        return;
    }

    record.state = state;
    record.reason = reason;

    ActionEventKind kind = ActionEventKind::FAILED;

    if (state == ActionState::SUCCEEDED) {
        kind = ActionEventKind::SUCCEEDED;
    } else if (state == ActionState::CANCELED) {
        kind = ActionEventKind::CANCELED;
    }

    emit_locked(record, kind, reason);

    if (active_id_ && *active_id_ == record.id) {
        active_id_.reset();
    }
    if (preempted_id_ && *preempted_id_ == record.id) {
        preempted_id_.reset();
    }
    if (canceling_id_ && *canceling_id_ == record.id) {
        canceling_id_.reset();
    }
}

SubmitResult ActionManager::submit(
    ActionType type,
    std::chrono::milliseconds timeout)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (timeout.count() <= 0) {
        return {false, 0, "Timeout must be positive"};
    }

    // 先进行飞行语义状态准入检查。
    const auto admission = fsm_->check_action(type);

    if (!admission.accepted) {
        return {false, 0, admission.reason};
    }

    if (type != ActionType::STOP && active_id_) {
        return {false, 0, "Another action is active"};
    }

    if (type != ActionType::STOP && canceling_id_) {
        return {false, 0, "Cancellation is in progress"};
    }

    if (type == ActionType::STOP && active_id_) {
        const auto& active = records_.at(*active_id_);

        if (!FlightStateMachine::stop_can_preempt(
                active.type)) {
            return {false, 0, "Stop is already active"};
        }
    }

    ActionRecord record;
    record.id = next_id_++;
    record.type = type;
    record.state = ActionState::PENDING;
    record.created_at = Clock::now();
    record.timeout = timeout;

    const ActionId new_id = record.id;

    records_.emplace(new_id, record);

    // Stop 抢占所有普通动作。
    if (type == ActionType::STOP) {
        if (active_id_) {
            const ActionId old_id = *active_id_;
            auto& old = records_.at(old_id);

            if (!terminal(old.state)) {
                old.state = ActionState::CANCELING;
                old.reason = "Preempted by Stop";

                emit_locked(
                    old,
                    ActionEventKind::PREEMPTED,
                    old.reason);

                preempted_id_ = old_id;
            }
        } else if (canceling_id_) {
            // 已有普通 Cancel 正在进行时，
            // Stop 接管停止流程。
            preempted_id_ = canceling_id_;
        }

        canceling_id_.reset();
    }

    active_id_ = new_id;

    emit_locked(
        records_.at(new_id),
        ActionEventKind::ACCEPTED,
        "Action accepted");

    return {true, new_id, "Action accepted"};
}

bool ActionManager::begin_flight_transition_locked(
    ActionType type)
{
    FlightEvent event;

    switch (type) {
    case ActionType::TAKEOFF:
        event = FlightEvent::TAKEOFF_STARTED;
        break;
    case ActionType::LAND:
        event = FlightEvent::LAND_STARTED;
        break;
    case ActionType::GOTO:
        event = FlightEvent::GOTO_STARTED;
        break;
    case ActionType::STOP:
        // FAULT 下仍允许 Stop 作为紧急请求。
        // 但不能借此退出 FAULT。
        if (fsm_->phase() == FlightPhase::FAULT) {
            return true;
        }
        event = FlightEvent::STOP_STARTED;
        break;
    case ActionType::ARM:
    case ActionType::DISARM:
        return true;
    default:
        return false;
    }

    return fsm_->process_event(event).accepted;
}

bool ActionManager::start(ActionId id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = records_.find(id);

    if (it == records_.end() ||
        !active_id_ ||
        *active_id_ != id ||
        it->second.state != ActionState::PENDING) {
        return false;
    }

    auto& record = it->second;

    if (!begin_flight_transition_locked(record.type)) {
        enter_fault_locked(
            "Unable to begin flight transition");
        return false;
    }

    record.state = ActionState::EXECUTING;
    record.started_at = Clock::now();

    emit_locked(
        record,
        ActionEventKind::STARTED,
        "Action started");

    return true;
}

bool ActionManager::cancel(ActionId id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = records_.find(id);

    if (it == records_.end()) {
        return false;
    }

    auto& record = it->second;

    if (terminal(record.state)) {
        return false;
    }

    // Stop 是安全停止过程，不接受普通 Cancel 中断。
    if (record.type == ActionType::STOP) {
        return false;
    }

    if (record.state == ActionState::CANCELING) {
        return true;
    }

    if (!active_id_ || *active_id_ != id) {
        return false;
    }

    // ARM/DISARM 也需要等待取消交接确认，
    // 避免命令已发送但状态尚未确认的竞态。
    if (motion_action(record.type)) {
        const auto transition =
            fsm_->process_event(FlightEvent::STOP_STARTED);

        if (!transition.accepted) {
            enter_fault_locked(
                "Unable to start cancellation stop");
            return false;
        }
    }

    record.state = ActionState::CANCELING;
    record.reason = "Cancellation requested";
    canceling_id_ = id;

    emit_locked(
        record,
        ActionEventKind::CANCEL_REQUESTED,
        record.reason);

    return true;
}

bool ActionManager::finish_flight_transition_locked(
    ActionType type)
{
    FlightEvent event;

    switch (type) {
    case ActionType::ARM:
        event = FlightEvent::ARM_CONFIRMED;
        break;
    case ActionType::DISARM:
        event = FlightEvent::DISARM_CONFIRMED;
        break;
    case ActionType::TAKEOFF:
        event = FlightEvent::TAKEOFF_COMPLETED;
        break;
    case ActionType::LAND:
        event = FlightEvent::LAND_COMPLETED;
        break;
    case ActionType::GOTO:
        event = FlightEvent::GOTO_COMPLETED;
        break;
    case ActionType::STOP:
        return false;
    default:
        return false;
    }

    return fsm_->process_event(event).accepted;
}

bool ActionManager::finish_stop_locked(ActionId id)
{
    const auto snapshot = fsm_->snapshot();
    const auto phase = fsm_->phase();

    // FAULT 不允许由 Stop 解除。
    if (phase == FlightPhase::FAULT) {
        return false;
    }

    FlightEvent event;

    if (snapshot.connected &&
        snapshot.landed_valid &&
        snapshot.landed) {
        event = FlightEvent::STOP_GROUNDED_CONFIRMED;
    } else {
        event = FlightEvent::STOP_HOVER_CONFIRMED;
    }

    if (!fsm_->process_event(event).accepted) {
        return false;
    }

    auto& record = records_.at(id);

    // Stop 抢占的旧动作在停止确认后才算取消完成。
    if (preempted_id_) {
        const ActionId old_id = *preempted_id_;

        finish_locked(
            records_.at(old_id),
            ActionState::CANCELED,
            "Preempted and safely stopped");
    }

    if (canceling_id_) {
        const ActionId old_id = *canceling_id_;

        finish_locked(
            records_.at(old_id),
            ActionState::CANCELED,
            "Canceled and safely stopped");
    }

    if (record.type == ActionType::STOP) {
        finish_locked(
            record,
            ActionState::SUCCEEDED,
            "Stop completed");
    }

    return true;
}

bool ActionManager::complete(ActionId id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = records_.find(id);

    if (it == records_.end() || terminal(it->second.state)) {
        return false;
    }

    auto& record = it->second;

    // 被抢占的旧动作不能再以成功结束。
    if (preempted_id_ && *preempted_id_ == id) {
        return false;
    }

    // 普通取消流程收到安全停止确认。
    if (canceling_id_ && *canceling_id_ == id) {
        if (motion_action(record.type)) {
            return finish_stop_locked(id);
        }

        finish_locked(
            record,
            ActionState::CANCELED,
            "Cancellation confirmed");

        return true;
    }

    if (!active_id_ ||
        *active_id_ != id ||
        record.state != ActionState::EXECUTING) {
        return false;
    }

    if (record.type == ActionType::STOP) {
        return finish_stop_locked(id);
    }

    if (!finish_flight_transition_locked(record.type)) {
        return false;
    }

    finish_locked(
        record,
        ActionState::SUCCEEDED,
        "Action completed");

    return true;
}

void ActionManager::enter_fault_locked(
    const std::string& reason)
{
    fsm_->process_event(FlightEvent::FAULT_DETECTED);

    // 所有尚未结束的动作均终止。
    for (auto& entry : records_) {
        auto& record = entry.second;

        if (!terminal(record.state)) {
            finish_locked(
                record,
                ActionState::FAILED,
                reason);
        }
    }

    active_id_.reset();
    preempted_id_.reset();
    canceling_id_.reset();
}

bool ActionManager::fail(
    ActionId id,
    const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = records_.find(id);

    if (it == records_.end() || terminal(it->second.state)) {
        return false;
    }

    auto& record = it->second;

    // 旧动作已被 Stop 抢占，其迟到失败事件不应
    // 覆盖新 Stop 的执行过程。
    if (preempted_id_ && *preempted_id_ == id) {
        return false;
    }

    if (motion_action(record.type)) {
        // 运动执行失败时无法可靠判断是否安全悬停，
        // 第一版采取保守策略，进入 FAULT。
        enter_fault_locked(reason);
        return true;
    }

    finish_locked(record, ActionState::FAILED, reason);
    return true;
}

void ActionManager::tick()
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = Clock::now();

    // 先收集超时动作，避免遍历时反复修改状态。
    std::vector<ActionId> expired;

    for (const auto& entry : records_) {
        const auto& record = entry.second;

        if (terminal(record.state)) {
            continue;
        }

        const auto elapsed = now - record.created_at;

        if (elapsed >= record.timeout) {
            expired.push_back(record.id);
        }
    }

    for (const ActionId id : expired) {
        auto it = records_.find(id);

        if (it == records_.end() ||
            terminal(it->second.state)) {
            continue;
        }

        if (motion_action(it->second.type) ||
            it->second.state == ActionState::CANCELING) {
            enter_fault_locked("Action timeout");
            return;
        }

        finish_locked(
            it->second,
            ActionState::FAILED,
            "Action timeout");
    }
}

void ActionManager::report_fault(
    const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enter_fault_locked(reason);
}

bool ActionManager::recover_flight_state()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 不允许在动作尚未终止时执行恢复。
    if (active_id_ || preempted_id_ || canceling_id_) {
        return false;
    }

    return fsm_->process_event(
        FlightEvent::STATE_RECOVERED).accepted;
}

std::optional<ActionRecord> ActionManager::get(
    ActionId id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto it = records_.find(id);

    if (it == records_.end()) {
        return std::nullopt;
    }

    return it->second;
}

std::optional<ActionId> ActionManager::active_action() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return active_id_;
}

std::vector<ActionEvent> ActionManager::drain_events()
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<ActionEvent> result;
    result.reserve(events_.size());

    while (!events_.empty()) {
        result.push_back(std::move(events_.front()));
        events_.pop_front();
    }

    return result;
}

}  // namespace drone_node::exec
