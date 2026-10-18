
#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drone_node/exec/action_types.hpp"
#include "drone_node/exec/flight_state_machine.hpp"

namespace drone_node::exec
{

using ActionId = std::uint64_t;

enum class ActionEventKind : std::uint8_t
{
    ACCEPTED,
    STARTED,
    CANCEL_REQUESTED,
    PREEMPTED,
    SUCCEEDED,
    CANCELED,
    FAILED
};

struct ActionEvent
{
    ActionId id{0};
    ActionType type{ActionType::STOP};
    ActionEventKind kind{ActionEventKind::FAILED};
    ActionState state{ActionState::FAILED};
    std::string reason;
};

struct ActionRecord
{
    ActionId id{0};
    ActionType type{ActionType::STOP};
    ActionState state{ActionState::PENDING};

    std::chrono::steady_clock::time_point created_at{};
    std::chrono::steady_clock::time_point started_at{};

    std::chrono::milliseconds timeout{30000};
    std::string reason;
};

struct SubmitResult
{
    bool accepted{false};
    ActionId id{0};
    std::string reason;
};

class ActionManager
{
public:
    explicit ActionManager(
        std::shared_ptr<FlightStateMachine> flight_state_machine);

    // 原子准入。Stop 可抢占正在执行的普通动作。
    SubmitResult submit(
        ActionType type,
        std::chrono::milliseconds timeout =
            std::chrono::milliseconds(30000));

    // 将 PENDING 动作转为 EXECUTING。
    bool start(ActionId id);

    // 取消普通动作。
    // 接受取消不代表已经安全停止。
    bool cancel(ActionId id);

    // 外部执行器确认动作完成。
    // 对普通运动动作，需要先完成飞行状态转换。
    bool complete(ActionId id);

    // 外部执行器报告动作失败。
    bool fail(ActionId id, const std::string& reason);

    // 处理超时。由外部定时器周期调用。
    void tick();

    // 全局故障：先使飞行状态机进入 FAULT，
    // 再终止所有未结束动作。
    void report_fault(const std::string& reason);

    // 唯一允许请求退出 FAULT 的管理器接口。
    // 内部仍由 FlightStateMachine 判定是否恢复成功。
    bool recover_flight_state();

    // 查询动作记录。
    std::optional<ActionRecord> get(ActionId id) const;

    // 当前拥有执行权的动作。
    std::optional<ActionId> active_action() const;

    // 获取并清空待处理的生命周期事件。
    std::vector<ActionEvent> drain_events();

private:
    using Clock = std::chrono::steady_clock;

    static bool terminal(ActionState state);
    static bool motion_action(ActionType type);

    void emit_locked(
        const ActionRecord& record,
        ActionEventKind kind,
        const std::string& reason);

    void finish_locked(
        ActionRecord& record,
        ActionState state,
        const std::string& reason);

    bool begin_flight_transition_locked(ActionType type);

    bool finish_flight_transition_locked(ActionType type);

    // Stop 或 Cancel 的停止确认。
    bool finish_stop_locked(ActionId id);

    // 停止失败时，进入 FAULT 并终止所有动作。
    void enter_fault_locked(const std::string& reason);

    std::shared_ptr<FlightStateMachine> fsm_;

    mutable std::mutex mutex_;

    ActionId next_id_{1};

    std::unordered_map<ActionId, ActionRecord> records_;

    // 当前拥有控制权的动作。
    std::optional<ActionId> active_id_;

    // Stop 抢占的旧动作，等待安全停止后终结。
    std::optional<ActionId> preempted_id_;

    // 普通 Cancel 正在等待停止确认。
    std::optional<ActionId> canceling_id_;

    std::deque<ActionEvent> events_;
};

}  // namespace drone_node::exec
