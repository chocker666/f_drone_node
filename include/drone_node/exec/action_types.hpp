
#pragma once

#include <cstdint>
#include <string>
#include <chrono>

namespace drone_node::exec
{

enum class ActionType : std::uint8_t
{
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    GOTO,
    STOP,
    CONTINUE,
    HOVER
};

enum class ActionState : std::uint8_t
{
    PENDING,
    EXECUTING,
    CANCELING,
    SUCCEEDED,
    CANCELED,
    FAILED
};

enum class FlightPhase : std::uint8_t
{
    UNKNOWN,
    GROUNDED,
    TAKING_OFF,
    HOVERING,
    MOVING,
    STOPPING,
    LANDING,
    FAULT
};

// MotionExecutor 根据真实缓存和接收时间生成。
// *_valid 表示对应数据在生成快照时有效且新鲜。
struct VehicleSnapshot
{
    std::chrono::steady_clock::time_point captured_at{};

    bool connected{false};
    bool connected_valid{false};

    bool armed{false};
    bool armed_valid{false};

    bool position_valid{false};
    bool velocity_valid{false};
    bool attitude_valid{false};

    bool landed_valid{false};
    bool landed{false};

    bool control_status_valid{false};
    bool control_available{false};
    bool control_tracking{false};
    bool control_hover{false};
    bool control_fault{false};

    // Ctrl HOVER 加上真实位置、Yaw、速度的连续稳定观测。
    // 不代替任务的执行完成事件。
    bool hover_confirmed{false};

    bool execution_active{false};
};

// 状态机接收的事件。
// 动作执行请求和实际完成确认严格分开。
enum class FlightEvent : std::uint8_t
{
    SYNC_GROUNDED,
    SYNC_AIRBORNE,

    ARM_CONFIRMED,
    DISARM_CONFIRMED,

    TAKEOFF_STARTED,
    TAKEOFF_COMPLETED,

    GOTO_STARTED,
    GOTO_COMPLETED,

    LAND_STARTED,
    LAND_COMPLETED,

    STOP_STARTED,
    STOP_HOVER_CONFIRMED,
    STOP_GROUNDED_CONFIRMED,

    // 控制异常、估计失效等严重故障。
    FAULT_DETECTED,

    // 唯一允许退出 FAULT 的事件。
    STATE_RECOVERED
};

struct TransitionResult
{
    bool accepted{false};
    FlightPhase previous{FlightPhase::UNKNOWN};
    FlightPhase current{FlightPhase::UNKNOWN};
    std::string reason;
};

// 动作准入判断，不负责占用动作执行权。
struct AdmissionResult
{
    bool accepted{false};
    std::string reason;
};

inline const char* to_string(FlightPhase phase)
{
    switch (phase) {
    case FlightPhase::UNKNOWN: return "UNKNOWN";
    case FlightPhase::GROUNDED: return "GROUNDED";
    case FlightPhase::TAKING_OFF: return "TAKING_OFF";
    case FlightPhase::HOVERING: return "HOVERING";
    case FlightPhase::MOVING: return "MOVING";
    case FlightPhase::STOPPING: return "STOPPING";
    case FlightPhase::LANDING: return "LANDING";
    case FlightPhase::FAULT: return "FAULT";
    }
    return "INVALID";
}

inline const char* to_string(ActionType type)
{
    switch (type) {
    case ActionType::ARM: return "ARM";
    case ActionType::DISARM: return "DISARM";
    case ActionType::TAKEOFF: return "TAKEOFF";
    case ActionType::LAND: return "LAND";
    case ActionType::GOTO: return "GOTO";
    case ActionType::STOP: return "STOP";
    case ActionType::CONTINUE: return "CONTINUE";
    case ActionType::HOVER: return "HOVER";
    }
    return "INVALID";
}

}  // namespace drone_node::exec
