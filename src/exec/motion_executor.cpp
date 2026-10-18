
#include "drone_node/exec/motion_executor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace drone_node::exec
{

namespace
{

template<typename T>
inline constexpr bool always_false_v = false;

template<typename ActionT>
const char* action_name()
{
    if constexpr (std::is_same_v<ActionT, drone_node::action::GoTo>)
        return "GoTo";
    else if constexpr (std::is_same_v<ActionT, drone_node::action::Takeoff>)
        return "Takeoff";
    else if constexpr (std::is_same_v<ActionT, drone_node::action::Land>)
        return "Land";
    else if constexpr (std::is_same_v<ActionT, drone_node::action::Arm>)
        return "Arm";
    else if constexpr (std::is_same_v<ActionT, drone_node::action::Disarm>)
        return "Disarm";
    else if constexpr (std::is_same_v<ActionT, drone_node::action::Stop>)
        return "Stop";
    else
        static_assert(always_false_v<ActionT>, "Unsupported Action");
}

bool terminal(ActionState state)
{
    return state == ActionState::SUCCEEDED ||
           state == ActionState::CANCELED ||
           state == ActionState::FAILED;
}

bool motion_action(ActionType type)
{
    return type == ActionType::TAKEOFF ||
           type == ActionType::LAND ||
           type == ActionType::GOTO ||
           type == ActionType::STOP;
}

}  // namespace

MotionExecutor::MotionExecutor(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("motion_executor", options)
{
    mock_duration_ =
        declare_parameter<double>("mock_duration", 3.0);

    mock_stop_duration_ =
        declare_parameter<double>("mock_stop_duration", 1.0);

    max_descent_speed_ =
        declare_parameter<double>("max_descent_speed", 1.0);

    action_timeout_ =
        declare_parameter<double>("action_timeout", 30.0);

    const bool initially_airborne =
        declare_parameter<bool>("mock_initially_airborne", false);

    const bool initially_armed =
        declare_parameter<bool>("mock_initially_armed", false);

    if (!std::isfinite(mock_duration_) ||
        mock_duration_ <= 0.0 ||
        !std::isfinite(mock_stop_duration_) ||
        mock_stop_duration_ <= 0.0 ||
        !std::isfinite(action_timeout_) ||
        action_timeout_ <= 0.0 ||
        !std::isfinite(max_descent_speed_) ||
        max_descent_speed_ <= 0.0) {
        throw std::runtime_error(
            "Mock durations, timeout and descent speed "
            "must be finite and positive");
    }

    fsm_ = std::make_shared<FlightStateMachine>();
    manager_ = std::make_shared<ActionManager>(fsm_);

    mock_snapshot_.connected = true;
    mock_snapshot_.position_valid = true;
    mock_snapshot_.control_available = true;
    mock_snapshot_.landed_valid = true;
    mock_snapshot_.landed = !initially_airborne;
    mock_snapshot_.armed =
        initially_airborne || initially_armed;

    fsm_->update_snapshot(mock_snapshot_);

    const auto sync_event = initially_airborne
        ? FlightEvent::SYNC_AIRBORNE
        : FlightEvent::SYNC_GROUNDED;

    if (!fsm_->process_event(sync_event).accepted) {
        throw std::runtime_error(
            "Unable to initialize Mock flight state");
    }

    create_action_server<GoTo>(
        "/drone_node/goto", goto_server_);

    create_action_server<Takeoff>(
        "/drone_node/takeoff", takeoff_server_);

    create_action_server<Land>(
        "/drone_node/land", land_server_);

    create_action_server<Arm>(
        "/drone_node/arm", arm_server_);

    create_action_server<Disarm>(
        "/drone_node/disarm", disarm_server_);

    create_action_server<Stop>(
        "/drone_node/stop", stop_server_);

    mock_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotionExecutor::update_mock, this));

    RCLCPP_INFO(
        get_logger(),
        "MotionExecutor MOCK started, phase=%s",
        to_string(fsm_->phase()));
}

std::string MotionExecutor::uuid_key(
    const GoalUUID& uuid)
{
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');

    for (const auto byte : uuid) {
        stream << std::setw(2)
               << static_cast<unsigned int>(byte);
    }

    return stream.str();
}

template<typename ActionT>
constexpr ActionType MotionExecutor::action_type()
{
    if constexpr (std::is_same_v<ActionT, GoTo>)
        return ActionType::GOTO;
    else if constexpr (std::is_same_v<ActionT, Takeoff>)
        return ActionType::TAKEOFF;
    else if constexpr (std::is_same_v<ActionT, Land>)
        return ActionType::LAND;
    else if constexpr (std::is_same_v<ActionT, Arm>)
        return ActionType::ARM;
    else if constexpr (std::is_same_v<ActionT, Disarm>)
        return ActionType::DISARM;
    else if constexpr (std::is_same_v<ActionT, Stop>)
        return ActionType::STOP;
    else
        static_assert(always_false_v<ActionT>, "Unsupported Action");
}

template<typename ActionT>
void MotionExecutor::create_action_server(
    const std::string& name,
    typename Server<ActionT>::SharedPtr& server)
{
    server = rclcpp_action::create_server<ActionT>(
        this,
        name,
        [this](
            const GoalUUID& uuid,
            std::shared_ptr<const typename ActionT::Goal> goal)
        {
            return handle_goal<ActionT>(uuid, goal);
        },
        [this](
            const std::shared_ptr<GoalHandle<ActionT>> handle)
        {
            return handle_cancel<ActionT>(handle);
        },
        [this](
            const std::shared_ptr<GoalHandle<ActionT>> handle)
        {
            handle_accepted<ActionT>(handle);
        });

    RCLCPP_INFO(
        get_logger(),
        "Registered Action server: %s",
        name.c_str());
}

template<typename ActionT>
bool MotionExecutor::validate_goal(
    const typename ActionT::Goal& goal) const
{
    if constexpr (std::is_same_v<ActionT, GoTo>) {
        return goal.header.frame_id == "map" &&
               std::isfinite(goal.position.x) &&
               std::isfinite(goal.position.y) &&
               std::isfinite(goal.position.z) &&
               std::isfinite(goal.yaw);
    }
    else if constexpr (std::is_same_v<ActionT, Takeoff>) {
        return std::isfinite(goal.height) &&
               goal.height > 0.0;
    }
    else if constexpr (std::is_same_v<ActionT, Land>) {
        return std::isfinite(goal.descent_speed) &&
               goal.descent_speed > 0.0 &&
               goal.descent_speed <= max_descent_speed_;
    }
    else {
        (void)goal;
        return true;
    }
}

template<typename ActionT>
rclcpp_action::GoalResponse MotionExecutor::handle_goal(
    const GoalUUID& uuid,
    std::shared_ptr<const typename ActionT::Goal> goal)
{
    if (!validate_goal<ActionT>(*goal)) {
        RCLCPP_WARN(
            get_logger(),
            "%s rejected: invalid goal",
            action_name<ActionT>());

        return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    const auto timeout = std::chrono::milliseconds(
        static_cast<std::int64_t>(action_timeout_ * 1000.0));

    const auto submitted =
        manager_->submit(action_type<ActionT>(), timeout);

    if (!submitted.accepted) {
        RCLCPP_WARN(
            get_logger(),
            "%s rejected: %s",
            action_name<ActionT>(),
            submitted.reason.c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto key = uuid_key(uuid);

    uuid_to_id_[key] = submitted.id;

    GoalBinding binding;
    binding.id = submitted.id;
    binding.type = action_type<ActionT>();

    bindings_.emplace(submitted.id, std::move(binding));

    RCLCPP_INFO(
        get_logger(),
        "%s accepted: ActionId=%llu",
        action_name<ActionT>(),
        static_cast<unsigned long long>(submitted.id));

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template<typename ActionT>
rclcpp_action::CancelResponse MotionExecutor::handle_cancel(
    const std::shared_ptr<GoalHandle<ActionT>> handle)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto it = uuid_to_id_.find(
        uuid_key(handle->get_goal_id()));

    if (it == uuid_to_id_.end()) {
        return rclcpp_action::CancelResponse::REJECT;
    }

    if (!manager_->cancel(it->second)) {
        return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_INFO(
        get_logger(),
        "%s cancellation accepted: ActionId=%llu",
        action_name<ActionT>(),
        static_cast<unsigned long long>(it->second));

    return rclcpp_action::CancelResponse::ACCEPT;
}

template<typename ActionT>
void MotionExecutor::handle_accepted(
    const std::shared_ptr<GoalHandle<ActionT>> handle)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto it = uuid_to_id_.find(
        uuid_key(handle->get_goal_id()));

    if (it == uuid_to_id_.end()) {
        RCLCPP_ERROR(
            get_logger(),
            "Accepted Goal has no ActionId");
        return;
    }

    const ActionId id = it->second;

    attach_goal<ActionT>(id, handle);

    // Stop 可能在旧 Goal 的 accepted 回调之前到达。
    // 此时旧动作已被抢占，不允许重新启动。
    if (!manager_->start(id)) {
        RCLCPP_WARN(
            get_logger(),
            "ActionId=%llu cannot start (possibly preempted)",
            static_cast<unsigned long long>(id));
    }
}

template<typename ActionT>
void MotionExecutor::attach_goal(
    ActionId id,
    const std::shared_ptr<GoalHandle<ActionT>>& handle)
{
    auto it = bindings_.find(id);

    if (it == bindings_.end()) {
        return;
    }

    auto& binding = it->second;

    binding.attached = true;
    binding.started_at = Clock::now();

    binding.feedback = [handle](double progress)
    {
        if (!handle->is_active()) {
            return;
        }

        auto feedback =
            std::make_shared<typename ActionT::Feedback>();

        if constexpr (
            std::is_same_v<ActionT, GoTo> ||
            std::is_same_v<ActionT, Takeoff> ||
            std::is_same_v<ActionT, Land>)
        {
            feedback->phase = "MOCK_EXECUTING";
            feedback->progress =
                static_cast<float>(progress);

            if constexpr (std::is_same_v<ActionT, GoTo>) {
                feedback->remaining_distance =
                    1.0 - progress;
            }
            else {
                feedback->current_height = progress;
            }
        }
        else {
            feedback->stage = progress < 1.0 ? 1 : 3;
            feedback->message =
                "MOCK: waiting for simulated confirmation";
        }

        handle->publish_feedback(feedback);
    };

    binding.finish =
        [handle](
            ActionState state,
            const std::string& reason,
            bool preempted)
    {
        if (!handle->is_active()) {
            return;
        }

        auto result =
            std::make_shared<typename ActionT::Result>();

        result->success = state == ActionState::SUCCEEDED;
        result->message = reason;

        if (state == ActionState::SUCCEEDED) {
            result->error_code = 0;
            handle->succeed(result);
        }
        else if (state == ActionState::CANCELED &&
                 !preempted &&
                 handle->is_canceling()) {
            result->error_code = 8;
            handle->canceled(result);
        }
        else {
            // 8: cancellation
            // 9: preempted
            // 10: execution failed
            // 这些错误码是当前 Mock 适配层的内部约定。
            result->error_code = preempted ? 9 : 10;
            handle->abort(result);
        }
    };
}

void MotionExecutor::apply_mock_confirmation(
    ActionId id,
    ActionType type,
    bool canceling)
{
    // 必须先模拟观测确认，再请求 ActionManager 完成。
    // 不直接修改 FlightStateMachine 的 FlightPhase。

    if (canceling || type == ActionType::STOP) {
        // 停止后的状态取决于模拟飞机是否已经接地。
        // 在空中则确认稳定悬停。
        mock_snapshot_.connected = true;
        mock_snapshot_.position_valid = true;
        mock_snapshot_.control_available = true;

        if (!mock_snapshot_.landed) {
            mock_snapshot_.armed = true;
        }
    }
    else {
        switch (type) {
        case ActionType::ARM:
            mock_snapshot_.armed = true;
            break;

        case ActionType::DISARM:
            mock_snapshot_.armed = false;
            break;

        case ActionType::TAKEOFF:
            mock_snapshot_.armed = true;
            mock_snapshot_.landed = false;
            mock_snapshot_.landed_valid = true;
            break;

        case ActionType::LAND:
            mock_snapshot_.landed = true;
            mock_snapshot_.landed_valid = true;
            break;

        case ActionType::GOTO:
        case ActionType::STOP:
            break;
        }
    }

    fsm_->update_snapshot(mock_snapshot_);

    if (!manager_->complete(id)) {
        RCLCPP_ERROR(
            get_logger(),
            "MOCK confirmation rejected: ActionId=%llu, phase=%s",
            static_cast<unsigned long long>(id),
            to_string(fsm_->phase()));

        manager_->fail(id, "MOCK completion rejected by state machine");
    }
}

void MotionExecutor::publish_mock_feedback(
    ActionId id,
    double progress)
{
    const auto it = bindings_.find(id);

    if (it == bindings_.end() ||
        !it->second.attached ||
        !it->second.feedback) {
        return;
    }

    try {
        it->second.feedback(progress);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "Feedback error: %s",
            e.what());
    }
}


void MotionExecutor::process_events()
{
    auto events = manager_->drain_events();

    events.insert(
        events.end(),
        deferred_events_.begin(),
        deferred_events_.end());

    deferred_events_.clear();

    for (const auto& event : events) {
        if (event.kind != ActionEventKind::SUCCEEDED &&
            event.kind != ActionEventKind::CANCELED &&
            event.kind != ActionEventKind::FAILED) {
            continue;
        }

        const auto it = bindings_.find(event.id);

        if (it == bindings_.end()) {
            continue;
        }

        if (!it->second.attached || !it->second.finish) {
            deferred_events_.push_back(event);
            continue;
        }

        const bool preempted =
            event.reason.find("Preempted") != std::string::npos;

        try {
            it->second.finish(
                event.state,
                event.reason,
                preempted);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(
                get_logger(),
                "Result delivery error: %s",
                e.what());
        }

        for (auto uuid_it = uuid_to_id_.begin();
             uuid_it != uuid_to_id_.end();) {
            if (uuid_it->second == event.id) {
                uuid_it = uuid_to_id_.erase(uuid_it);
            }
            else {
                ++uuid_it;
            }
        }

        bindings_.erase(it);

        RCLCPP_INFO(
            get_logger(),
            "ActionId=%llu terminated: %s",
            static_cast<unsigned long long>(event.id),
            event.reason.c_str());
    }
}


void MotionExecutor::update_mock()
{
    std::lock_guard<std::mutex> lock(mutex_);

    manager_->tick();

    const auto active = manager_->active_action();

    if (active) {
        const ActionId id = *active;

        const auto record = manager_->get(id);
        const auto binding_it = bindings_.find(id);

        if (record &&
            binding_it != bindings_.end() &&
            binding_it->second.attached) {

            const auto state = record->state;

            if (state == ActionState::EXECUTING ||
                state == ActionState::CANCELING) {

                const auto now = Clock::now();

                const double elapsed =
                    std::chrono::duration<double>(
                        now - binding_it->second.started_at).count();

                const bool stopping =
                    state == ActionState::CANCELING ||
                    record->type == ActionType::STOP;

                const double duration = stopping
                    ? mock_stop_duration_
                    : mock_duration_;

                const double progress = std::clamp(
                    elapsed / duration, 0.0, 1.0);

                if (progress < 1.0) {
                    publish_mock_feedback(id, progress);
                }
                else {
                    apply_mock_confirmation(
                        id, record->type,
                        state == ActionState::CANCELING);
                }
            }
        }
    }

    // 统一处理成功、取消、失败事件。
    process_events();
}

}  // namespace drone_node::exec
