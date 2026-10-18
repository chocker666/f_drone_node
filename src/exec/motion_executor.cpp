#include "drone_node/exec/motion_executor.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>
#include <stdexcept>

namespace drone_node::exec
{

namespace
{

template<typename T>
inline constexpr bool always_false_v = false;

template<typename ActionT>
std::string action_name()
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
        static_assert(always_false_v<ActionT>, "Unsupported action");
}

}  // namespace

MotionExecutor::MotionExecutor(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("motion_executor", options)
{
    mock_duration_ =
        this->declare_parameter<double>("mock_duration", 3.0);

    max_descent_speed_ =
        this->declare_parameter<double>("max_descent_speed", 1.0);

    if (!std::isfinite(mock_duration_) || mock_duration_ <= 0.0) {
        throw std::runtime_error("mock_duration must be positive");
    }

    if (!std::isfinite(max_descent_speed_) ||
        max_descent_speed_ <= 0.0) {
        throw std::runtime_error(
            "max_descent_speed must be positive");
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

    mock_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MotionExecutor::update_mock_task, this));

    RCLCPP_INFO(
        get_logger(),
        "MotionExecutor started in MOCK mode");
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
        "Action server registered: %s",
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
    const GoalUUID&,
    std::shared_ptr<const typename ActionT::Goal> goal)
{
    if (!validate_goal<ActionT>(*goal)) {
        RCLCPP_WARN(
            get_logger(),
            "%s goal rejected: invalid parameters",
            action_name<ActionT>().c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(task_mutex_);

    if (goal_reserved_ || active_task_) {
        RCLCPP_WARN(
            get_logger(),
            "%s goal rejected: executor busy",
            action_name<ActionT>().c_str());

        return rclcpp_action::GoalResponse::REJECT;
    }

    goal_reserved_ = true;

    RCLCPP_INFO(
        get_logger(),
        "%s goal accepted",
        action_name<ActionT>().c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template<typename ActionT>
rclcpp_action::CancelResponse MotionExecutor::handle_cancel(
    const std::shared_ptr<GoalHandle<ActionT>>)
{
    RCLCPP_INFO(
        get_logger(),
        "%s cancel requested",
        action_name<ActionT>().c_str());

    return rclcpp_action::CancelResponse::ACCEPT;
}

template<typename ActionT>
void MotionExecutor::handle_accepted(
    const std::shared_ptr<GoalHandle<ActionT>> goal_handle)
{
    start_mock_task<ActionT>(goal_handle);
}

template<typename ActionT>
void MotionExecutor::start_mock_task(
    const std::shared_ptr<GoalHandle<ActionT>> goal_handle)
{
    auto task = std::make_shared<MockTask>();

    task->name = action_name<ActionT>();
    task->start_time = std::chrono::steady_clock::now();
    task->duration_seconds = mock_duration_;

    task->is_canceling = [goal_handle]() {
        return goal_handle->is_canceling();
    };

    task->publish_feedback =
        [goal_handle](double progress)
    {
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
                // Synthetic distance, not vehicle telemetry.
                feedback->remaining_distance =
                    1.0 - progress;
            }
            else {
                // Synthetic height, not vehicle telemetry.
                feedback->current_height = progress;
            }
        }
        else {
            feedback->stage =
                progress < 0.5 ? 1 : 3;

            feedback->message =
                progress < 0.5
                    ? "MOCK: request accepted"
                    : "MOCK: waiting for simulated confirmation";
        }

        goal_handle->publish_feedback(feedback);
    };

    task->finish =
        [goal_handle](bool canceled)
    {
        auto result =
            std::make_shared<typename ActionT::Result>();

        if (canceled) {
            result->success = false;
            result->error_code = 8;
            result->message = "MOCK: task canceled";

            goal_handle->canceled(result);
        }
        else {
            result->success = true;
            result->error_code = 0;
            result->message = "MOCK: task completed";

            goal_handle->succeed(result);
        }
    };

    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        active_task_ = task;
        goal_reserved_ = false;
    }

    RCLCPP_INFO(
        get_logger(),
        "MOCK task started: %s",
        task->name.c_str());
}

void MotionExecutor::update_mock_task()
{
    std::shared_ptr<MockTask> task;

    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task = active_task_;
    }

    if (!task) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    const double elapsed =
        std::chrono::duration<double>(
            now - task->start_time).count();

    const double progress = std::clamp(
        elapsed / task->duration_seconds,
        0.0,
        1.0);

    const bool canceled = task->is_canceling();
    const bool finished = canceled || progress >= 1.0;

    if (!finished) {
        task->publish_feedback(progress);
        return;
    }

    // Keep the executor busy while finishing the Action.
    // This prevents a new Goal from being accepted during
    // terminal-state processing.
    try {
        task->finish(canceled);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_logger(),
            "MOCK task finish error: %s",
            e.what());
    }

    {
        std::lock_guard<std::mutex> lock(task_mutex_);

        if (active_task_ == task) {
            active_task_.reset();
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "MOCK task finished: %s (%s)",
        task->name.c_str(),
        canceled ? "CANCELED" : "SUCCEEDED");
}

// Explicit instantiation is not needed: all template usages
// occur within this translation unit.

}  // namespace drone_node::exec