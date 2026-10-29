#include "drone_node/exec/action_manager.hpp"
#include "action_helpers.hpp"

#include <chrono>
#include <string>
#include <cmath>
#include <utility>
#include <stdexcept>

namespace drone_node::exec
{

double ActionManager::measuredYaw(
    const geometry_msgs::msg::QuaternionStamped& attitude)
{
    const auto& q = attitude.quaternion;

    const double norm = std::hypot(
        std::hypot(q.x, q.y),
        std::hypot(q.z, q.w));

    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;

    return std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
}

void ActionManager::startMotion()
{
    if (!motion_)
        return;

    refreshSnapshot();

    const auto admission =
        fsm_->check_action(motion_->type);

    if (!admission.accepted) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            admission.reason);
        return;
    }

    const auto raw = motion_executor_->getStateSnapshot();
    const auto now = Clock::now();

    const double timeout =
        node_.get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, timeout) ||
        !freshCache(raw.velocity, now, timeout) ||
        !freshCache(raw.attitude, now, timeout) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion start feedback is invalid or stale");
        return;
    }

    const auto& position = raw.position.message.point;
    const auto& velocity = raw.velocity.message.vector;

    PlanningRequest request = motion_->request;

    // 规划起点：使用当前飞机反馈。
    // 当前状态消息没有提供加速度，因此起点加速度暂设为零。
    request.start.position = Eigen::Vector3d(
        position.x, position.y, position.z);

    request.start.velocity = Eigen::Vector3d(
        velocity.x, velocity.y, velocity.z);

    request.start.acceleration = Eigen::Vector3d::Zero();
    request.start.yaw = measuredYaw(raw.attitude.message);
    request.start.yaw_rate = 0.0;
    request.start.yaw_acceleration = 0.0;

    // 当前只实现五次多项式规划。
    request.planner_type = PlannerType::QUINTIC;

    // 默认终点静止。
    request.goal.velocity = Eigen::Vector3d::Zero();
    request.goal.acceleration = Eigen::Vector3d::Zero();
    request.goal.yaw_rate = 0.0;
    request.goal.yaw_acceleration = 0.0;

    if (motion_->type == ActionType::TAKEOFF &&
        !motion_->resumed_target) {

        request.frame_id = "map";

        request.goal.position = Eigen::Vector3d(
            position.x,
            position.y,
            position.z + motion_->height);

        request.goal.yaw = request.start.yaw;
        request.yaw_direction =
            PlanningRequest::YawDirection::SHORTEST;

        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }
    else if (motion_->type == ActionType::LAND &&
            !motion_->resumed_target) {

        if (position.z <= LAND_TARGET_Z) {
            finishMotion(
                ActionState::FAILED,
                ErrorCode::STATE_REJECTED,
                "Land requires current z above landing target");
            return;
        }

        request.frame_id = "map";

        request.goal.position = Eigen::Vector3d(
            position.x,
            position.y,
            LAND_TARGET_Z);

        request.goal.yaw = request.start.yaw;
        request.yaw_direction =
            PlanningRequest::YawDirection::SHORTEST;

        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }
    else if (motion_->is_hover &&
            !motion_->resumed_target) {

        request.frame_id = "map";

        request.goal.position = request.start.position;
        request.goal.yaw = request.start.yaw;

        request.yaw_direction =
            PlanningRequest::YawDirection::SHORTEST;

        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }

    // 必须保证规划时间有效。
    // 不在这里随意设置默认时长，以免掩盖上层配置错误。
    if (!std::isfinite(request.duration) ||
        request.duration <= 0.0) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion duration must be finite and positive");
        return;
    }

    const auto planning = planner_.plan(request);

    if (!planning.success()) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion planning failed: " + planning.reason);
        return;
    }

    const double initial_distance =
        (request.goal.position - request.start.position).norm();

    if (!std::isfinite(initial_distance)) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion target distance is not finite");
        return;
    }

    if (!motion_->resumed_target)
        motion_->start_z = position.z;

    motion_->initial_distance = initial_distance;

    // 保存本次实际规划请求，供后续目标检查和反馈使用。
    motion_->request = request;

    // 保存新生成的 ControlTrajectory。
    motion_->motion = planning.trajectory;


    const auto start_event =
        motion_->type == ActionType::TAKEOFF
            ? FlightEvent::TAKEOFF_STARTED
            : (motion_->type == ActionType::LAND
                ? FlightEvent::LAND_STARTED
                : FlightEvent::GOTO_STARTED);

    const auto transition = fsm_->process_event(start_event);

    if (!transition.accepted) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            transition.reason);
        return;
    }

    motion_->fsm_started = true;

    std::string rejection;

    const auto id = motion_executor_->startExecution(
        motion_->motion,
        motion_->constraints,
        rejection);

    if (!id) {
        latchFault("Motion dispatch failed: " + rejection);

        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            rejection);
        return;
    }

    motion_->trajectory_id = *id;
    motion_->dispatched_at = Clock::now();

    RCLCPP_INFO(
        node_.get_logger(),
        "%s dispatched: id=%llu "
        "target=(%.3f, %.3f, %.3f) yaw=%.3f",
        motion_->is_continue
            ? "Continue"
            : to_string(motion_->source_type),
        static_cast<unsigned long long>(*id),
        request.goal.position.x(),
        request.goal.position.y(),
        request.goal.position.z(),
        request.goal.yaw);
}

bool ActionManager::motionAtTarget() const
{
    if (!motion_)
        return false;

    const auto observation = fsm_->snapshot();

    if (!observation.position_valid ||
        !observation.velocity_valid ||
        !observation.attitude_valid ||
        !observation.armed_valid ||
        !observation.armed ||
        !observation.landed_valid ||
        observation.landed ||
        !observation.hover_confirmed) {
        return false;
    }

    const auto raw = motion_executor_->getStateSnapshot();
    const auto now = Clock::now();

    const double timeout =
        node_.get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, timeout) ||
        !freshCache(raw.velocity, now, timeout) ||
        !freshCache(raw.attitude, now, timeout) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        return false;
    }

    const auto& p = raw.position.message.point;
    const auto& v = raw.velocity.message.vector;
    const auto& target = motion_->request.goal.position;

    const double position_error = std::hypot(
        p.x - target.x(),
        p.y - target.y(),
        p.z - target.z());

    const double speed = std::hypot(v.x, v.y, v.z);

    const double delta =
        measuredYaw(raw.attitude.message) - motion_->request.goal.yaw;

    const double yaw_error =
        std::abs(std::atan2(std::sin(delta), std::cos(delta)));

    return
        position_error <=
            motion_->constraints.position_tolerance &&
        speed <=
            motion_->constraints.velocity_tolerance &&
        yaw_error <=
            motion_->constraints.yaw_tolerance;
}

bool ActionManager::landingConfirmed() const
{
    if (!motion_ || motion_->type != ActionType::LAND)
        return false;

    const auto observation = fsm_->snapshot();

    if (!observation.connected_valid ||
        !observation.connected ||
        !observation.armed_valid ||
        !observation.landed_valid ||
        !observation.landed ||
        !observation.position_valid ||
        !observation.velocity_valid ||
        !observation.attitude_valid ||
        !observation.control_status_valid ||
        !observation.control_available ||
        !observation.control_hover ||
        observation.control_fault ||
        observation.execution_active) {
        return false;
    }

    const auto raw = motion_executor_->getStateSnapshot();
    const auto now = Clock::now();

    const double timeout =
        node_.get_parameter(
            "execution.state_timeout").as_double();

    if (!freshCache(raw.position, now, timeout) ||
        !freshCache(raw.velocity, now, timeout) ||
        !freshCache(raw.attitude, now, timeout) ||
        raw.position.message.header.frame_id != "map" ||
        raw.velocity.message.header.frame_id != "map" ||
        raw.attitude.message.header.frame_id != "map") {
        return false;
    }

    const auto& p = raw.position.message.point;
    const auto& v = raw.velocity.message.vector;
    const auto& target = motion_->request.goal.position;

    // -1.0 是触发下降的参考点，不要求飞机实际到达地下高度。
    const double horizontal_error =
        std::hypot(p.x - target.x(), p.y - target.y());

    const double speed = std::hypot(v.x, v.y, v.z);

    const double delta =
        measuredYaw(raw.attitude.message) - motion_->request.goal.yaw;

    const double yaw_error =
        std::abs(std::atan2(std::sin(delta), std::cos(delta)));

    return
        horizontal_error <=
            motion_->constraints.position_tolerance &&
        speed <=
            motion_->constraints.velocity_tolerance &&
        yaw_error <=
            motion_->constraints.yaw_tolerance;
}

void ActionManager::beginLandDisarm()
{
    if (!motion_ ||
        motion_->type != ActionType::LAND ||
        active_) {
        throw std::logic_error(
            "Invalid Land Disarm operation state");
    }

    ArmOperation operation;
    operation.uuid = motion_->uuid;
    operation.desired_arm = false;
    operation.attached = true;
    operation.started_at = Clock::now();

    operation.feedback =
        [this](std::uint8_t, const std::string&)
        {
            if (!motion_ || !motion_->feedback)
                return;

            const auto raw = motion_executor_->getStateSnapshot();
            const auto now = Clock::now();

            const double timeout =
                node_.get_parameter(
                    "execution.state_timeout").as_double();

            if (!freshCache(raw.position, now, timeout) ||
                raw.position.message.header.frame_id != "map") {
                return;
            }

            const auto& p = raw.position.message.point;
            double value = p.z;

            if (motion_->is_continue) {
                const auto& target = motion_->request.goal.position;
                value = std::hypot(
                    p.x - target.x(),
                    p.y - target.y(),
                    p.z - target.z());
            }

            motion_->feedback("DISARMING", 1.0F, value);
        };

    operation.finish =
        [this](
            ActionState state,
            ErrorCode error,
            const std::string& message)
        {
            finishMotion(
                state,
                error,
                state == ActionState::SUCCEEDED
                    ? "Landing and actual Disarm confirmed"
                    : "Landing confirmed; Disarm stage: " + message);
        };

    motion_->landing_disarm = true;
    active_.emplace(std::move(operation));
}

}  // namespace drone_node::exec
