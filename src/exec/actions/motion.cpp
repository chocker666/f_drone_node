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
    PlanningRequest request = motion_->request;

    if (motion_->type == ActionType::TAKEOFF &&
        !motion_->resumed_target) {
        request.planner_type = PlannerType::FIXED;
        request.frame_id = "map";

        request.target_position = Eigen::Vector3d(
            position.x,
            position.y,
            position.z + motion_->height);

        request.target_yaw = measuredYaw(raw.attitude.message);
        request.yaw_direction = 0;
        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }
    else if (motion_->type == ActionType::LAND &&
             !motion_->resumed_target) {
        if (position.z <= LAND_TARGET_Z) {
            finishMotion(
                ActionState::FAILED,
                ErrorCode::STATE_REJECTED,
                "Land requires current map z above -1.0");
            return;
        }

        request.planner_type = PlannerType::FIXED;
        request.frame_id = "map";

        request.target_position = Eigen::Vector3d(
            position.x,
            position.y,
            LAND_TARGET_Z);

        request.target_yaw = measuredYaw(raw.attitude.message);
        request.yaw_direction = 0;
        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }
    else if (motion_->is_hover && !motion_->resumed_target) {
        request.planner_type = PlannerType::FIXED;
        request.frame_id = "map";

        request.target_position = Eigen::Vector3d(
            position.x,
            position.y,
            position.z);

        request.target_yaw = measuredYaw(raw.attitude.message);
        request.yaw_direction = 0;
        request.controller_id = "position_yaw_pid";
        request.config_name = "default";
    }

    const auto planning = planner_.plan(request);

    if (!planning.success()) {
        finishMotion(
            ActionState::FAILED,
            ErrorCode::STATE_REJECTED,
            "Motion planning failed: " + planning.reason);
        return;
    }

    const double initial_distance = std::hypot(
        request.target_position.x() - position.x,
        request.target_position.y() - position.y,
        request.target_position.z() - position.z);

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
    motion_->motion = planning.motion;

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
        request.target_position.x(),
        request.target_position.y(),
        request.target_position.z(),
        request.target_yaw);
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
    const auto& target = motion_->motion.target_position;

    const double position_error = std::hypot(
        p.x - target.x(),
        p.y - target.y(),
        p.z - target.z());

    const double speed = std::hypot(v.x, v.y, v.z);

    const double delta =
        measuredYaw(raw.attitude.message) -
        motion_->motion.target_yaw;

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
    const auto& target = motion_->motion.target_position;

    // -1.0 是触发下降的参考点，不要求飞机实际到达地下高度。
    const double horizontal_error =
        std::hypot(p.x - target.x(), p.y - target.y());

    const double speed = std::hypot(v.x, v.y, v.z);

    const double delta =
        measuredYaw(raw.attitude.message) -
        motion_->motion.target_yaw;

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
                const auto& target = motion_->motion.target_position;
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
