#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "drone_node/ctrl/trajectory_manager.hpp"
#include "drone_node/ctrl/trajectory_sampler/fixed_sampler.hpp"

namespace
{

using namespace drone_node;

constexpr double EPS = 1e-12;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

TrajectoryManager::ControlTrajectory point(
    uint64_t id,
    double x,
    double yaw)
{
    TrajectoryManager::ControlTrajectory msg;

    msg.trajectory_id = id;
    msg.frame_id = "map";
    msg.trajectory_type =
        TrajectoryManager::ControlTrajectory::POINT;
    msg.controller_id = "position_yaw_pid";
    msg.config_name = "default";

    msg.point_params.position.x = x;
    msg.point_params.position.y = 2.0;
    msg.point_params.position.z = 3.0;
    msg.point_params.yaw = yaw;
    msg.point_params.yaw_direction =
        drone_node::msg::FixedTrajectory::SHORTEST;

    return msg;
}

void check_reference(
    const ReferenceState& reference,
    double x,
    double yaw)
{
    require(
        std::abs(reference.position.x() - x) < EPS,
        "Incorrect POINT position");

    require(
        std::abs(reference.yaw - yaw) < EPS,
        "Incorrect POINT yaw");

    require(
        reference.velocity.isZero(0.0) &&
        reference.acceleration.isZero(0.0) &&
        reference.yaw_rate == 0.0,
        "POINT reference contains motion");
}

void test_point_sampler()
{
    const auto msg = point(1, 4.0, 0.7);
    trajectory_sampler::PointSampler sampler(msg.point_params);

    require(
        std::isinf(sampler.duration()) && sampler.duration() > 0.0,
        "POINT duration is not positive infinity");

    check_reference(sampler.sample(0.0), 4.0, 0.7);
    check_reference(sampler.sample(1e9), 4.0, 0.7);

    bool rejected = false;

    try {
        sampler.sample(-0.1);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    require(rejected, "Negative sample time was accepted");
}

void test_manager()
{
    TrajectoryManager manager;
    std::string reason;

    const SteadyTime t0 =
        SteadyTime{} + std::chrono::seconds(10);
    const SteadyTime t1 = t0 + std::chrono::seconds(2);

    require(
        manager.accept(point(1, 4.0, 0.7), t0, reason) ==
            TrajectoryManager::AcceptResult::ACCEPTED,
        "Initial POINT rejected");

    require(
        manager.has_trajectory() &&
        manager.is_started(t0) &&
        manager.elapsed(t0) == 0.0,
        "POINT did not activate immediately");

    check_reference(manager.sample(t0), 4.0, 0.7);

    require(
        std::abs(manager.elapsed(t1) - 2.0) < EPS,
        "Incorrect monotonic elapsed time");

    require(
        manager.accept(point(2, -8.0, -0.4), t1, reason) ==
            TrajectoryManager::AcceptResult::ACCEPTED,
        "Replacement POINT rejected");

    require(
        manager.trajectory_id() == 2 &&
        manager.elapsed(t1) == 0.0,
        "Replacement did not reset activation time");

    // The new target is visible immediately, without blending.
    check_reference(manager.sample(t1), -8.0, -0.4);

    auto assert_rejected_without_replacement =
        [&](const TrajectoryManager::ControlTrajectory& invalid)
        {
            require(
                manager.accept(
                    invalid,
                    t1 + std::chrono::seconds(1),
                    reason) ==
                    TrajectoryManager::AcceptResult::REJECTED,
                "Invalid trajectory accepted");

            require(
                manager.trajectory_id() == 2 &&
                std::abs(manager.elapsed(t1)) < EPS,
                "Rejected trajectory changed active metadata");

            check_reference(manager.sample(t1), -8.0, -0.4);
        };

    auto invalid = point(3, 0.0, 0.0);
    invalid.point_params.position.x =
        std::numeric_limits<double>::quiet_NaN();
    assert_rejected_without_replacement(invalid);

    invalid = point(3, 0.0, 0.0);
    invalid.point_params.yaw_direction = 255;
    assert_rejected_without_replacement(invalid);

    invalid = point(3, 0.0, 0.0);
    invalid.frame_id = "unsupported";
    assert_rejected_without_replacement(invalid);

    invalid = point(3, 0.0, 0.0);
    invalid.controller_id.clear();
    assert_rejected_without_replacement(invalid);

    invalid = point(3, 0.0, 0.0);
    invalid.config_name.clear();
    assert_rejected_without_replacement(invalid);

    bool rejected = false;

    try {
        manager.sample(t1 - std::chrono::milliseconds(1));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    require(rejected, "Pre-activation sampling was accepted");

    manager.clear();

    require(
        !manager.has_trajectory() &&
        manager.trajectory_id() == 0,
        "Clear did not remove active trajectory");

    rejected = false;

    try {
        manager.sample(t1);
    } catch (const std::logic_error&) {
        rejected = true;
    }

    require(rejected, "Sampling after clear was accepted");
}

}  // namespace

int main()
{
    try {
        test_point_sampler();
        test_manager();
        std::cout << "TrajectoryManager tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}