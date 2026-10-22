#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "drone_node/ctrl/trajectory_sampler/discrete_sampler.hpp"
#include "drone_node/ctrl/trajectory_manager.hpp"

namespace
{

using namespace drone_node;
using trajectory_sampler::ListSampler;

constexpr double PI = 3.14159265358979323846;
constexpr double EPS = 1e-9;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char* message)
{
    require(std::abs(actual - expected) < EPS, message);
}

msg::FixedTrajectory point(
    double x,
    double y = 0.0,
    double yaw = 0.0)
{
    msg::FixedTrajectory result;
    result.position.x = x;
    result.position.y = y;
    result.yaw = yaw;
    result.yaw_direction = msg::FixedTrajectory::SHORTEST;
    return result;
}

msg::ListTrajectory path()
{
    msg::ListTrajectory result;
    result.target_speed = 0.0;
    result.max_speed = 2.0;
    result.max_acceleration = 1.0;
    result.max_yaw_rate = 2.0;
    result.max_yaw_acceleration = 1.0;
    return result;
}

void test_profiles()
{
    auto input = path();
    input.points = {point(0.0), point(10.0)};

    ListSampler trapezoid(input);

    near(trapezoid.duration(), 7.0, "Trapezoid duration");
    near(trapezoid.sample(1.0).position.x(), 0.5, "Acceleration position");
    near(trapezoid.sample(1.0).velocity.x(), 1.0, "Acceleration speed");
    near(trapezoid.sample(3.0).velocity.x(), 2.0, "Cruise speed");
    near(trapezoid.sample(6.0).position.x(), 9.5, "Deceleration position");

    input.points = {point(0.0), point(1.0)};
    ListSampler triangle(input);

    near(triangle.duration(), 2.0, "Triangle duration");
    near(triangle.sample(1.0).position.x(), 0.5, "Triangle midpoint");
    near(triangle.sample(1.0).velocity.x(), 1.0, "Triangle peak speed");

    input = path();
    input.target_speed = 0.5;
    input.points = {point(0.0), point(10.0)};
    ListSampler requested_speed(input);

    near(
        requested_speed.sample(5.0).velocity.norm(),
        0.5,
        "Requested speed ignored");

    input.target_speed = 20.0;
    ListSampler capped_speed(input);

    near(
        capped_speed.sample(3.0).velocity.norm(),
        2.0,
        "Requested speed not capped");
}

void test_corners_and_collinearity()
{
    auto input = path();
    input.points = {point(0.0), point(5.0), point(10.0)};

    ListSampler straight(input);

    near(straight.duration(), 7.0, "Collinear segments not merged");
    near(straight.sample(3.5).position.x(), 5.0, "Collinear midpoint");
    near(straight.sample(3.5).velocity.x(), 2.0, "Stopped at collinear point");

    input.points = {
        point(0.0, 0.0),
        point(1.0, 0.0),
        point(1.0, 1.0)
    };

    ListSampler corner(input);

    near(corner.duration(), 4.0, "Corner duration");

    const auto at_corner = corner.sample(2.0);
    near(at_corner.position.x(), 1.0, "Corner x");
    near(at_corner.position.y(), 0.0, "Corner y");
    near(at_corner.velocity.norm(), 0.0, "Corner did not stop");

    input.points = {point(0.0), point(1.0), point(0.0)};
    ListSampler reversal(input);

    near(reversal.duration(), 4.0, "Reversal incorrectly merged");
    near(reversal.sample(2.0).velocity.norm(), 0.0, "Reversal did not stop");
}

void test_independent_yaw_and_hold()
{
    auto input = path();
    input.max_yaw_rate = 0.5;
    input.points = {point(0.0), point(1.0, 0.0, PI)};

    ListSampler slow_yaw(input);
    const auto translation_finished = slow_yaw.sample(2.0);

    near(translation_finished.position.x(), 1.0, "Translation waited for yaw");
    near(translation_finished.velocity.norm(), 0.0, "Translation endpoint speed");
    require(
        translation_finished.yaw > 0.0 &&
        translation_finished.yaw < PI &&
        translation_finished.yaw_rate > 0.0,
        "Yaw not independently moving");

    near(
        slow_yaw.duration(),
        2.0 * PI + 0.5,
        "Independent yaw duration");

    input = path();
    input.points = {point(0.0), point(10.0, 0.0, 1.0)};

    ListSampler fast_yaw(input);
    const auto yaw_finished = fast_yaw.sample(2.0);

    near(yaw_finished.yaw, 1.0, "Yaw waited for translation");
    near(yaw_finished.yaw_rate, 0.0, "Yaw endpoint rate");
    require(
        yaw_finished.position.x() < 10.0 &&
        yaw_finished.velocity.x() > 0.0,
        "Translation not independently moving");

    for (double time : {fast_yaw.duration(), 1e6}) {
        const auto held = fast_yaw.sample(time);
        near(held.position.x(), 10.0, "Endpoint position not held");
        near(held.yaw, 1.0, "Endpoint yaw not held");
        near(held.velocity.norm(), 0.0, "Endpoint velocity not zero");
        near(held.acceleration.norm(), 0.0, "Endpoint acceleration not zero");
        near(held.yaw_rate, 0.0, "Endpoint yaw rate not zero");
    }

    // Continuous yaw across +/-pi.
    const double degree = PI / 180.0;
    input = path();
    input.points = {
        point(0.0, 0.0, 179.0 * degree),
        point(0.0, 0.0, -179.0 * degree)
    };

    ListSampler shortest(input);
    near(
        shortest.sample(shortest.duration()).yaw,
        181.0 * degree,
        "Yaw was not unwrapped");

    input.points.back().yaw_direction = msg::FixedTrajectory::CW;
    ListSampler clockwise(input);

    near(
        clockwise.sample(clockwise.duration()).yaw,
        -179.0 * degree,
        "Clockwise direction ignored");
}

void test_limits()
{
    auto input = path();
    input.points = {
        point(0.0, 0.0, 0.0),
        point(10.0, 0.0, 1.0),
        point(10.0, 5.0, -1.0)
    };

    ListSampler sampler(input);

    const double dt = sampler.duration() / 10000.0;
    double previous_yaw_rate = sampler.sample(0.0).yaw_rate;

    for (int i = 1; i <= 10000; ++i) {
        const auto reference = sampler.sample(i * dt);

        require(
            reference.velocity.norm() <= input.max_speed + EPS,
            "Translation speed exceeded");

        require(
            reference.acceleration.norm() <= input.max_acceleration + EPS,
            "Translation acceleration exceeded");

        require(
            std::abs(reference.yaw_rate) <= input.max_yaw_rate + EPS,
            "Yaw rate exceeded");

        require(
            std::abs(reference.yaw_rate - previous_yaw_rate) <=
                input.max_yaw_acceleration * dt + EPS,
            "Yaw acceleration exceeded");

        previous_yaw_rate = reference.yaw_rate;
    }
}

void test_manager_replacement()
{
    TrajectoryManager manager;
    std::string reason;
    const SteadyTime origin{};

    msg::ControlTrajectory command;
    command.frame_id = "map";
    command.controller_id = "position_yaw_pid";
    command.config_name = "default";
    command.trajectory_type = msg::ControlTrajectory::LIST;
    command.trajectory_id = 1;
    command.list_params = path();
    command.list_params.points = {point(4.0), point(5.0)};

    require(
        manager.accept(command, origin, reason) ==
            TrajectoryManager::AcceptResult::ACCEPTED,
        "Valid LIST rejected");

    // No implicit segment from the vehicle position.
    near(manager.sample(origin).position.x(), 4.0, "LIST origin changed");

    command.trajectory_id = 2;
    command.list_params.points = {point(-8.0), point(-7.0)};

    require(
        manager.accept(command, origin, reason) ==
            TrajectoryManager::AcceptResult::ACCEPTED,
        "Replacement LIST rejected");

    near(manager.sample(origin).position.x(), -8.0, "LIST replacement blended");

    command.trajectory_id = 3;
    command.list_params.points.back().yaw =
        std::numeric_limits<double>::quiet_NaN();

    require(
        manager.accept(command, origin, reason) ==
            TrajectoryManager::AcceptResult::REJECTED,
        "Invalid LIST accepted");

    require(manager.trajectory_id() == 2, "Invalid LIST replaced active ID");
    near(manager.sample(origin).position.x(), -8.0, "Invalid LIST replaced path");

    command.list_params.points.clear();

    require(
        manager.accept(command, origin, reason) ==
            TrajectoryManager::AcceptResult::REJECTED,
        "Empty LIST accepted");
}

}  // namespace

int main()
{
    try {
        test_profiles();
        test_corners_and_collinearity();
        test_independent_yaw_and_hold();
        test_limits();
        test_manager_replacement();
        std::cout << "ListSampler tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}