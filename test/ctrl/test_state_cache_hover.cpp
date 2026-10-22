#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "drone_node/ctrl/state_cache.hpp"
#include "drone_node/ctrl/hover_reference.hpp"

namespace
{

using namespace drone_node;

constexpr double PI = 3.14159265358979323846;
constexpr double EPS = 1e-9;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Eigen::Quaterniond attitude(double yaw)
{
    return Eigen::Quaterniond(
        std::cos(yaw / 2.0),
        0.0,
        0.0,
        std::sin(yaw / 2.0));
}

void test_state_cache()
{
    StateCache cache(0.2);
    const SteadyTime t0{};

    require(
        !StateCache::usable(cache.snapshot(t0)),
        "Empty cache is usable");

    cache.update_position(Eigen::Vector3d(1.0, 2.0, 3.0), t0);
    cache.update_velocity(Eigen::Vector3d::Zero(), t0);

    require(
        !StateCache::usable(cache.snapshot(t0)),
        "Missing attitude was accepted");

    auto q = attitude(179.0 * PI / 180.0);
    q.coeffs() *= 3.0;

    require(
        cache.update_attitude(q, t0) ==
            StateCache::AttitudeResult::ACCEPTED,
        "Valid quaternion rejected");

    auto state = cache.snapshot(t0);

    require(StateCache::usable(state), "Complete cache unusable");
    require(
        std::abs(state.orientation.norm() - 1.0) < EPS,
        "Quaternion not normalized");

    const auto t1 = t0 + std::chrono::milliseconds(10);

    cache.update_attitude(
        attitude(-179.0 * PI / 180.0), t1);

    state = cache.snapshot(t1);

    require(
        std::abs(state.unwrapped_yaw - 181.0 * PI / 180.0) < EPS,
        "Yaw unwrap failed");

    require(
        std::abs(
            state.measured_yaw_rate -
            (2.0 * PI / 180.0) / 0.01) < EPS,
        "Yaw rate estimate failed");

    require(
        cache.update_attitude(
            Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0), t1) ==
            StateCache::AttitudeResult::REJECTED,
        "Zero quaternion accepted");

    Eigen::Vector3d invalid = Eigen::Vector3d::Zero();
    invalid.x() = std::numeric_limits<double>::quiet_NaN();

    require(
        !cache.update_position(invalid, t1),
        "Non-finite position accepted");

    state = cache.snapshot(t1);

    require(
        state.position.x() == 1.0,
        "Invalid feedback replaced valid position");

    const auto t2 = t0 + std::chrono::milliseconds(20);

    require(
        cache.update_attitude(
            attitude(1.0 * PI / 180.0), t2) ==
            StateCache::AttitudeResult::YAW_DISCONTINUITY,
        "Ambiguous pi displacement accepted");

    require(
        !cache.snapshot(t2).yaw_valid,
        "Discontinuous yaw remained valid");

    cache.update_attitude(
        attitude(2.0 * PI / 180.0),
        t0 + std::chrono::milliseconds(30));

    require(
        cache.snapshot(
            t0 + std::chrono::milliseconds(30)).yaw_valid,
        "Yaw estimator did not reinitialize");

    const auto expired = cache.snapshot(
        t0 + std::chrono::milliseconds(250));

    require(
        !expired.position_feedback.valid &&
        !expired.velocity_feedback.valid &&
        !expired.attitude_feedback.valid &&
        !expired.yaw_valid,
        "Feedback timeout not enforced");
}

StateSnapshot hover_state(
    SteadyTime time,
    double position,
    double speed,
    double yaw_rate = 0.0)
{
    StateSnapshot state;
    state.captured_at = time;
    state.position.x() = position;
    state.velocity.x() = speed;
    state.measured_yaw_rate = yaw_rate;

    state.position_feedback.valid = true;
    state.velocity_feedback.valid = true;
    state.attitude_feedback.valid = true;
    state.yaw_valid = true;
    return state;
}

void test_hover()
{
    HoverReference::Config config;
    config.speed_threshold = 0.1;
    config.yaw_rate_threshold = 0.1;
    config.settle_duration = 0.3;

    HoverReference hover(config);
    const SteadyTime t0{};

    hover.start(hover_state(t0, 0.0, 1.0));

    auto reference = hover.sample(hover_state(
        t0 + std::chrono::milliseconds(100),
        1.0, 0.5));

    require(
        hover.phase() == HoverReference::Phase::BRAKING,
        "HOVER did not brake");

    require(
        reference.position.x() == 1.0 &&
        reference.velocity.isZero(0.0),
        "Braking pulled toward CANCEL position");

    hover.sample(hover_state(
        t0 + std::chrono::milliseconds(200),
        1.5, 0.05));

    require(
        hover.phase() == HoverReference::Phase::LOW_SPEED,
        "Low-speed confirmation not started");

    // Rotation is still too fast: confirmation must restart.
    hover.sample(hover_state(
        t0 + std::chrono::milliseconds(300),
        1.6, 0.05, 0.2));

    require(
        hover.phase() == HoverReference::Phase::BRAKING,
        "High yaw rate did not interrupt confirmation");

    hover.sample(hover_state(
        t0 + std::chrono::milliseconds(400),
        1.8, 0.05));

    reference = hover.sample(hover_state(
        t0 + std::chrono::milliseconds(750),
        2.0, 0.02));

    require(
        hover.phase() == HoverReference::Phase::HOLD,
        "Low-speed dwell did not enter HOLD");

    require(
        reference.position.x() == 2.0,
        "HOLD did not capture deceleration-end position");

    reference = hover.sample(hover_state(
        t0 + std::chrono::milliseconds(900),
        9.0, 0.5));

    require(
        reference.position.x() == 2.0,
        "HOLD target followed actual position");
}

}  // namespace

int main()
{
    try {
        test_state_cache();
        test_hover();
        std::cout << "StateCache/HOVER tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}