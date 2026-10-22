#include <cmath>
#include <iostream>
#include <stdexcept>

#include "drone_node/ctrl/controller/controller_factory.hpp"

namespace
{

using namespace drone_node;
using namespace drone_node::controller;

constexpr double PI = 3.14159265358979323846;
constexpr double EPS = 1e-11;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

PhysicalControllerInput valid_input()
{
    PhysicalControllerInput input;
    input.dt = 0.01;

    input.state.position_feedback.valid = true;
    input.state.velocity_feedback.valid = true;
    input.state.attitude_feedback.valid = true;
    input.state.yaw_valid = true;

    input.reference_validity = {true, true, true, true, true};
    return input;
}

void test_replay(algorithm::PIDLimitMode mode)
{
    PositionYawPIDConfig config;

    config.position.kp = 0.8;
    config.position.ki = 0.35;
    config.position.kd = 0.2;
    config.position.max_output = 0.7;
    config.position.max_output_rate = 0.9;
    config.position.integral_limit = 0.4;
    config.position.limit_mode = mode;

    config.yaw = config.position;
    config.yaw.limit_mode = algorithm::PIDLimitMode::PER_AXIS;

    auto controller = ControllerFactory::create(
        "position_yaw_pid", config);

    // Baseline: the original separate PID objects and error wiring.
    algorithm::PIDController<3> legacy_position(config.position);
    algorithm::PIDController<1> legacy_yaw(config.yaw);

    auto initial = valid_input();
    controller->initialize(initial, YawDirection::SHORTEST);

    Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
    double previous_yaw_rate = 0.0;

    for (int i = 0; i < 1200; ++i) {
        if (i == 600) {
            controller->reset();
            controller->initialize(initial, YawDirection::SHORTEST);
            legacy_position.reset();
            legacy_yaw.reset();
            previous_velocity.setZero();
            previous_yaw_rate = 0.0;
        }

        auto input = valid_input();
        input.dt = i % 3 == 0 ? 0.004 : (i % 3 == 1 ? 0.01 : 0.02);

        const double t = i * 0.017;
        const double scale = i < 400 ? 8.0 : 0.2;

        input.reference.position = Eigen::Vector3d(
            scale * std::sin(t),
            scale * std::cos(0.7 * t),
            scale * std::sin(0.4 * t));

        input.state.position = Eigen::Vector3d(
            0.1 * std::cos(t), 0.2 * std::sin(t), -0.1);

        input.reference.velocity = Eigen::Vector3d(0.2, -0.1, 0.3);
        input.state.velocity = Eigen::Vector3d(
            std::sin(t), std::cos(t), 0.1);

        input.reference.yaw = scale * std::sin(0.3 * t);
        input.reference.yaw_rate = 0.2 * std::cos(t);
        input.state.unwrapped_yaw = 0.1 * std::sin(t);
        input.state.measured_yaw_rate = 0.3 * std::cos(t);

        algorithm::ControllerInput<3> position_input;
        position_input.error =
            input.reference.position - input.state.position;
        position_input.error_rate =
            input.reference.velocity - input.state.velocity;
        position_input.dt = input.dt;

        algorithm::ControllerInput<1> yaw_input;
        yaw_input.error(0) =
            input.reference.yaw - input.state.unwrapped_yaw;
        yaw_input.error_rate(0) =
            input.reference.yaw_rate - input.state.measured_yaw_rate;
        yaw_input.dt = input.dt;

        const auto expected_position =
            legacy_position.update(position_input);
        const auto expected_yaw =
            legacy_yaw.update(yaw_input);
        const auto actual = controller->update(input);

        require(
            actual.output.mode == ControlMode::VELOCITY,
            "Incorrect output mode");

        require(
            (actual.output.velocity -
             expected_position.command).norm() < EPS,
            "Position PID replay mismatch");

        require(
            std::abs(actual.output.yaw_rate -
                     expected_yaw.command(0)) < EPS,
            "Yaw PID replay mismatch");

        require(
            std::abs(actual.errors.position_error -
                     position_input.error.norm()) < EPS,
            "Position error mismatch");

        require(
            std::abs(actual.errors.yaw_error -
                     yaw_input.error(0)) < EPS,
            "Yaw error mismatch");

        const Eigen::Vector3d delta =
            actual.output.velocity - previous_velocity;

        if (mode == algorithm::PIDLimitMode::NORM) {
            require(
                actual.output.velocity.norm() <=
                    config.position.max_output + EPS,
                "Position norm limit exceeded");

            require(
                delta.norm() <=
                    config.position.max_output_rate * input.dt + EPS,
                "Position norm rate limit exceeded");
        } else {
            require(
                actual.output.velocity.cwiseAbs().maxCoeff() <=
                    config.position.max_output + EPS,
                "Position axis limit exceeded");

            require(
                delta.cwiseAbs().maxCoeff() <=
                    config.position.max_output_rate * input.dt + EPS,
                "Position axis rate limit exceeded");
        }

        require(
            std::abs(actual.output.yaw_rate) <=
                config.yaw.max_output + EPS,
            "Yaw amplitude limit exceeded");

        require(
            std::abs(actual.output.yaw_rate - previous_yaw_rate) <=
                config.yaw.max_output_rate * input.dt + EPS,
            "Yaw rate-of-output limit exceeded");

        previous_velocity = actual.output.velocity;
        previous_yaw_rate = actual.output.yaw_rate;
    }
}

void test_yaw_direction()
{
    struct Case
    {
        double current;
        double target;
        YawDirection direction;
        double expected_error;
    };

    const double degree = PI / 180.0;

    const Case cases[] = {
        {179 * degree, -179 * degree, YawDirection::SHORTEST, 2 * degree},
        {179 * degree, -179 * degree, YawDirection::CW, -358 * degree},
        {-179 * degree, 179 * degree, YawDirection::CCW, 358 * degree},
        {0.0, -PI, YawDirection::SHORTEST, PI},
        {0.0, PI, YawDirection::CW, -PI},
        {0.0, PI, YawDirection::CCW, PI}
    };

    for (const auto& test : cases) {
        PositionYawPIDController controller(PositionYawPIDConfig{});
        auto input = valid_input();
        input.state.unwrapped_yaw = test.current;
        input.reference.yaw = test.target;

        controller.initialize(input, test.direction);
        const auto result = controller.update(input);

        require(
            std::abs(result.errors.yaw_error -
                     test.expected_error) < EPS,
            "Yaw direction or +/-pi handling mismatch");
    }
}

void test_rejection()
{
    bool rejected = false;

    try {
        ControllerFactory::create("unknown", PositionYawPIDConfig{});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    require(rejected, "Unknown controller was accepted");

    PositionYawPIDController controller(PositionYawPIDConfig{});
    auto input = valid_input();
    controller.initialize(input, YawDirection::SHORTEST);

    input.state.velocity_feedback.valid = false;
    rejected = false;

    try {
        controller.update(input);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    require(rejected, "Unavailable velocity was accepted");

    input = valid_input();
    input.dt = 0.0;
    rejected = false;

    try {
        controller.update(input);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    require(rejected, "Invalid dt was accepted");
}

}  // namespace

int main()
{
    try {
        test_replay(algorithm::PIDLimitMode::NORM);
        test_replay(algorithm::PIDLimitMode::PER_AXIS);
        test_yaw_direction();
        test_rejection();
        std::cout << "PositionYawPIDController tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}