#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "drone_node/ctrl/flight_controller.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using Status = drone_node::msg::ControlStatus;
using Command = drone_node::msg::ControlCommand;
using Trajectory = drone_node::msg::ControlTrajectory;
using Setpoint = drone_node::msg::Setpoint;
using PointStamped = geometry_msgs::msg::PointStamped;
using VectorStamped = geometry_msgs::msg::Vector3Stamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;

constexpr double PI = 3.14159265358979323846;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class Fixture
{
public:
    Fixture()
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("control_frequency", 100.0),
            rclcpp::Parameter("status_frequency", 20.0),
            rclcpp::Parameter("state_timeout", 0.3),
            rclcpp::Parameter("hover.settle_duration", 0.12),
            rclcpp::Parameter(
                "controllers.position_yaw_pid.config_names",
                std::vector<std::string>{"default", "slow"}),
            rclcpp::Parameter(
                "controllers.position_yaw_pid.configs.slow."
                "position_pid.max_output",
                0.2),
            rclcpp::Parameter(
                "controllers.position_yaw_pid.configs.slow."
                "position_pid.max_output_rate",
                0.4)
        });

        controller_ = std::make_shared<FlightController>(options);
        peer_ = std::make_shared<rclcpp::Node>("ctrl_integration_peer");

        position_pub_ = peer_->create_publisher<PointStamped>(
            "/px4_interface/out/position", rclcpp::QoS(1).reliable());

        velocity_pub_ = peer_->create_publisher<VectorStamped>(
            "/px4_interface/out/velocity", rclcpp::QoS(1).reliable());

        attitude_pub_ = peer_->create_publisher<QuaternionStamped>(
            "/px4_interface/out/attitude", rclcpp::QoS(1).reliable());

        trajectory_pub_ = peer_->create_publisher<Trajectory>(
            "/flight_controller/in/trajectory", rclcpp::QoS(1).reliable());

        command_pub_ = peer_->create_publisher<Command>(
            "/flight_controller/in/command", rclcpp::QoS(10).reliable());

        status_sub_ = peer_->create_subscription<Status>(
            "/flight_controller/out/control_status",
            rclcpp::QoS(10).reliable(),
            [this](const Status::SharedPtr message) {
                status = *message;
                ++status_count;
            });

        setpoint_sub_ = peer_->create_subscription<Setpoint>(
            "/px4_interface/in/setpoint",
            rclcpp::QoS(100).reliable(),
            [this](const Setpoint::SharedPtr message) {
                setpoint = *message;
                ++setpoint_count;
            });

        executor_.add_node(controller_);
        executor_.add_node(peer_);

        wait_for(
            [&] {
                return
                    position_pub_->get_subscription_count() > 0 &&
                    velocity_pub_->get_subscription_count() > 0 &&
                    attitude_pub_->get_subscription_count() > 0 &&
                    trajectory_pub_->get_subscription_count() > 0 &&
                    command_pub_->get_subscription_count() > 0 &&
                    status_sub_->get_publisher_count() > 0 &&
                    setpoint_sub_->get_publisher_count() > 0;
            },
            "ROS endpoints were not discovered",
            5.0);

        pump(0.3);
    }

    ~Fixture()
    {
        executor_.remove_node(controller_);
        executor_.remove_node(peer_);
    }

    void pump(double seconds)
    {
        const auto deadline =
            Clock::now() + std::chrono::duration<double>(seconds);

        while (Clock::now() < deadline) {
            const auto current = Clock::now();

            if (current >= next_feedback_) {
                publish_feedback();
                next_feedback_ = current + std::chrono::milliseconds(10);
            }

            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void wait_for(
        const std::function<bool()>& predicate,
        const char* message,
        double timeout = 2.0)
    {
        const auto deadline =
            Clock::now() + std::chrono::duration<double>(timeout);

        while (!predicate() && Clock::now() < deadline) {
            pump(0.01);
        }

        require(predicate(), message);
    }

    void point(
        uint64_t id,
        double x,
        const std::string& config = "default",
        const std::string& controller = "position_yaw_pid")
    {
        Trajectory message;
        message.trajectory_id = id;
        message.frame_id = "map";
        message.trajectory_type = Trajectory::POINT;
        message.controller_id = controller;
        message.config_name = config;
        message.point_params.position.x = x;
        message.point_params.yaw = 0.0;
        trajectory_pub_->publish(message);
    }

    void short_list(uint64_t id)
    {
        Trajectory message;
        message.trajectory_id = id;
        message.frame_id = "map";
        message.trajectory_type = Trajectory::LIST;
        message.controller_id = "position_yaw_pid";
        message.config_name = "default";

        drone_node::msg::FixedTrajectory first;
        first.position.x = feedback_x;

        auto last = first;
        last.position.x += 0.01;

        message.list_params.points = {first, last};
        message.list_params.max_speed = 1.0;
        message.list_params.max_acceleration = 1.0;
        message.list_params.max_yaw_rate = 1.0;
        message.list_params.max_yaw_acceleration = 1.0;

        trajectory_pub_->publish(message);
    }

    void command(uint8_t value)
    {
        Command message;
        message.command = value;
        command_pub_->publish(message);
    }

    void wait_tracking(uint64_t id)
    {
        wait_for(
            [&] {
                return
                    status.state == Status::TRACKING &&
                    status.trajectory_id == id;
            },
            "Expected TRACKING trajectory was not activated");
    }

    void require_no_setpoints()
    {
        // Drain messages already in transport before counting.
        pump(0.15);
        const auto previous = setpoint_count;
        pump(0.2);

        require(
            setpoint_count == previous,
            "Setpoints continued in IDLE or FAULT");
    }

    Status status{};
    Setpoint setpoint{};
    std::size_t status_count{0};
    std::size_t setpoint_count{0};

    double feedback_x{0.0};
    double feedback_vx{0.0};
    double feedback_yaw{0.0};
    bool velocity_enabled{true};

private:
    void publish_feedback()
    {
        const auto stamp = peer_->now();

        PointStamped position;
        position.header.stamp = stamp;
        position.header.frame_id = "map";
        position.point.x = feedback_x;
        position_pub_->publish(position);

        if (velocity_enabled) {
            VectorStamped velocity;
            velocity.header.stamp = stamp;
            velocity.header.frame_id = "map";
            velocity.vector.x = feedback_vx;
            velocity_pub_->publish(velocity);
        }

        QuaternionStamped attitude;
        attitude.header.stamp = stamp;
        attitude.header.frame_id = "map";
        attitude.quaternion.w = std::cos(feedback_yaw / 2.0);
        attitude.quaternion.z = std::sin(feedback_yaw / 2.0);
        attitude_pub_->publish(attitude);
    }

    std::shared_ptr<FlightController> controller_;
    std::shared_ptr<rclcpp::Node> peer_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    Clock::time_point next_feedback_{};

    rclcpp::Publisher<PointStamped>::SharedPtr position_pub_;
    rclcpp::Publisher<VectorStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<QuaternionStamped>::SharedPtr attitude_pub_;
    rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<Command>::SharedPtr command_pub_;

    rclcpp::Subscription<Status>::SharedPtr status_sub_;
    rclcpp::Subscription<Setpoint>::SharedPtr setpoint_sub_;
};

void test_invalid_configuration()
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter(
            "controllers.position_yaw_pid.configs.default."
            "position_pid.max_output_rate",
            0.0)
    });

    bool rejected = false;

    try {
        auto invalid = std::make_shared<FlightController>(options);
        (void)invalid;
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    require(rejected, "Invalid PID profile was accepted at startup");
}

void test_node()
{
    Fixture fixture;

    require(fixture.status_count > 0, "No ControlStatus received");
    require(fixture.status.state == Status::IDLE, "Initial state not IDLE");
    require(fixture.setpoint_count == 0, "IDLE published Setpoints");

    // No arming or output-enable publisher exists in this test.
    fixture.point(1, 100.0, "slow");
    fixture.wait_tracking(1);
    fixture.pump(0.8);

    require(
        fixture.setpoint_count > 0 &&
        fixture.setpoint.control_mode == Setpoint::VELOCITY,
        "TRACKING did not publish velocity Setpoints");

    require(
        fixture.setpoint.velocity.x > 0.18 &&
        fixture.setpoint.velocity.x <= 0.200001,
        "Named slow profile was not applied");

    require(
        std::abs(fixture.status.position_error - 100.0) < 1e-6,
        "Position tracking error was not published");

    fixture.point(2, -100.0, "missing");
    fixture.pump(0.2);

    require(
        fixture.status.state == Status::TRACKING &&
        fixture.status.trajectory_id == 1 &&
        fixture.setpoint.velocity.x > 0.0,
        "Unknown profile replaced the active trajectory");

    fixture.point(3, -100.0, "default", "missing");
    fixture.pump(0.2);

    require(
        fixture.status.state == Status::TRACKING &&
        fixture.status.trajectory_id == 1,
        "Unknown controller replaced the active trajectory");

    fixture.feedback_x = 3.0;
    fixture.feedback_vx = 1.0;
    fixture.pump(0.06);
    fixture.command(Command::CANCEL);

    fixture.wait_for(
        [&] { return fixture.status.state == Status::HOVER; },
        "CANCEL did not enter HOVER");

    fixture.feedback_x = 4.0;
    fixture.pump(0.1);

    require(
        std::abs(fixture.setpoint.velocity.x) < 0.03,
        "Braking pulled toward the CANCEL position");

    fixture.feedback_x = 5.0;
    fixture.feedback_vx = 0.0;
    fixture.pump(0.35);

    // The hold point should now be 5, rather than CANCEL position 3.
    fixture.feedback_x = 6.0;
    fixture.pump(0.35);

    require(
        fixture.setpoint.velocity.x < -0.03 &&
        std::abs(fixture.setpoint.velocity.x) <= 0.200001,
        "HOVER failed to hold the settled position or retain the profile");

    // New trajectories are accepted from HOVER and select a new profile.
    fixture.point(4, -100.0, "default");
    fixture.wait_tracking(4);

    fixture.wait_for(
        [&] { return fixture.setpoint.velocity.x < -0.25; },
        "Default profile did not replace slow profile");

    fixture.command(Command::EXIT_IDLE);
    fixture.wait_for(
        [&] { return fixture.status.state == Status::IDLE; },
        "EXIT_IDLE required arming feedback");

    fixture.require_no_setpoints();

    require(
        std::isnan(fixture.status.position_error) &&
        std::isnan(fixture.status.yaw_error),
        "IDLE errors are not NaN");

    fixture.point(5, 7.0);
    fixture.wait_tracking(5);

    // Position and attitude continue; only velocity becomes stale.
    fixture.velocity_enabled = false;

    fixture.wait_for(
        [&] { return fixture.status.state == Status::FAULT; },
        "Velocity timeout did not enter FAULT");

    require(
        fixture.status.fault_code == Status::FAULT_STATE_TIMEOUT,
        "Incorrect feedback timeout fault");

    fixture.require_no_setpoints();

    fixture.velocity_enabled = true;
    fixture.pump(0.1);

    fixture.point(6, 7.0);
    fixture.pump(0.2);

    require(
        fixture.status.state == Status::FAULT,
        "Ordinary trajectory cleared FAULT");

    fixture.command(Command::RESET_FAULT);
    fixture.wait_for(
        [&] { return fixture.status.state == Status::IDLE; },
        "RESET_FAULT required arming feedback");

    fixture.point(7, 7.0);
    fixture.wait_tracking(7);

    fixture.short_list(8);
    fixture.wait_tracking(8);
    fixture.pump(0.6);

    require(
        fixture.status.state == Status::TRACKING &&
        fixture.status.trajectory_id == 8,
        "Finite trajectory automatically left TRACKING");

    require(
        std::abs(fixture.status.position_error - 0.01) < 1e-6,
        "Finite trajectory endpoint was not held");

    // An ambiguous pi jump must fault rather than silently unwrap.
    fixture.feedback_yaw = PI;

    fixture.wait_for(
        [&] { return fixture.status.state == Status::FAULT; },
        "Ambiguous yaw jump did not enter FAULT");

    require(
        fixture.status.fault_code == Status::FAULT_INVALID_STATE,
        "Incorrect yaw discontinuity fault");

    fixture.require_no_setpoints();
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;

    try {
        test_invalid_configuration();
        test_node();
        std::cout << "FlightController integration tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }

    rclcpp::shutdown();
    return result;
}