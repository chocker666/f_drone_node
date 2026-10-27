#include "drone_node/inter/px4_interface.hpp"
#include "drone_node/inter/frame_transforms.hpp"
#include "drone_node/msg/battery.hpp"
#include "drone_node/msg/setpoint.hpp"
#include "drone_node/srv/set_arming.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

namespace ft = drone_node::frame_transforms;

namespace
{
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Setpoint = drone_node::msg::Setpoint;
using Arm = drone_node::srv::SetArming;
using Command = px4_msgs::srv::VehicleCommand;
using Status = px4_msgs::msg::VehicleStatus;
using Ack = px4_msgs::msg::VehicleCommandAck;
using Position = px4_msgs::msg::VehicleLocalPosition;
using Attitude = px4_msgs::msg::VehicleAttitude;
using Land = px4_msgs::msg::VehicleLandDetected;
using Battery = px4_msgs::msg::BatteryStatus;
using Trajectory = px4_msgs::msg::TrajectorySetpoint;
using AttitudeTarget = px4_msgs::msg::VehicleAttitudeSetpoint;
using RatesTarget = px4_msgs::msg::VehicleRatesSetpoint;
using Mode = px4_msgs::msg::OffboardControlMode;
using Bool = std_msgs::msg::Bool;
using PointStamped = geometry_msgs::msg::PointStamped;
using VectorStamped = geometry_msgs::msg::Vector3Stamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;

constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
constexpr float NAN_F = std::numeric_limits<float>::quiet_NaN();

double age(Time now, Time then)
{
    return std::chrono::duration<double>(now - then).count();
}

bool valid_number(double value)
{
    return std::isfinite(value) &&
           std::abs(value) <= std::numeric_limits<float>::max();
}

template<class V>
bool valid_vector(const V &value)
{
    return valid_number(value.x) &&
           valid_number(value.y) &&
           valid_number(value.z);
}

template<class V>
Eigen::Vector3d eigen_vector(const V &value)
{
    return {value.x, value.y, value.z};
}

template<class V>
std::array<float, 3> ned(const V &value)
{
    const auto result = ft::enu_to_ned_local_frame(eigen_vector(value));
    return {
        static_cast<float>(result.x()),
        static_cast<float>(result.y()),
        static_cast<float>(result.z())
    };
}

template<class V>
std::array<float, 3> frd(const V &value)
{
    const auto result =
        ft::baselink_to_aircraft_body_frame(eigen_vector(value));
    return {
        static_cast<float>(result.x()),
        static_cast<float>(result.y()),
        static_cast<float>(result.z())
    };
}

bool valid_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    const double norm2 =
        q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return std::isfinite(norm2) && norm2 > 1e-12;
}
}  // namespace

struct PX4Interface::Impl
{
    using Client = rclcpp::Client<Command>;
    using Header = std::shared_ptr<rmw_request_id_t>;

    PX4Interface &node;

    rclcpp::CallbackGroup::SharedPtr telemetry_group;
    rclcpp::CallbackGroup::SharedPtr control_group;
    rclcpp::CallbackGroup::SharedPtr command_group;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;

    rclcpp::Publisher<Bool>::SharedPtr arm_pub;
    rclcpp::Publisher<Bool>::SharedPtr connected_pub;
    rclcpp::Publisher<Bool>::SharedPtr landed_pub;
    rclcpp::Publisher<drone_node::msg::Battery>::SharedPtr battery_pub;
    rclcpp::Publisher<PointStamped>::SharedPtr position_pub;
    rclcpp::Publisher<VectorStamped>::SharedPtr velocity_pub;
    rclcpp::Publisher<QuaternionStamped>::SharedPtr attitude_pub;

    rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub;
    rclcpp::Publisher<AttitudeTarget>::SharedPtr attitude_target_pub;
    rclcpp::Publisher<RatesTarget>::SharedPtr rates_target_pub;
    rclcpp::Publisher<Mode>::SharedPtr mode_pub;

    Client::SharedPtr command_client;
    rclcpp::Service<Arm>::SharedPtr arm_service;

    std::string frame;
    std::array<bool, 5> enabled{};
    double connection_timeout;
    double setpoint_timeout;
    double command_timeout;
    double arm_timeout;

    Setpoint cached{};
    Time setpoint_received{};
    bool has_setpoint{false};
    uint8_t published_mode{Setpoint::VELOCITY};

    Time status_received{};
    bool has_status{false};
    bool px4_armed{false};
    bool reported_connected{false};
    uint64_t status_sequence{0};

    uint8_t px4_nav_state{0};
    bool px4_position_valid{false};

    Time offboard_heartbeat_started{};
    Time last_offboard_request{};

    double offboard_warmup;
    double offboard_retry_interval;

    bool offboard_requested{false};

    bool pending{false};
    bool desired_arm{false};
    bool reply_received{false};
    Time command_started{};
    uint64_t command_sequence{0};
    uint64_t generation{0};
    int64_t request_id{-1};
    Header upstream_header;

    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;
    uint8_t source_component;

    template<class T>
    T parameter(const std::string &name, const T &value)
    {
        return node.declare_parameter<T>(name, value);
    }

    double positive(const std::string &name, double value)
    {
        value = parameter<double>(name, value);
        if (!std::isfinite(value) || value <= 0.0)
            throw std::invalid_argument(name + " must be positive");
        return value;
    }

    uint8_t identifier(const std::string &name)
    {
        const auto value = parameter<int64_t>(name, 1);
        if (value < 1 || value > 255)
            throw std::invalid_argument(name + " must be in [1, 255]");
        return static_cast<uint8_t>(value);
    }

    template<class T>
    typename rclcpp::Publisher<T>::SharedPtr publisher(
        const std::string &name, const std::string &topic, size_t depth = 10)
    {
        return node.create_publisher<T>(
            parameter<std::string>(name, topic),
            rclcpp::QoS(depth).reliable());
    }

    template<class T, class Callback>
    void subscribe(
        const std::string &name, const std::string &topic,
        const rclcpp::QoS &qos,
        const rclcpp::CallbackGroup::SharedPtr &group,
        Callback callback)
    {
        rclcpp::SubscriptionOptions options;
        options.callback_group = group;
        subscriptions.push_back(node.create_subscription<T>(
            parameter<std::string>(name, topic),
            qos, std::move(callback), options));
    }

    template<class Callback>
    void timer(
        double rate, const rclcpp::CallbackGroup::SharedPtr &group,
        Callback callback)
    {
        const auto period = std::chrono::duration_cast<
            std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));
        if (period.count() <= 0)
            throw std::invalid_argument("timer period is too small");
        timers.push_back(node.create_wall_timer(
            period, std::move(callback), group));
    }

    explicit Impl(PX4Interface &owner) : node(owner)
    {
        using Group = rclcpp::CallbackGroupType;
        telemetry_group = node.create_callback_group(Group::MutuallyExclusive);
        control_group = node.create_callback_group(Group::MutuallyExclusive);
        command_group = node.create_callback_group(Group::MutuallyExclusive);

        frame = parameter<std::string>("frame_id", "map");
        if (frame.empty())
            throw std::invalid_argument("frame_id must not be empty");

        offboard_warmup = positive("offboard_warmup", 1.5);
        offboard_retry_interval = positive("offboard_retry_interval", 2.0);
        connection_timeout = positive("connection_timeout", 1.0);
        setpoint_timeout = positive("setpoint_timeout", 0.5);
        command_timeout = positive("command_timeout", 2.0);
        arm_timeout = positive("arm_timeout", 5.0);
        if (arm_timeout < command_timeout)
            throw std::invalid_argument("arm_timeout must be >= command_timeout");

        const double connection_rate = positive("connection_check_rate", 2.0);
        const double setpoint_rate = positive("setpoint_publish_rate", 50.0);
        const double heartbeat_rate = positive("offboard_heartbeat_rate", 20.0);

        const std::array<std::string, 5> names{
            "position", "velocity", "acceleration", "attitude", "body_rate"
        };
        for (size_t i = 0; i < names.size(); ++i)
            enabled[i] = parameter<bool>("control_modes." + names[i], i < 3);

        target_system = identifier("target_system");
        target_component = identifier("target_component");
        source_system = identifier("source_system");
        source_component = identifier("source_component");

        arm_pub = publisher<Bool>("arm_topic", "/px4_interface/out/arm");
        connected_pub = publisher<Bool>("connected_topic", "/px4_interface/out/connected");
        landed_pub = publisher<Bool>("landed_topic", "/px4_interface/out/landed");
        battery_pub = publisher<drone_node::msg::Battery>(
            "battery_topic", "/px4_interface/out/battery");
        position_pub = publisher<PointStamped>(
            "position_topic", "/px4_interface/out/position", 1);
        velocity_pub = publisher<VectorStamped>(
            "velocity_topic", "/px4_interface/out/velocity", 1);
        attitude_pub = publisher<QuaternionStamped>(
            "attitude_topic", "/px4_interface/out/attitude", 1);

        trajectory_pub = publisher<Trajectory>(
            "px4_trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint", 1);
        attitude_target_pub = publisher<AttitudeTarget>(
            "px4_attitude_setpoint_topic",
            "/fmu/in/vehicle_attitude_setpoint_v1", 1);
        rates_target_pub = publisher<RatesTarget>(
            "px4_rates_setpoint_topic", "/fmu/in/vehicle_rates_setpoint", 1);
        mode_pub = publisher<Mode>(
            "offboard_control_mode_topic", "/fmu/in/offboard_control_mode", 1);

        const auto sensor_qos = rclcpp::SensorDataQoS();
        subscribe<Status>(
            "vehicle_status_topic", "/fmu/out/vehicle_status_v1",
            sensor_qos, command_group,
            [this](Status::SharedPtr msg) { status_callback(*msg); });

        subscribe<Position>(
            "vehicle_local_position_topic",
            "/fmu/out/vehicle_local_position_v1",
            sensor_qos, telemetry_group,
            [this](Position::SharedPtr msg) { position_callback(*msg); });

        subscribe<Attitude>(
            "vehicle_attitude_topic", "/fmu/out/vehicle_attitude",
            sensor_qos, telemetry_group,
            [this](Attitude::SharedPtr msg) { attitude_callback(*msg); });

        subscribe<Land>(
            "vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected",
            sensor_qos, telemetry_group,
            [this](Land::SharedPtr msg) {
                Bool out;
                out.data = msg->landed;
                landed_pub->publish(out);
            });

        subscribe<Battery>(
            "battery_status_topic", "/fmu/out/battery_status_v1",
            sensor_qos, telemetry_group,
            [this](Battery::SharedPtr msg) {
                drone_node::msg::Battery out;
                out.voltage = msg->voltage_v;
                out.current = msg->current_a;
                out.current_a = msg->current_average_a;
                out.discharged = msg->discharged_mah;
                out.remaining = msg->remaining;
                battery_pub->publish(out);
            });

        subscribe<Setpoint>(
            "setpoint_topic", "/px4_interface/in/setpoint",
            rclcpp::QoS(1).reliable(), control_group,
            [this](Setpoint::SharedPtr msg) {
                if (!valid_setpoint(*msg)) {
                    RCLCPP_WARN_THROTTLE(
                        node.get_logger(), *node.get_clock(), 2000,
                        "Rejected invalid or disabled setpoint");
                    return;
                }
                cached = *msg;
                setpoint_received = Clock::now();
                has_setpoint = true;
            });

        command_client = node.create_client<Command>(
            parameter<std::string>(
                "vehicle_command_service", "/fmu/vehicle_command"),
            rmw_qos_profile_services_default, command_group);

        arm_service = node.create_service<Arm>(
            parameter<std::string>("arm_service", "/px4_interface/in/arm"),
            [this](Header header, Arm::Request::SharedPtr request) {
                arm_request(std::move(header), request->arm);
            },
            rmw_qos_profile_services_default, command_group);

        timer(2.0, command_group, [this] {manage_offboard(); });
        timer(connection_rate, command_group, [this] { publish_connection(); });
        timer(20.0, command_group, [this] { check_command(); });
        timer(setpoint_rate, control_group, [this] { publish_setpoint(); });
        timer(heartbeat_rate, control_group, [this] { publish_heartbeat(); });
    }

    uint64_t timestamp() const
    {
        const auto value = node.now().nanoseconds();
        return value > 0 ? static_cast<uint64_t>(value / 1000) : 0;
    }

    bool connected(Time now) const
    {
        return has_status && age(now, status_received) < connection_timeout;
    }

    void status_callback(const Status &msg)
    {
        status_received = Clock::now();
        has_status = true;
        ++status_sequence;
        px4_armed = msg.arming_state == Status::ARMING_STATE_ARMED;
        px4_nav_state = msg.nav_state;

        Bool out;
        out.data = connected(status_received) && px4_armed;
        arm_pub->publish(out);
    }

    void publish_connection()
    {
        const bool current = connected(Clock::now());
        Bool out;
        out.data = current;
        connected_pub->publish(out);

        if (reported_connected && !current) {
            out.data = false;
            arm_pub->publish(out);
        }
        reported_connected = current;
    }

    void position_callback(const Position &msg)
    {
        const auto stamp = node.now();

        if (msg.xy_valid && msg.z_valid &&
            valid_number(msg.x) && valid_number(msg.y) && valid_number(msg.z)) {
            const auto value = ft::ned_to_enu_local_frame(
                Eigen::Vector3d(msg.x, msg.y, msg.z));
            PointStamped out;
            out.header.stamp = stamp;
            out.header.frame_id = frame;
            out.point.x = value.x();
            out.point.y = value.y();
            out.point.z = value.z();
            position_pub->publish(out);
        }

        if (msg.v_xy_valid && msg.v_z_valid &&
            valid_number(msg.vx) && valid_number(msg.vy) &&
            valid_number(msg.vz)) {
            const auto value = ft::ned_to_enu_local_frame(
                Eigen::Vector3d(msg.vx, msg.vy, msg.vz));
            VectorStamped out;
            out.header.stamp = stamp;
            out.header.frame_id = frame;
            out.vector.x = value.x();
            out.vector.y = value.y();
            out.vector.z = value.z();
            velocity_pub->publish(out);
        }
        px4_position_valid = msg.xy_valid && msg.z_valid;
    }

    void attitude_callback(const Attitude &msg)
    {
        Eigen::Quaterniond q(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
        if (!std::isfinite(q.squaredNorm()) || q.squaredNorm() <= 1e-12)
            return;

        q = ft::px4_to_ros_orientation(q.normalized());
        QuaternionStamped out;
        out.header.stamp = node.now();
        out.header.frame_id = frame;
        out.quaternion.w = q.w();
        out.quaternion.x = q.x();
        out.quaternion.y = q.y();
        out.quaternion.z = q.z();
        attitude_pub->publish(out);
    }

    bool valid_setpoint(const Setpoint &msg) const
    {
        if (msg.control_mode >= enabled.size() || !enabled[msg.control_mode])
            return false;
        if (msg.stamp.sec < 0 || msg.stamp.nanosec >= 1000000000u)
            return false;

        switch (msg.control_mode) {
        case Setpoint::POSITION:
            return valid_vector(msg.position) && valid_number(msg.yaw);
        case Setpoint::VELOCITY:
            return valid_vector(msg.velocity) && valid_number(msg.yawspeed);
        case Setpoint::ACCELERATION:
            return valid_vector(msg.acceleration) &&
                valid_number(msg.yawspeed) &&
                (!msg.use_jerk || valid_vector(msg.jerk));
        case Setpoint::ATTITUDE:
        case Setpoint::BODY_RATE:
            if (!valid_vector(msg.thrust) ||
                std::abs(msg.thrust.x) > 1.0 ||
                std::abs(msg.thrust.y) > 1.0 ||
                std::abs(msg.thrust.z) > 1.0)
                return false;
            return msg.control_mode == Setpoint::ATTITUDE ?
                valid_quaternion(msg.orientation) : valid_vector(msg.body_rate);
        default:
            return false;
        }
    }

    Setpoint effective_setpoint() const
    {
        if (has_setpoint &&
            age(Clock::now(), setpoint_received) < setpoint_timeout)
            return cached;

        Setpoint out;
        out.control_mode = Setpoint::VELOCITY;
        return out;
    }

    void publish_setpoint()
    {
        const auto msg = effective_setpoint();
        const auto stamp = timestamp();

        if (msg.control_mode == Setpoint::ATTITUDE) {
            const auto &v = msg.orientation;
            const auto q = ft::ros_to_px4_orientation(
                Eigen::Quaterniond(v.w, v.x, v.y, v.z).normalized());
            AttitudeTarget out{};
            out.timestamp = stamp;
            out.q_d = {
                static_cast<float>(q.w()), static_cast<float>(q.x()),
                static_cast<float>(q.y()), static_cast<float>(q.z())
            };
            out.thrust_body = frd(msg.thrust);
            attitude_target_pub->publish(out);
        } else if (msg.control_mode == Setpoint::BODY_RATE) {
            const auto rate = frd(msg.body_rate);
            RatesTarget out{};
            out.timestamp = stamp;
            out.roll = rate[0];
            out.pitch = rate[1];
            out.yaw = rate[2];
            out.thrust_body = frd(msg.thrust);
            rates_target_pub->publish(out);
        } else {
            Trajectory out{};
            out.timestamp = stamp;
            out.position = out.velocity = out.acceleration = out.jerk =
                {NAN_F, NAN_F, NAN_F};
            out.yaw = out.yawspeed = NAN_F;

            switch (msg.control_mode) {
            case Setpoint::POSITION: {
                out.position = ned(msg.position);
                const auto q = ft::ros_to_px4_orientation(
                    ft::utils::quaternion::quaternion_from_euler(
                        0.0, 0.0, msg.yaw * DEG_TO_RAD));
                out.yaw = static_cast<float>(
                    ft::utils::quaternion::quaternion_get_yaw(q));
                break;
            }
            case Setpoint::VELOCITY:
                out.velocity = ned(msg.velocity);
                out.yawspeed = static_cast<float>(-msg.yawspeed * DEG_TO_RAD);
                break;
            case Setpoint::ACCELERATION:
                out.acceleration = ned(msg.acceleration);
                if (msg.use_jerk)
                    out.jerk = ned(msg.jerk);
                out.yawspeed = static_cast<float>(-msg.yawspeed * DEG_TO_RAD);
                break;
            default:
                return;
            }
            trajectory_pub->publish(out);
        }
        published_mode = msg.control_mode;
    }

    void publish_heartbeat()
    {
        Mode out{};
        out.timestamp = timestamp();
        out.position = published_mode == Setpoint::POSITION;
        out.velocity = published_mode == Setpoint::VELOCITY;
        out.acceleration = published_mode == Setpoint::ACCELERATION;
        out.attitude = published_mode == Setpoint::ATTITUDE;
        out.body_rate = published_mode == Setpoint::BODY_RATE;
        mode_pub->publish(out);
        if (offboard_heartbeat_started == Time{})
            offboard_heartbeat_started = Clock::now();
    }

    void respond(const Header &header, bool success, const std::string &message)
    {
        Arm::Response response;
        response.success = success;
        response.message = message;
        try {
            arm_service->send_response(*header, response);
        } catch (const std::exception &error) {
            RCLCPP_WARN(node.get_logger(), "Arm response failed: %s", error.what());
        }
    }

    void finish(bool success, const std::string &message)
    {
        const auto header = upstream_header;
        if (request_id >= 0)
            command_client->remove_pending_request(request_id);
        pending = false;
        request_id = -1;
        upstream_header.reset();
        ++generation;
        respond(header, success, message);
    }

    void arm_request(Header header, bool arm)
    {
        if (pending) {
            respond(header, false, "BUSY");
            return;
        }
        if (!connected(Clock::now())) {
            respond(header, false, "PX4_DISCONNECTED");
            return;
        }
        if (px4_armed == arm) {
            respond(header, true, "ALREADY_IN_REQUESTED_STATE");
            return;
        }
        if (!command_client->service_is_ready()) {
            respond(header, false, "PX4_SERVICE_UNAVAILABLE");
            return;
        }

        auto request = std::make_shared<Command::Request>();
        auto &cmd = request->request;
        cmd.timestamp = timestamp();
        cmd.command =
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = arm ? 1.0f : 0.0f;
        cmd.param2 = 0.0f;
        cmd.target_system = target_system;
        cmd.target_component = target_component;
        cmd.source_system = source_system;
        cmd.source_component = source_component;
        cmd.from_external = true;

        pending = true;
        desired_arm = arm;
        reply_received = false;
        command_started = Clock::now();
        command_sequence = status_sequence;
        upstream_header = std::move(header);
        const auto token = ++generation;

        try {
            const auto result = command_client->async_send_request(
                request, [this, token](Client::SharedFuture future) {
                    if (!pending || token != generation)
                        return;
                    request_id = -1;
                    try {
                        const auto &reply = future.get()->reply;
                        if (reply.command !=
                            px4_msgs::msg::VehicleCommand::
                                VEHICLE_CMD_COMPONENT_ARM_DISARM) {
                            finish(false, "UNEXPECTED_COMMAND_ACK");
                            return;
                        }

                        reply_received = true;
                        switch (reply.result) {
                        case Ack::VEHICLE_CMD_RESULT_ACCEPTED:
                        case Ack::VEHICLE_CMD_RESULT_IN_PROGRESS:
                            break;
                        case Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                            finish(false, "TEMPORARILY_REJECTED");
                            break;
                        case Ack::VEHICLE_CMD_RESULT_DENIED:
                            finish(false, "DENIED");
                            break;
                        case Ack::VEHICLE_CMD_RESULT_UNSUPPORTED:
                            finish(false, "UNSUPPORTED");
                            break;
                        case Ack::VEHICLE_CMD_RESULT_FAILED:
                            finish(false, "FAILED");
                            break;
                        case Ack::VEHICLE_CMD_RESULT_CANCELLED:
                            finish(false, "CANCELLED");
                            break;
                        default:
                            finish(false, "UNKNOWN_ACK_RESULT");
                            break;
                        }
                    } catch (const std::exception &error) {
                        finish(false, std::string("SERVICE_ERROR: ") + error.what());
                    }
                });
            request_id = result.request_id;
        } catch (const std::exception &error) {
            finish(false, std::string("SEND_ERROR: ") + error.what());
        }
    }

    void check_command()
    {
        if (!pending)
            return;
        const auto now = Clock::now();
        if (!connected(now)) {
            finish(false, "PX4_DISCONNECTED");
        } else if (!reply_received && age(now, command_started) >= command_timeout) {
            finish(false, "SERVICE_TIMEOUT");
        } else if (age(now, command_started) >= arm_timeout) {
            finish(false, "STATE_CONFIRMATION_TIMEOUT");
        } else if (reply_received && status_sequence > command_sequence &&
                   px4_armed == desired_arm) {
            finish(true, "STATE_CONFIRMED");
        }
    }

    void manage_offboard()
    {
        const auto now = Clock::now();

        if (!connected(now) || !px4_position_valid) {
            return;
        }

        if (px4_nav_state == Status::NAVIGATION_STATE_OFFBOARD) {
            offboard_requested = true;
            return;
        }

        // 飞行过程中退出 Offboard，禁止自动抢占控制权
        if (px4_armed) {
            return;
        }

        if (offboard_heartbeat_started == Time{}) {
            return;
        }

        if (age(now, offboard_heartbeat_started) <
            offboard_warmup) {
            return;
        }

        if (last_offboard_request != Time{} &&
            age(now, last_offboard_request) <
                offboard_retry_interval) {
            return;
        }

        if (!command_client->service_is_ready()) {
            return;
        }

        auto request = std::make_shared<Command::Request>();
        auto& cmd = request->request;

        cmd.timestamp = timestamp();
        cmd.command =
            px4_msgs::msg::VehicleCommand::
                VEHICLE_CMD_DO_SET_MODE;

        cmd.param1 = 1.0f;
        cmd.param2 = 6.0f;

        cmd.target_system = target_system;
        cmd.target_component = target_component;
        cmd.source_system = source_system;
        cmd.source_component = source_component;
        cmd.from_external = true;

        last_offboard_request = now;

        command_client->async_send_request(
            request,
            [this](Client::SharedFuture future) {
                try {
                    const auto& reply = future.get()->reply;

                    if (reply.result ==
                        Ack::VEHICLE_CMD_RESULT_ACCEPTED) {
                        RCLCPP_INFO(
                            node.get_logger(),
                            "Offboard mode command accepted");
                    } else {
                        RCLCPP_WARN(
                            node.get_logger(),
                            "Offboard mode command rejected: %u",
                            static_cast<unsigned>(reply.result));
                    }
                } catch (const std::exception& error) {
                    RCLCPP_ERROR(
                        node.get_logger(),
                        "Offboard command error: %s",
                        error.what());
                }
            });
    }
};

PX4Interface::PX4Interface(const rclcpp::NodeOptions &options)
    : Node("px4_interface", options), impl_(std::make_unique<Impl>(*this))
{
}

PX4Interface::~PX4Interface() = default;