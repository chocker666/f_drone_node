#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include "drone_node/exec/action_manager.hpp"
#include "drone_node/exec/exec_node.hpp"

using namespace drone_node::exec;
using namespace std::chrono_literals;

namespace
{
void require(bool value, const char* message)
{
    if (!value)
        throw std::runtime_error(message);
}

template<typename Predicate>
void until(Predicate predicate)
{
    const auto deadline = std::chrono::steady_clock::now() + 8s;
    while (!predicate()) {
        require(std::chrono::steady_clock::now() < deadline, "Timed out");
        std::this_thread::sleep_for(5ms);
    }
}

template<typename Future>
auto ready(Future future)
{
    require(future.wait_for(8s) == std::future_status::ready, "Future timed out");
    return future.get();
}

// Only the shared node is added to the executor. The fake vehicle uses the
// default group; all production callbacks use the two explicit groups.
struct Fixture
{
    using Status = MotionExecutor::ControlStatus;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::CallbackGroup::SharedPtr action_group;
    rclcpp::CallbackGroup::SharedPtr motion_group;
    std::shared_ptr<FlightStateMachine> fsm;
    std::shared_ptr<MotionExecutor> motion;
    std::shared_ptr<ActionManager> manager;
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 2};
    std::thread worker;
    std::atomic<bool> armed{false}, landed{true}, publish_status{true};
    std::atomic<std::uint8_t> state{Status::IDLE};
    std::atomic<std::uint64_t> trajectory{0};
    std::atomic<unsigned> events{0};
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm, land, connected;
    rclcpp::Publisher<Status>::SharedPtr status;
    rclcpp::Subscription<MotionExecutor::ControlTrajectory>::SharedPtr trajectories;
    rclcpp::Subscription<MotionExecutor::ControlCommand>::SharedPtr commands;
    rclcpp::TimerBase::SharedPtr telemetry;

    explicit Fixture(bool in_air = false)
    {
        armed = in_air;
        landed = !in_air;
        state = in_air ? Status::HOVER : Status::IDLE;
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("arm.service_wait_timeout", 0.5),
            rclcpp::Parameter("observation.hover_settle_duration", 0.05),
            rclcpp::Parameter("goto.settle_duration", 0.05),
            rclcpp::Parameter("stop.settle_duration", 0.05)});
        node = std::make_shared<rclcpp::Node>("exec_test", options);
        action_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        motion_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        fsm = std::make_shared<FlightStateMachine>();
        motion = std::make_shared<MotionExecutor>(*node, motion_group);
        manager = std::make_shared<ActionManager>(*node, action_group, motion, fsm);
        std::weak_ptr<ActionManager> weak = manager;
        motion->setExecutionCallback([this, weak](const auto& event) {
            // Reentering a snapshot accessor verifies notification is outside
            // execution_mutex_. FSM writes remain in the manager's group.
            (void)motion->getExecutionSnapshot();
            ++events;
            if (auto target = weak.lock())
                target->enqueueExecutionEvent(event);
        });
        const auto qos = rclcpp::QoS(10).reliable();
        position = node->create_publisher<geometry_msgs::msg::PointStamped>("/px4_interface/out/position", qos);
        velocity = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/px4_interface/out/velocity", qos);
        attitude = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("/px4_interface/out/attitude", qos);
        arm = node->create_publisher<std_msgs::msg::Bool>("/px4_interface/out/arm", qos);
        land = node->create_publisher<std_msgs::msg::Bool>("/px4_interface/out/landed", qos);
        connected = node->create_publisher<std_msgs::msg::Bool>("/px4_interface/out/connected", qos);
        status = node->create_publisher<Status>("/flight_controller/out/control_status", qos);
        trajectories = node->create_subscription<MotionExecutor::ControlTrajectory>(
            "/flight_controller/in/trajectory", qos, [this](MotionExecutor::ControlTrajectory::ConstSharedPtr msg) {
                trajectory = msg->trajectory_id;
                state = Status::TRACKING;
            });
        commands = node->create_subscription<MotionExecutor::ControlCommand>(
            "/flight_controller/in/command", qos, [this](MotionExecutor::ControlCommand::ConstSharedPtr) {
                trajectory = 0;
                state = Status::HOVER;
            });
        telemetry = node->create_wall_timer(10ms, [this] {
            geometry_msgs::msg::PointStamped p;
            p.header.frame_id = "map";
            p.point.z = 1.0;
            position->publish(p);
            geometry_msgs::msg::Vector3Stamped v;
            v.header.frame_id = "map";
            velocity->publish(v);
            geometry_msgs::msg::QuaternionStamped q;
            q.header.frame_id = "map";
            q.quaternion.w = 1.0;
            attitude->publish(q);
            std_msgs::msg::Bool b;
            b.data = armed.load(); arm->publish(b);
            b.data = landed.load(); land->publish(b);
            b.data = true; connected->publish(b);
            if (publish_status) {
                Status s;
                s.stamp = node->now();
                s.state = state.load();
                s.trajectory_id = trajectory.load();
                status->publish(s);
            }
        });
        executor.add_node(node);
        worker = std::thread([this] { executor.spin(); });
        try {
            until([this, in_air] { return fsm->phase() ==
                (in_air ? FlightPhase::HOVERING : FlightPhase::GROUNDED); });
        }
        catch (...) {
            quiesce();
            throw;
        }
    }

    void quiesce()
    {
        executor.cancel();
        if (worker.joinable()) worker.join();
    }

    ~Fixture()
    {
        quiesce();
        manager->stop();
        motion->stop();
        motion->setExecutionCallback({});
        manager.reset();
        motion.reset();
    }

    template<typename Action>
    auto client(const char* name)
    {
        auto result = rclcpp_action::create_client<Action>(node, name);
        require(result->wait_for_action_server(5s), "Action server unavailable");
        return result;
    }


};

void arm_timeout_and_group_progress()
{
    Fixture f;
    auto client = f.client<drone_node::action::Arm>("/drone_node/arm");
    auto goal = ready(client->async_send_goal(drone_node::action::Arm::Goal{}));
    require(bool(goal), "Arm rejected");
    auto duplicate = ready(client->async_send_goal(drone_node::action::Arm::Goal{}));
    require(!duplicate, "Duplicate Arm accepted");
    const auto before = f.motion->getStateSnapshot().position.received_at;
    const auto result = ready(client->async_get_result(goal));
    require(result.code == rclcpp_action::ResultCode::ABORTED, "Missing service must abort Arm");
    require(result.result->error_code == 2, "Expected service availability timeout");
    require(f.motion->getStateSnapshot().position.received_at > before, "Group B stalled during Arm");
}

void fault_and_reset()
{
    Fixture f;
    f.publish_status = false;
    // Grounded FSM tolerates a stale Ctrl status, but ResetFault must reject it.
    f.state = Fixture::Status::FAULT;
    f.publish_status = true;
    until([&] { return f.fsm->phase() == FlightPhase::FAULT; });
    auto reset = f.node->create_client<std_srvs::srv::Trigger>("/drone_node/reset_fault");
    require(reset->wait_for_service(5s), "ResetFault unavailable");
    auto rejected = ready(reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()));
    require(!rejected->success, "Reset accepted Ctrl fault");
    f.state = Fixture::Status::IDLE;
    until([&] { return !f.motion->getVehicleSnapshot().control_fault; });
    auto accepted = ready(reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()));
    require(accepted->success, "Reset rejected fresh grounded state");
    require(f.fsm->phase() == FlightPhase::GROUNDED, "Reset did not restore GROUNDED");
}

void motion_cancel_stop_and_completion()
{
    Fixture f(true);
    auto client = f.client<drone_node::action::GoTo>("/drone_node/goto");
    auto stop_client = f.client<drone_node::action::Stop>("/drone_node/stop");
    drone_node::action::GoTo::Goal request;
    request.header.frame_id = "map";
    request.position.x = 3.0;
    request.position.z = 1.0;
    auto terminal_count = std::make_shared<std::atomic<unsigned>>(0);
    rclcpp_action::Client<drone_node::action::GoTo>::SendGoalOptions options;
    options.result_callback = [terminal_count](const auto&) { ++*terminal_count; };
    auto goal = ready(client->async_send_goal(request, options));
    require(bool(goal), "GoTo rejected");
    until([&] { return f.motion->getExecutionSnapshot().phase == MotionExecutor::ExecutionPhase::TRACKING; });
    require(!ready(client->async_send_goal(request)), "Duplicate GoTo accepted");
    auto canceled = ready(client->async_cancel_goal(goal));
    require(!canceled->goals_canceling.empty(), "GoTo cancellation rejected");
    auto stop = ready(stop_client->async_send_goal(drone_node::action::Stop::Goal{}));
    require(bool(stop), "Stop rejected");
    require(ready(client->async_get_result(goal)).code == rclcpp_action::ResultCode::CANCELED,
        "GoTo must finish CANCELED once");
    require(ready(stop_client->async_get_result(stop)).code == rclcpp_action::ResultCode::SUCCEEDED,
        "Stop failed");
    until([&] { return terminal_count->load() == 1; });
    require(f.events == 1, "Execution terminal event duplicated");
    require(f.fsm->phase() == FlightPhase::HOVERING, "Stop did not restore HOVERING");

    // Target equals the fake vehicle position: normal completion still needs
    // TRACKING, fresh HOVER and the original stability window.
    request.position.x = 0.0;
    auto completed = ready(client->async_send_goal(request));
    require(bool(completed), "Second GoTo rejected");
    require(ready(client->async_get_result(completed)).code == rclcpp_action::ResultCode::SUCCEEDED,
        "GoTo completion failed");
    require(f.events == 2 && terminal_count->load() == 1, "Duplicate terminal delivery");

    auto stale = ready(client->async_send_goal(request));
    require(bool(stale), "Stale-state scenario rejected");
    f.publish_status = false;
    require(ready(client->async_get_result(stale)).code == rclcpp_action::ResultCode::ABORTED,
        "Stale Ctrl status must abort motion");
    require(f.fsm->phase() == FlightPhase::FAULT, "Stale Ctrl did not latch FAULT");
}

void group_isolation_and_event_queue()
{
    Fixture f(true);
    auto client = f.client<drone_node::action::GoTo>("/drone_node/goto");
    drone_node::action::GoTo::Goal request;
    request.header.frame_id = "map";
    request.position.x = 3.0;
    request.position.z = 1.0;
    require(bool(ready(client->async_send_goal(request))), "GoTo rejected");
    until([&] { return f.motion->getExecutionSnapshot().phase == MotionExecutor::ExecutionPhase::TRACKING; });
    std::atomic<bool> entered{false}, finished{false};
    std::atomic<bool> isolated{false};
    auto blocker = f.node->create_wall_timer(10ms, [&] {
        if (entered.exchange(true)) return;
        const auto before = f.motion->getStateSnapshot().position.received_at;
        MotionExecutor::ExecutionEvent event;
        event.trajectory_id = f.trajectory.load() + 100; // stale/unbound event
        event.outcome = MotionExecutor::ExecutionOutcome::FAILED;
        f.manager->enqueueExecutionEvent(event);
        std::this_thread::sleep_for(150ms);
        isolated = f.motion->getStateSnapshot().position.received_at > before &&
            f.fsm->phase() == FlightPhase::MOVING;
        finished = true;
    }, f.action_group);
    until([&] { return finished.load(); });
    require(isolated, "Group B blocked or queued event wrote FSM across groups");
    blocker->cancel();
    // After Group A resumes, the unmatched event must still be ignored.
    std::this_thread::sleep_for(100ms);
    require(f.fsm->phase() == FlightPhase::MOVING, "Unbound event changed FSM");
    f.quiesce();
}

void shutdown_active_goal()
{
    Fixture f;
    auto client = f.client<drone_node::action::Arm>("/drone_node/arm");
    require(bool(ready(client->async_send_goal(drone_node::action::Arm::Goal{}))), "Arm rejected");
    // Fixture stops executor workers before releasing the active operation.
}
}  // namespace

int main(int argc, char** argv)
{
    static_assert(!std::is_base_of_v<rclcpp::Node, MotionExecutor>);
    static_assert(!std::is_base_of_v<rclcpp::Node, ActionManager>);
    rclcpp::init(argc, argv);
    try {
        arm_timeout_and_group_progress();
        fault_and_reset();
        motion_cancel_stop_and_completion();
        group_isolation_and_event_queue();
        shutdown_active_goal();
        auto node = std::make_shared<ExecNode>();
        node->shutdown();
        node->shutdown();
    }
    catch (...) {
        rclcpp::shutdown();
        throw;
    }
    rclcpp::shutdown();
}
