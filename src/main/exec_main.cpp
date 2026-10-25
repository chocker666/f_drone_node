#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_node/exec/action_manager.hpp"
#include "drone_node/exec/flight_state_machine.hpp"
#include "drone_node/exec/motion_executor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int result = 0;

    try {
        using namespace drone_node::exec;

        auto motion_executor =
            std::make_shared<MotionExecutor>();

        auto fsm =
            std::make_shared<FlightStateMachine>();

        auto action_manager =
            std::make_shared<ActionManager>(
                motion_executor, fsm);

        std::weak_ptr<ActionManager> weak_manager =
            action_manager;

        motion_executor->setExecutionCallback(
            [weak_manager](
                const MotionExecutor::ExecutionEvent& event)
            {
                if (auto manager = weak_manager.lock())
                    manager->enqueueExecutionEvent(event);
            });

        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 4);

        executor.add_node(motion_executor);
        executor.add_node(action_manager);
        executor.spin();

        motion_executor->setExecutionCallback({});
    }
    catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("exec_main"),
            "Exec failed: %s",
            error.what());

        result = 1;
    }

    rclcpp::shutdown();
    return result;
}