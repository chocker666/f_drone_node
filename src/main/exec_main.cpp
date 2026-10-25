#include <chrono>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

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

        auto fsm = std::make_shared<FlightStateMachine>();

        const auto logger = motion_executor->get_logger();

        std::weak_ptr<MotionExecutor> weak_executor =
            motion_executor;

        // 阶段 4 只接入执行故障。
        // 正常完成与具体 Action 的对应关系，由阶段 5 的管理器接管。
        motion_executor->setExecutionCallback(
            [fsm, weak_executor, logger](
                const MotionExecutor::ExecutionEvent& event)
            {
                auto executor_node = weak_executor.lock();
                if (!executor_node)
                    return;

                fsm->update_snapshot(
                    executor_node->getVehicleSnapshot());

                if (event.outcome ==
                    MotionExecutor::ExecutionOutcome::FAILED) {
                    fsm->process_event(FlightEvent::FAULT_DETECTED);

                    RCLCPP_ERROR(
                        logger,
                        "Execution failure latched in FSM: %s",
                        event.message.c_str());
                }
            });

        auto previous_phase =
            std::make_shared<FlightPhase>(FlightPhase::UNKNOWN);

        auto state_timer = motion_executor->create_wall_timer(
            std::chrono::milliseconds(50),
            [fsm, weak_executor, previous_phase, logger]
            {
                auto executor_node = weak_executor.lock();
                if (!executor_node)
                    return;

                fsm->update_snapshot(
                    executor_node->getVehicleSnapshot());

                const auto current = fsm->phase();

                if (current == *previous_phase)
                    return;

                RCLCPP_INFO(
                    logger,
                    "Flight phase: %s -> %s",
                    to_string(*previous_phase),
                    to_string(current));

                if (current == FlightPhase::FAULT) {
                    RCLCPP_ERROR(
                        logger,
                        "FSM fault: %s",
                        fsm->fault_reason().c_str());
                }

                *previous_phase = current;
            });

        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2);

        executor.add_node(motion_executor);
        executor.spin();

        state_timer->cancel();
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