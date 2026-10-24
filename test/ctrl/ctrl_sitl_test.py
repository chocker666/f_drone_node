
#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from drone_node.msg import (
    ControlTrajectory,
    ControlCommand,
    ControlStatus,
    Setpoint,
    FixedTrajectory,
    ListTrajectory,
)


class CtrlTest(Node):
    def __init__(self):
        super().__init__("ctrl_sitl_test")

        self.trajectory_pub = self.create_publisher(
            ControlTrajectory,
            "/flight_controller/in/trajectory",
            1,
        )

        self.command_pub = self.create_publisher(
            ControlCommand,
            "/flight_controller/in/command",
            10,
        )

        self.status_sub = self.create_subscription(
            ControlStatus,
            "/flight_controller/out/control_status",
            self.on_status,
            10,
        )

        self.setpoint_sub = self.create_subscription(
            Setpoint,
            "/px4_interface/in/setpoint",
            self.on_setpoint,
            1,
        )

        self.position = None
        self.yaw = None
        self.status = None
        self.last_setpoint = None

        from geometry_msgs.msg import PointStamped, QuaternionStamped

        self.create_subscription(
            PointStamped,
            "/px4_interface/out/position",
            self.on_position,
            1,
        )

        self.create_subscription(
            QuaternionStamped,
            "/px4_interface/out/attitude",
            self.on_attitude,
            1,
        )

    def on_position(self, msg):
        self.position = (
            msg.point.x,
            msg.point.y,
            msg.point.z,
        )

    def on_attitude(self, msg):
        q = msg.quaternion
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_status(self, msg):
        self.status = msg

    def on_setpoint(self, msg):
        self.last_setpoint = msg

    def current_position(self):
        if self.position is None:
            raise RuntimeError("No position feedback received")
        return self.position

    def current_yaw(self):
        if self.yaw is None:
            raise RuntimeError("No attitude feedback received")
        return self.yaw

    @staticmethod
    def point(x, y, z, yaw, direction=0):
        p = FixedTrajectory()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.yaw = float(yaw)
        p.yaw_direction = int(direction)
        return p

    def base_trajectory(self, trajectory_id):
        msg = ControlTrajectory()
        msg.trajectory_id = int(trajectory_id)
        msg.frame_id = "map"
        msg.controller_id = "position_yaw_pid"
        msg.config_name = "default"
        return msg

    def send_point(self, trajectory_id, x, y, z, yaw):
        msg = self.base_trajectory(trajectory_id)
        msg.trajectory_type = ControlTrajectory.POINT
        msg.point_params = self.point(x, y, z, yaw)
        self.trajectory_pub.publish(msg)

        print(
            f"POINT id={trajectory_id} "
            f"p=({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.2f}"
        )

    def send_list(self, trajectory_id, points):
        msg = self.base_trajectory(trajectory_id)
        msg.trajectory_type = ControlTrajectory.LIST

        params = ListTrajectory()
        params.points = [
            self.point(*point) for point in points
        ]

        params.target_speed = 0.5
        params.max_speed = 1.0
        params.max_acceleration = 0.5
        params.max_yaw_rate = 0.5
        params.max_yaw_acceleration = 0.5

        msg.list_params = params
        self.trajectory_pub.publish(msg)
        print(f"LIST id={trajectory_id}, points={len(points)}")

    def send_command(self, command):
        msg = ControlCommand()
        msg.command = command
        self.command_pub.publish(msg)
        print(f"COMMAND={command}")

    def log_status(self):
        if self.status is None:
            print("No ControlStatus")
            return

        s = self.status
        p = self.position

        print(
            f"state={s.state} "
            f"traj={s.trajectory_id} "
            f"fault={s.fault_code} "
            f"pos_error={s.position_error:.3f} "
            f"yaw_error={s.yaw_error:.3f} "
            f"position={p}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "test",
        choices=["point", "yaw", "list", "cancel", "reset", "status"],
    )
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = CtrlTest()

    try:
        print("Collecting current vehicle feedback...")

        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.position is not None and node.yaw is not None:
                break

        if args.test in ("point", "yaw", "list"):
            x, y, z = node.current_position()
            yaw = node.current_yaw()

            if args.test == "point":
                node.send_point(10, x + 1.0, y, z+3, yaw)

            elif args.test == "yaw":
                node.send_point(
                    102, x + 1.0, y, z,
                    yaw + math.pi / 2.0,
                )

            elif args.test == "list":
                node.send_list(
                    103,
                    [
                        (x, y, z, yaw),
                        (x + 1.0, y, z, yaw+90),
                        (x + 1.0, y + 1.0, z, yaw),
                        (x, y + 1.0, z, yaw+90),
                        (x, y, z, yaw-90),
                    ],
                )

        elif args.test == "cancel":
            node.send_command(ControlCommand.CANCEL)

        elif args.test == "reset":
            node.send_command(ControlCommand.RESET_FAULT)

        finish = time.monotonic() + args.duration
        last_print = 0.0

        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=0.1)

            if time.monotonic() - last_print >= 1.0:
                node.log_status()
                last_print = time.monotonic()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
