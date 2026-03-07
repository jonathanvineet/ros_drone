#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode

TARGET = (0.0, 0.0, 5.0)   # hover 5m above home
TOL = 0.25
RATE = 0.05                # 20 Hz


class OffboardHover(Node):
    def __init__(self):
        super().__init__("offboard_hover")

        self.current = None
        self.reached = False

        # publish local position setpoints
        self.sp_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10
        )

        # subscribe to odom
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_cb,
            qos
        )

        # services
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.get_logger().info("Waiting for services…")

        self.arm_cli.wait_for_service()
        self.mode_cli.wait_for_service()

        # 1️⃣ start sending setpoints first
        self.timer = self.create_timer(RATE, self.loop)

        self.get_logger().info("Pre-feeding OFFBOARD setpoints…")
        time.sleep(2.0)

        # 2️⃣ enter offboard
        self.set_offboard()
        time.sleep(0.5)

        # 3️⃣ arm
        self.arm()

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arm_cli.call_async(req)
        self.get_logger().info("Arming…")

    def set_offboard(self):
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.mode_cli.call_async(req)
        self.get_logger().info("Switching to OFFBOARD…")

    def odom_cb(self, msg):
        self.current = msg.pose.pose.position

    def loop(self):
        # continuously publish target
        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.pose.position.x = TARGET[0]
        sp.pose.position.y = TARGET[1]
        sp.pose.position.z = TARGET[2]

        self.sp_pub.publish(sp)

        if not self.current:
            return

        dist = math.sqrt(
            (TARGET[0] - self.current.x) ** 2 +
            (TARGET[1] - self.current.y) ** 2 +
            (TARGET[2] - self.current.z) ** 2
        )

        if not self.reached and dist < TOL:
            self.reached = True
            self.get_logger().info("Reached hover altitude — holding position (no landing).")


def main():
    rclpy.init()
    node = OffboardHover()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
