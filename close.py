import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math


class ClosestBox(Node):
    def __init__(self):
        super().__init__("closest_box_detector")

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Point,
            "/closest_box",
            10
        )

        self.get_logger().info("Publishing closest box on /closest_box")

    def callback(self, msg):

        ranges = [r for r in msg.ranges if math.isfinite(r)]

        if not ranges:
            return

        # find closest distance
        min_dist = min(ranges)
        min_index = msg.ranges.index(min_dist)

        angle = msg.angle_min + min_index * msg.angle_increment

        x = min_dist * math.cos(angle)
        y = min_dist * math.sin(angle)

        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0

        self.pub.publish(p)

        self.get_logger().info(
            f"Closest box → dist={min_dist:.2f}m  pos(x={x:.2f}, y={y:.2f})"
        )


def main():
    rclpy.init()
    node = ClosestBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
