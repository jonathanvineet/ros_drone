import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarDetector(Node):
    def __init__(self):
        super().__init__("lidar_multi_detector")

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.callback,
            10
        )

        self.get_logger().info("Listening for objects on /scan ...")

    def callback(self, msg):

        ranges = list(msg.ranges)

        # Replace inf values so they don't break clustering
        ranges = [r if math.isfinite(r) else None for r in ranges]

        clusters = []
        current_cluster = []

        DIST_JUMP = 0.25     # break cluster if gap > 25cm
        MIN_POINTS = 4       # filter noise

        previous = None

        for i, r in enumerate(ranges):

            if r is None:
                if len(current_cluster) >= MIN_POINTS:
                    clusters.append(current_cluster)
                current_cluster = []
                previous = None
                continue

            if previous is None:
                current_cluster.append((i, r))
            else:
                if abs(r - previous) < DIST_JUMP:
                    current_cluster.append((i, r))
                else:
                    if len(current_cluster) >= MIN_POINTS:
                        clusters.append(current_cluster)
                    current_cluster = [(i, r)]

            previous = r

        if len(current_cluster) >= MIN_POINTS:
            clusters.append(current_cluster)

        if len(clusters) == 0:
            self.get_logger().info("No objects detected")
            return

        self.get_logger().info(f"Detected {len(clusters)} objects")

        for idx, cluster in enumerate(clusters):

            indices = [c[0] for c in cluster]
            distances = [c[1] for c in cluster]

            center_index = sum(indices) / len(indices)
            center_distance = sum(distances) / len(distances)

            angle = msg.angle_min + center_index * msg.angle_increment

            # Convert to XY
            x = center_distance * math.cos(angle)
            y = center_distance * math.sin(angle)

            # Estimate object width based on outer rays
            left = cluster[0][1]
            right = cluster[-1][1]
            width = abs(left - right)

            self.get_logger().info(
                f"[Object {idx+1}] "
                f"dist={center_distance:.2f} m | "
                f"angle={math.degrees(angle):.1f}° | "
                f"pos(x={x:.2f}, y={y:.2f}) | "
                f"approx width={width:.2f} m"
            )


def main():
    rclpy.init()
    node = LidarDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
