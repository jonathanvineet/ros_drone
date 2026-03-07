import asyncio
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


TARGET_ALT = -2.0


class BoxNode(Node):
    def __init__(self):
        super().__init__("box_node")
        self.target = None
        self.create_subscription(Point, "/closest_box", self.cb, 10)

    def cb(self, msg):
        self.target = msg


async def wait_until_altitude(drone, target, tol=0.15):
    async for pos in drone.telemetry.position_velocity_ned():
        if abs(pos.position.down_m - target) < tol:
            return
        await asyncio.sleep(0.05)


async def wait_until_xy(drone, tx, ty, tol=0.2):
    async for pos in drone.telemetry.position_velocity_ned():
        dx = pos.position.north_m - tx
        dy = pos.position.east_m - ty
        if math.sqrt(dx*dx + dy*dy) < tol:
            return
        await asyncio.sleep(0.05)


async def main():
    rclpy.init()
    node = BoxNode()

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection…")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("Arming…")
    await drone.action.arm()

    print("Sending initial offboard hold setpoints…")
    for _ in range(20):
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))
        await asyncio.sleep(0.05)

    try:
        await drone.offboard.start()
        print("Offboard started.")
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return

    print("Climbing to hover altitude…")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, TARGET_ALT, 0))
    await wait_until_altitude(drone, TARGET_ALT)

    print("Waiting 5 seconds to stabilize…")
    await asyncio.sleep(5)

    print("Waiting for closest box…")
    while node.target is None:
        rclpy.spin_once(node)
        await asyncio.sleep(0.1)

    tx = node.target.x
    ty = -node.target.y

    print(f"Closest box: N={tx:.2f}, E={ty:.2f}")

    print("Flying above box…")
    await drone.offboard.set_position_ned(PositionNedYaw(tx, ty, TARGET_ALT, 0))
    await wait_until_xy(drone, tx, ty)

    print("Reached box. Landing…")
    await drone.action.land()

    rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
