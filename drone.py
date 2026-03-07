import asyncio
from mavsdk import System

async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered")
            break

    print("Waiting for global position...")

    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Position OK")
            break

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("Landing...")
    await drone.action.land()


asyncio.run(run())