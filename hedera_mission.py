import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def run():

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Arming...")
    await drone.action.arm()

    print("Takeoff")
    await drone.action.takeoff()

    await asyncio.sleep(6)

    print("Starting OFFBOARD")

    await drone.offboard.set_position_ned(
        PositionNedYaw(0,0,-120,0)
    )

    await drone.offboard.start()

    print("Hovering at 60m")
    await asyncio.sleep(10)

    print("Flying forest border")

    waypoints = [

        (-120,-120,-600),
        (120,-120,-60),
        (120,120,-60),
        (-120,120,-60),
        (-120,-120,-60)

    ]

    for wp in waypoints:

        print("Going to:",wp)

        await drone.offboard.set_position_ned(
            PositionNedYaw(wp[0],wp[1],wp[2],0)
        )

        await asyncio.sleep(12)

    print("Landing")

    await drone.action.land()

asyncio.run(run())
