import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw


TARGET_ALTITUDE_D = -6.0
CLIMB_SPEED_MPS = 2.0


async def climb_straight_up(drone, target_down_m, speed_mps=2.0, tol=0.3):
    """Command a vertical climb in NED until the target altitude is reached."""
    async for pos in drone.telemetry.position_velocity_ned():
        current_down = pos.position.down_m

        if current_down <= target_down_m + tol:
            print(f"Target altitude reached: D={current_down:.2f} m")
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, target_down_m, 0.0))
            return

        # Negative down velocity means climb up in NED.
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -speed_mps, 0.0))
        print(f"Climbing... current D={current_down:.2f} m")

        await asyncio.sleep(0.1)

async def run():

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Waiting for local position...")

    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Local position OK")
            break

    print("Arming...")
    await drone.action.arm()

    print("Starting OFFBOARD")

    # Pre-feed setpoints before offboard start. This is required by PX4.
    for _ in range(20):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.05)

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        await drone.action.land()
        return

    print(f"Climbing straight up to {abs(TARGET_ALTITUDE_D):.1f} m at {CLIMB_SPEED_MPS:.1f} m/s")
    await climb_straight_up(drone, TARGET_ALTITUDE_D, speed_mps=CLIMB_SPEED_MPS)

    print("Reached target altitude. Landing")
    await drone.action.land()

asyncio.run(run())
