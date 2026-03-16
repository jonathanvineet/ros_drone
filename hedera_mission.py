import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw


TARGET_ALTITUDE_D = -6.0
CLIMB_SPEED_MPS = 4.0
POSITION_TOLERANCE = 0.5  # meters

# Define waypoints as (north, east, down) in meters
WAYPOINTS = [
    {"north": 0.0, "east": 0.0, "down": -8.0, "name": "Waypoint 1"},
    {"north": 48.0, "east": 5.0, "down": -8.0, "name": "Waypoint 2"},
    {"north": 10.0, "east": 0.0, "down": -6.0, "name": "Waypoint 3"},
    # Add more waypoints here in the same format
]


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

async def move_to_waypoint(drone, waypoint, tolerance=POSITION_TOLERANCE):
    """Move drone to a waypoint and check if it reached there."""
    north = waypoint["north"]
    east = waypoint["east"]
    down = waypoint["down"]
    name = waypoint.get("name", "Waypoint")
    
    print(f"\nMoving to {name}: N={north:.1f}, E={east:.1f}, D={down:.1f}")
    
    # Send position command
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, 0.0))
    
    # Check if drone reached the waypoint
    while True:
        async for pos in drone.telemetry.position_velocity_ned():
            current_north = pos.position.north_m
            current_east = pos.position.east_m
            current_down = pos.position.down_m
            
            # Calculate distance to waypoint
            distance = ((current_north - north)**2 + 
                       (current_east - east)**2 + 
                       (current_down - down)**2)**0.5
            
            print(f"  {name} - Distance: {distance:.2f}m (N={current_north:.1f}, E={current_east:.1f}, D={current_down:.1f})")
            
            if distance <= tolerance:
                print(f"✓ Reached {name}!")
                return True
            
            break  # Break inner loop to check again
        
        await asyncio.sleep(0.5)

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

    print("\n=== Starting Waypoint Mission ===")
    
    # Visit each waypoint
    for i, waypoint in enumerate(WAYPOINTS, 1):
        print(f"\n[{i}/{len(WAYPOINTS)}]", end=" ")
        await move_to_waypoint(drone, waypoint, tolerance=POSITION_TOLERANCE)
        await asyncio.sleep(1)  # Pause at each waypoint
    
    print("\n=== Mission Complete ===")
    print("Landing...")
    await drone.action.land()

asyncio.run(run())
