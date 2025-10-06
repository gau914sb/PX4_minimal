#!/usr/bin/env python3

"""
PX4 Takeoff and Hover Script
This script connects to PX4 SITL, arms the drone, takes off to 2.5m, and hovers for 30 seconds.
"""

import asyncio
import time
from mavsdk import System


async def takeoff_and_hover():
    # Initialize the drone
    drone = System()

    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for takeoff to complete
    print("-- Waiting for takeoff to complete...")
    await asyncio.sleep(10)

    # Check altitude and confirm takeoff
    async for position in drone.telemetry.position():
        print(f"-- Altitude: {position.relative_altitude_m:.1f}m")
        if position.relative_altitude_m > 2.0:
            print("-- Takeoff complete!")
            break

    print("-- Hovering for 30 seconds...")
    await asyncio.sleep(30)

    print("-- Landing")
    await drone.action.land()

    # Wait for landing to complete
    print("-- Waiting for landing...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed!")
            break

    print("-- Disarming")
    await drone.action.disarm()

    print("-- Mission complete!")


if __name__ == "__main__":
    # Check if MAVSDK is installed
    try:
        from mavsdk import System
        print("MAVSDK found, starting mission...")
        asyncio.run(takeoff_and_hover())
    except ImportError:
        print("MAVSDK not found. Installing...")
        import subprocess
        import sys

        # Install MAVSDK
        subprocess.check_call([sys.executable, "-m", "pip", "install", "mavsdk"])
        print("MAVSDK installed! Please run the script again.")
