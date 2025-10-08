#!/usr/bin/env python3
"""
Script to arm and takeoff PX4 quadcopter using MAVSDK
"""
import asyncio
import sys
from mavsdk import System

async def run():
    drone = System()
    print("Connecting to PX4 on udp://:14540...")
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✅ Connected to drone!")
            break
    
    # Get vehicle info
    try:
        info = await drone.info.get_version()
        print(f"📋 Version: {info.flight_sw_major}.{info.flight_sw_minor}.{info.flight_sw_patch}")
    except Exception as e:
        print(f"⚠️  Could not get version: {e}")
    
    # Check if armed
    async for armed in drone.telemetry.armed():
        print(f"🔒 Armed status: {armed}")
        break
    
    # Get position
    async for position in drone.telemetry.position():
        print(f"📍 Position: Lat={position.latitude_deg:.6f}, Lon={position.longitude_deg:.6f}, Alt={position.relative_altitude_m:.2f}m")
        break
    
    # Get battery
    async for battery in drone.telemetry.battery():
        print(f"🔋 Battery: {battery.remaining_percent*100:.1f}%, {battery.voltage_v:.2f}V")
        break
    
    print("\n" + "="*50)
    print("🚁 ATTEMPTING TO ARM")
    print("="*50)
    try:
        await drone.action.arm()
        print("✅ Armed successfully!")
    except Exception as e:
        print(f"❌ Arming failed: {e}")
        return
    
    await asyncio.sleep(2)
    
    print("\n" + "="*50)
    print("🚀 ATTEMPTING TAKEOFF to 2.5m")
    print("="*50)
    try:
        await drone.action.set_takeoff_altitude(2.5)
        await drone.action.takeoff()
        print("✅ Takeoff command sent!")
    except Exception as e:
        print(f"❌ Takeoff failed: {e}")
        return
    
    # Monitor altitude during takeoff
    print("\n📈 Monitoring altitude during ascent...")
    start_time = asyncio.get_event_loop().time()
    async for position in drone.telemetry.position():
        current_time = asyncio.get_event_loop().time()
        elapsed = current_time - start_time
        alt = position.relative_altitude_m
        print(f"   Altitude: {alt:6.2f}m  |  Time: {elapsed:5.1f}s", end="\r")
        
        if alt > 2.0:
            print(f"\n✅ Reached target altitude: {alt:.2f}m")
            break
        
        if elapsed > 30:
            print(f"\n⚠️  Timeout waiting for altitude")
            break
        
        await asyncio.sleep(0.2)
    
    print("\n" + "="*50)
    print("🛸 HOVERING for 5 seconds")
    print("="*50)
    await asyncio.sleep(5)
    
    print("\n" + "="*50)
    print("🛬 LANDING")
    print("="*50)
    try:
        await drone.action.land()
        print("✅ Landing command sent!")
    except Exception as e:
        print(f"❌ Landing failed: {e}")
    
    # Wait for landing
    print("\n📉 Monitoring descent...")
    async for armed in drone.telemetry.armed():
        if not armed:
            print("\n✅ Landed and disarmed!")
            break
        await asyncio.sleep(0.5)
    
    print("\n" + "="*50)
    print("✅ MISSION COMPLETE")
    print("="*50)

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n\n⚠️  Script interrupted by user")
        sys.exit(0)
