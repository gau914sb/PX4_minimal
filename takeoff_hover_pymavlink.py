#!/usr/bin/env python3

"""
Simple PX4 Takeoff and Hover Script using pymavlink
Alternative script that uses the lower-level pymavlink library
"""

import time
import sys

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink
except ImportError:
    print("pymavlink not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pymavlink"])
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink


def wait_heartbeat(connection):
    """Wait for a heartbeat so we know the target system ID"""
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print(f"Heartbeat from system {connection.target_system}, component {connection.target_component}")


def arm_drone(connection):
    """Arm the drone"""
    print("Arming drone...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        0, 0, 0, 0, 0, 0
    )
    
    # Wait for arming confirmation
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
    if msg and msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
            print("-- Armed successfully")
        else:
            print(f"-- Arming failed: {msg.result}")
            return False
    return True


def takeoff(connection, altitude=2.5):
    """Command takeoff to specified altitude"""
    print(f"Taking off to {altitude}m...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0,  # pitch
        0, 0, 0, 0, 0,
        altitude  # altitude
    )
    
    # Wait for takeoff confirmation
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
    if msg and msg.command == mavlink.MAV_CMD_NAV_TAKEOFF:
        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
            print("-- Takeoff command accepted")
        else:
            print(f"-- Takeoff failed: {msg.result}")
            return False
    return True


def wait_for_altitude(connection, target_altitude, tolerance=0.5):
    """Wait until drone reaches target altitude"""
    print(f"Waiting to reach {target_altitude}m altitude...")
    
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Convert mm to m
            print(f"Current altitude: {current_altitude:.1f}m")
            
            if abs(current_altitude - target_altitude) < tolerance:
                print(f"-- Reached target altitude: {current_altitude:.1f}m")
                break
        time.sleep(0.5)


def land(connection):
    """Command landing"""
    print("Landing...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavlink.MAV_CMD_NAV_LAND,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0
    )
    
    # Wait for landing confirmation
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
    if msg and msg.command == mavlink.MAV_CMD_NAV_LAND:
        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
            print("-- Landing command accepted")
        else:
            print(f"-- Landing failed: {msg.result}")
            return False
    return True


def disarm_drone(connection):
    """Disarm the drone"""
    print("Disarming drone...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # disarm
        0, 0, 0, 0, 0, 0
    )
    
    # Wait for disarming confirmation
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
    if msg and msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
            print("-- Disarmed successfully")
        else:
            print(f"-- Disarming failed: {msg.result}")
            return False
    return True


def main():
    # Connect to PX4
    print("Connecting to PX4...")
    connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    
    # Wait for heartbeat
    wait_heartbeat(connection)
    
    # Set mode to GUIDED (required for takeoff)
    print("Setting mode to GUIDED...")
    connection.mav.set_mode_send(
        connection.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode for PX4
    )
    
    time.sleep(2)
    
    # Arm the drone
    if not arm_drone(connection):
        print("Failed to arm drone. Exiting.")
        return
    
    time.sleep(2)
    
    # Takeoff
    if not takeoff(connection, 2.5):
        print("Takeoff failed. Exiting.")
        return
    
    # Wait for takeoff to complete
    wait_for_altitude(connection, 2.5)
    
    # Hover for 30 seconds
    print("Hovering for 30 seconds...")
    start_time = time.time()
    while time.time() - start_time < 30:
        # Monitor altitude during hover
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            current_altitude = msg.relative_alt / 1000.0
            print(f"Hovering at {current_altitude:.1f}m - {30 - (time.time() - start_time):.0f}s remaining")
        time.sleep(2)
    
    # Land
    if not land(connection):
        print("Landing command failed.")
        return
    
    # Wait for landing
    print("Waiting for landing...")
    while True:
        msg = connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
        if msg and msg.landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("-- Landed successfully")
            break
        time.sleep(0.5)
    
    time.sleep(2)
    
    # Disarm
    disarm_drone(connection)
    
    print("Mission complete!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
