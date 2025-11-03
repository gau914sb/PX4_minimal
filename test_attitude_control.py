#!/usr/bin/env python3
"""
Test script for PX4 attitude control
Tests the mc_att_control and mc_rate_control modules
"""

import time
import math
from pymavlink import mavutil

# Constants for SET_ATTITUDE_TARGET message
# Bit mask to indicate which fields should be ignored by the vehicle
# Bit 1: Ignore body roll rate
# Bit 2: Ignore body pitch rate
# Bit 3: Ignore body yaw rate
ATTITUDE_TARGET_TYPEMASK = 0b00000111  # Ignore body rates, use attitude + thrust

# Connect to PX4
print("Connecting to PX4...")
connection_strings = [
    'udp:127.0.0.1:14540',
    'udpin:localhost:14540',
    'udp:localhost:14540',
]

master = None
for conn_str in connection_strings:
    try:
        print(f"Trying {conn_str}...")
        master = mavutil.mavlink_connection(conn_str)
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=5)
        print(f"✓ Connected! System ID: {master.target_system}, Component ID: {master.target_component}")
        break
    except Exception as e:
        print(f"  Failed: {e}")
        continue

if master is None:
    print("\n❌ Could not connect to PX4!")
    print("\nTroubleshooting:")
    print("1. Make sure PX4 is running: ./build/px4_sitl_default/bin/px4")
    print("2. Check PX4 console for MAVLink messages (should show ports)")
    print("3. Try running in PX4 console: mavlink status")
    exit(1)

def is_armed():
    """Check if vehicle is already armed"""
    # Request HEARTBEAT message
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        # Check if MAV_MODE_FLAG_SAFETY_ARMED bit is set
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        return armed != 0
    return False

def arm():
    """Arm the vehicle"""
    # Check if already armed
    if is_armed():
        print("\n[1/4] Vehicle is already ARMED ✓")
        return

    print("\n[1/4] Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        0, 0, 0, 0, 0, 0
    )
    time.sleep(2)

    # Verify armed status
    if is_armed():
        print("✓ Vehicle ARMED successfully")
    else:
        print("⚠ Arm command sent, but vehicle may not be armed (check safety switch/pre-arm checks)")

def set_mode_offboard():
    """Set OFFBOARD mode (required for external attitude control)"""
    print("\n[2/5] Setting OFFBOARD mode...")
    print("  Sending initial setpoints to enable OFFBOARD...")

    # Pre-send setpoints at high rate (PX4 requires >2Hz stream before accepting OFFBOARD)
    # We send at 50Hz for 1 second to establish the stream
    start_time = time.time()
    for i in range(50):  # 50 messages at 50Hz = 1 second
        time_boot_ms = int((time.time() - start_time) * 1000) % 4294967295

        # Level attitude (quaternion for roll=0, pitch=0, yaw=0)
        q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z

        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            ATTITUDE_TARGET_TYPEMASK,  # Use attitude + thrust
            q,  # Quaternion
            0, 0, 0,  # Body roll rate, pitch rate, yaw rate (rad/s)
            0.5  # Thrust (0-1, normalized)
        )
        time.sleep(0.02)  # 50 Hz (20ms interval)

    # Now switch to OFFBOARD mode (custom_mode = 6 for PX4)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6,  # PX4 OFFBOARD mode
        0, 0, 0, 0, 0
    )

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        print(f"✓ OFFBOARD mode command sent")
    else:
        print(f"⚠ No acknowledgment for OFFBOARD mode")

    time.sleep(0.1)

def takeoff(altitude=2.5):
    """Command takeoff by sending attitude + thrust commands"""
    print(f"\n[3/5] Taking off to {altitude}m using attitude control...")
    print("  Sending level attitude with increasing thrust...")

    # Send level attitude with hover thrust for 5 seconds at 50Hz
    start_time = time.time()
    while time.time() - start_time < 5:
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Level attitude (no rotation)
        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            0b00000111,  # ignore body rates
            [1, 0, 0, 0],  # quaternion (level)
            0, 0, 0,
            0.6  # 60% thrust for takeoff
        )
        time.sleep(0.02)  # 50 Hz - CRITICAL for preventing timeout

    print("✓ Takeoff thrust applied")

def send_attitude_setpoint(roll, pitch, yaw, thrust):
    """Send attitude setpoint"""
    master.mav.set_attitude_target_send(
        int(time.time() * 1000),  # timestamp in ms
        master.target_system,
        master.target_component,
        0b00000111,  # type_mask (ignore body roll/pitch/yaw rates)
        [roll, pitch, yaw, 0],  # quaternion [w, x, y, z] - simplified, use Euler angles
        0, 0, 0,  # body roll/pitch/yaw rates
        thrust
    )

def test_attitude_control():
    """Test attitude control by sending different attitude commands with real-time feedback"""
    print("\n[4/5] Testing attitude control with feedback...")
    print("Sending attitude setpoints for 10 seconds...")
    print("\nFormat: [Setpoint → Actual] Roll | Pitch | Yaw | Thrust")
    print("-" * 70)

    # Convert degrees to radians for quaternion
    test_duration = 10
    start_time = time.time()
    last_feedback_time = 0

    while time.time() - start_time < test_duration:
        elapsed = time.time() - start_time

        # Vary yaw angle over time
        yaw_deg = (elapsed * 36) % 360  # 36 deg/sec rotation
        yaw_rad = math.radians(yaw_deg)

        # Convert to quaternion (simplified - just yaw rotation)
        qw = math.cos(yaw_rad / 2)
        qx = 0
        qy = 0
        qz = math.sin(yaw_rad / 2)

        # Send attitude command
        thrust_setpoint = 0.5  # 50% thrust

        # Calculate timestamp in milliseconds (use elapsed time to avoid overflow)
        time_boot_ms = int((time.time() - start_time) * 1000) % 4294967295

        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            0b00000111,  # ignore body rates
            [qw, qx, qy, qz],
            0, 0, 0,
            thrust_setpoint
        )

        # Get feedback at 2Hz (every 0.5 seconds)
        if time.time() - last_feedback_time >= 0.5:
            # Try to get ATTITUDE message
            msg = master.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                actual_roll = math.degrees(msg.roll)
                actual_pitch = math.degrees(msg.pitch)
                actual_yaw = math.degrees(msg.yaw)

                # Get thrust feedback from SERVO_OUTPUT_RAW (actual motor outputs)
                servo_msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
                if servo_msg:
                    # Average the motor outputs (servo1-4) and normalize to 0-1 range
                    # PWM range is typically 1000-2000, so normalize around 1500
                    avg_pwm = (servo_msg.servo1_raw + servo_msg.servo2_raw +
                              servo_msg.servo3_raw + servo_msg.servo4_raw) / 4.0
                    # Convert PWM (1000-2000) to normalized thrust (0-1)
                    actual_thrust = (avg_pwm - 1000) / 1000.0
                else:
                    actual_thrust = 0.0

                print(f"  [{yaw_deg:6.1f}° → {actual_yaw:6.1f}°] "
                      f"R:{actual_roll:6.2f}° P:{actual_pitch:6.2f}° Y:{actual_yaw:6.2f}° "
                      f"T:{actual_thrust:.2f}")

                last_feedback_time = time.time()

        time.sleep(0.02)  # 50 Hz - CRITICAL: Must be >2Hz to prevent timeout

    print("-" * 70)
    print("✓ Attitude commands sent for 10 seconds with real-time feedback")

def monitor_attitude():
    """Monitor actual attitude and rates for detailed feedback while maintaining control"""
    print("\n[5/5] Monitoring detailed attitude response...")
    print("Format: Roll | Pitch | Yaw | Roll Rate | Pitch Rate | Yaw Rate")
    print("-" * 80)

    start_time = time.time()
    attitude_count = 0

    while time.time() - start_time < 5:
        # CRITICAL: Keep sending commands to prevent timeout
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Send level hover attitude
        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            0b00000111,  # ignore body rates
            [1, 0, 0, 0],  # quaternion (level)
            0, 0, 0,
            0.5  # 50% hover thrust
        )

        # Get attitude feedback
        msg = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.01)
        if msg:
            attitude_count += 1
            roll_deg = math.degrees(msg.roll)
            pitch_deg = math.degrees(msg.pitch)
            yaw_deg = math.degrees(msg.yaw)
            rollspeed_deg = math.degrees(msg.rollspeed)
            pitchspeed_deg = math.degrees(msg.pitchspeed)
            yawspeed_deg = math.degrees(msg.yawspeed)

            if attitude_count % 5 == 0:  # Print every 5th message for readability
                print(f"  R:{roll_deg:7.2f}° P:{pitch_deg:7.2f}° Y:{yaw_deg:7.2f}° | "
                      f"R':{rollspeed_deg:6.1f}°/s P':{pitchspeed_deg:6.1f}°/s Y':{yawspeed_deg:6.1f}°/s")

        time.sleep(0.02)  # 50 Hz

    print("-" * 80)
    print(f"✓ Received {attitude_count} attitude messages (~{attitude_count/5:.0f} Hz)")

def get_servo_output():
    """Get servo/actuator output to show thrust feedback"""
    print("\n[BONUS] Checking actuator outputs...")

    # Request SERVO_OUTPUT_RAW message
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
    if msg:
        print(f"  Servo outputs (PWM):")
        print(f"    Motor 1: {msg.servo1_raw}")
        print(f"    Motor 2: {msg.servo2_raw}")
        print(f"    Motor 3: {msg.servo3_raw}")
        print(f"    Motor 4: {msg.servo4_raw}")
        return True
    else:
        print("  ⚠ No servo output data available")
        return False

def disarm():
    """Disarm the vehicle"""
    # Check if already disarmed
    if not is_armed():
        print("\n[6/5] Vehicle is already DISARMED ✓")
        return

    print("\n[6/5] Disarming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # disarm
        0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

    # Verify disarmed status
    if not is_armed():
        print("✓ Vehicle DISARMED successfully")
    else:
        print("⚠ Disarm command sent, but vehicle may still be armed")

if __name__ == "__main__":

    print("\n" + "="*60)
    print("PX4 ATTITUDE CONTROL TEST")
    print("="*60)

    # Check initial armed state
    print("\nChecking initial vehicle state...")
    initial_armed = is_armed()
    print(f"Initial state: {'ARMED ⚠' if initial_armed else 'DISARMED ✓'}")

    try:
        arm()
        set_mode_offboard()
        takeoff(altitude=2.5)
        test_attitude_control()
        monitor_attitude()
        get_servo_output()
        disarm()

        print("\n" + "="*60)
        print("✓ TEST COMPLETE!")
        print("="*60)
        print("\nSummary:")
        print("  ✓ Attitude setpoints sent with real-time feedback")
        print("  ✓ Actual attitude and rates monitored")
        print("  ✓ Servo/motor outputs checked")
        print("\nIf you see changing attitude values above, the attitude controller is working!")
        print("Check the PX4 console for mc_att_control and mc_rate_control status.")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        disarm()
    except Exception as e:
        print(f"\n\nError during test: {e}")
        import traceback
        traceback.print_exc()
        disarm()
