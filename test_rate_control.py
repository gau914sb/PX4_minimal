#!/usr/bin/env python3
"""
Test script for PX4 rate control (bypassing attitude controller)
Sends angular velocity commands directly to mc_rate_control
"""

import time
import math
from pymavlink import mavutil

# Constants for SET_ATTITUDE_TARGET message
# Bit mask to indicate which fields should be ignored by the vehicle
# Bit 7: Ignore attitude (use body rates instead)
RATE_TARGET_TYPEMASK = 0b10000000  # Use body rates, ignore attitude

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
    print("1. Make sure PX4 is running: ./run_px4.sh")
    print("2. Check PX4 console for MAVLink messages")
    print("3. Try running in PX4 console: mavlink status")
    exit(1)

def is_armed():
    """Check if vehicle is already armed"""
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        return armed != 0
    return False

def arm():
    """Arm the vehicle"""
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

    if is_armed():
        print("✓ Vehicle ARMED successfully")
    else:
        print("⚠ Arm command sent, but vehicle may not be armed")

def set_mode_offboard():
    """Set OFFBOARD mode (required for external rate control)"""
    print("\n[2/4] Setting OFFBOARD mode...")
    print("  Sending initial rate setpoints to enable OFFBOARD...")
    print(f"  Using type_mask = {RATE_TARGET_TYPEMASK} (0b{RATE_TARGET_TYPEMASK:08b})")
    print(f"  Bit 7 (ignore attitude): {bool(RATE_TARGET_TYPEMASK & 128)}")
    print(f"  Bit 0 (ignore roll rate): {bool(RATE_TARGET_TYPEMASK & 1)}")
    print(f"  Bit 1 (ignore pitch rate): {bool(RATE_TARGET_TYPEMASK & 2)}")
    print(f"  Bit 2 (ignore yaw rate): {bool(RATE_TARGET_TYPEMASK & 4)}")

    # Pre-send setpoints at high rate (PX4 requires >2Hz stream before accepting OFFBOARD)
    start_time = time.time()
    for i in range(50):  # 50 messages at 50Hz = 1 second
        time_boot_ms = int((time.time() - start_time) * 1000) % 4294967295

        # Zero rates with hover thrust
        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            RATE_TARGET_TYPEMASK,  # Use rates, ignore attitude
            [1, 0, 0, 0],  # Quaternion (ignored when using rates)
            0, 0, 0,  # Zero body roll/pitch/yaw rates (rad/s)
            0.5  # Thrust (0-1, normalized)
        )
        time.sleep(0.02)  # 50 Hz

    # Now switch to OFFBOARD mode
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
        print(f"✓ OFFBOARD mode set")
    else:
        print(f"⚠ No acknowledgment for OFFBOARD mode")

    time.sleep(0.1)

def get_attitude_and_rates():
    """Get current attitude and angular rates"""
    msg = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
    if msg:
        return {
            'roll': math.degrees(msg.roll),
            'pitch': math.degrees(msg.pitch),
            'yaw': math.degrees(msg.yaw),
            'rollspeed': math.degrees(msg.rollspeed),
            'pitchspeed': math.degrees(msg.pitchspeed),
            'yawspeed': math.degrees(msg.yawspeed)
        }
    return None

def get_motor_outputs():
    """Get current motor PWM outputs"""
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=False, timeout=0.1)
    if msg:
        return [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
    return None

def test_rate_control():
    """Test rate control by sending different rate commands with real-time feedback"""
    print("\n[3/4] Testing rate control with feedback...")

    # Test 1: Hover with zero rates
    print("\n=== TEST 1: Hover (zero rates, 3 seconds) ===")
    print("Format: [Setpoint rates → Actual rates] | Attitude | Motors")
    print("-" * 80)

    start_time = time.time()
    test_duration = 3
    message_count = 0

    while time.time() - start_time < test_duration:
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Zero angular rates
        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = 0.0
        thrust = 0.5

        # Send rate command
        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            RATE_TARGET_TYPEMASK,
            [1, 0, 0, 0],  # quaternion (ignored)
            roll_rate, pitch_rate, yaw_rate,
            thrust
        )

        # Get feedback
        feedback = get_attitude_and_rates()
        motors = get_motor_outputs()

        if feedback and message_count % 5 == 0:  # Print every 5th message (10 Hz)
            print(f"[{elapsed:.1f}s] Setpoint: R={math.degrees(roll_rate):5.1f}° P={math.degrees(pitch_rate):5.1f}° Y={math.degrees(yaw_rate):5.1f}° → "
                  f"Actual: R={feedback['rollspeed']:5.1f}°/s P={feedback['pitchspeed']:5.1f}°/s Y={feedback['yawspeed']:5.1f}°/s | "
                  f"Att: R={feedback['roll']:5.1f}° P={feedback['pitch']:5.1f}° Y={feedback['yaw']:5.1f}° | "
                  f"Motors: {motors if motors else 'N/A'}")

        message_count += 1
        time.sleep(0.02)  # 50 Hz

    print(f"✓ Test 1 complete ({message_count} messages sent)")

    # Test 2: Constant yaw rate
    print("\n=== TEST 2: Constant Yaw Rate (30 deg/s, 6 seconds) ===")
    yaw_rate_deg = 30.0
    yaw_rate_rad = math.radians(yaw_rate_deg)

    start_time = time.time()
    test_duration = 6
    message_count = 0
    yaw_measurements = []

    while time.time() - start_time < test_duration:
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Constant yaw rate
        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            RATE_TARGET_TYPEMASK,
            [1, 0, 0, 0],
            0, 0, yaw_rate_rad,
            0.5
        )

        # Get feedback
        feedback = get_attitude_and_rates()
        motors = get_motor_outputs()

        if feedback:
            yaw_measurements.append({
                'time': elapsed,
                'setpoint': yaw_rate_deg,
                'actual': feedback['yawspeed'],
                'yaw_angle': feedback['yaw']
            })

            if message_count % 10 == 0:  # Print every 10th message (5 Hz)
                error = abs(yaw_rate_deg - feedback['yawspeed'])
                print(f"[{elapsed:.1f}s] Yaw rate: {yaw_rate_deg:5.1f}°/s → {feedback['yawspeed']:5.1f}°/s "
                      f"(error: {error:.1f}°/s) | Yaw: {feedback['yaw']:6.1f}° | Motors: {motors if motors else 'N/A'}")

        message_count += 1
        time.sleep(0.02)  # 50 Hz

    print(f"✓ Test 2 complete ({message_count} messages sent)")

    # Analyze yaw rate tracking
    if yaw_measurements:
        errors = [abs(m['setpoint'] - m['actual']) for m in yaw_measurements]
        avg_error = sum(errors) / len(errors)
        max_error = max(errors)
        print(f"  Average tracking error: {avg_error:.2f} deg/s")
        print(f"  Maximum tracking error: {max_error:.2f} deg/s")

    # Test 3: Varying yaw rate (sine wave)
    print("\n=== TEST 3: Varying Yaw Rate (Sine wave, 6 seconds) ===")

    start_time = time.time()
    test_duration = 6
    message_count = 0

    while time.time() - start_time < test_duration:
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Sinusoidal yaw rate: amplitude=45 deg/s, period=3 seconds
        yaw_rate_deg = 45.0 * math.sin(2 * math.pi * elapsed / 3.0)
        yaw_rate_rad = math.radians(yaw_rate_deg)

        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            RATE_TARGET_TYPEMASK,
            [1, 0, 0, 0],
            0, 0, yaw_rate_rad,
            0.5
        )

        feedback = get_attitude_and_rates()

        if feedback and message_count % 10 == 0:
            print(f"[{elapsed:.1f}s] Yaw rate: {yaw_rate_deg:6.1f}°/s → {feedback['yawspeed']:6.1f}°/s | "
                  f"Yaw: {feedback['yaw']:6.1f}°")

        message_count += 1
        time.sleep(0.02)  # 50 Hz

    print(f"✓ Test 3 complete ({message_count} messages sent)")

    # Test 4: Roll and pitch rates
    print("\n=== TEST 4: Roll and Pitch Rates (20 deg/s each, 4 seconds) ===")

    start_time = time.time()
    test_duration = 4
    message_count = 0

    while time.time() - start_time < test_duration:
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Constant roll and pitch rates
        roll_rate_rad = math.radians(20.0)
        pitch_rate_rad = math.radians(20.0)

        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            RATE_TARGET_TYPEMASK,
            [1, 0, 0, 0],
            roll_rate_rad, pitch_rate_rad, 0,
            0.5
        )

        feedback = get_attitude_and_rates()

        if feedback and message_count % 10 == 0:
            print(f"[{elapsed:.1f}s] Rates - R: {feedback['rollspeed']:5.1f}°/s P: {feedback['pitchspeed']:5.1f}°/s Y: {feedback['yawspeed']:5.1f}°/s | "
                  f"Att - R: {feedback['roll']:5.1f}° P: {feedback['pitch']:5.1f}° Y: {feedback['yaw']:6.1f}°")

        message_count += 1
        time.sleep(0.02)  # 50 Hz

    print(f"✓ Test 4 complete ({message_count} messages sent)")

    # Return to hover
    print("\n=== Returning to hover (zero rates, 2 seconds) ===")
    start_time = time.time()
    message_count = 0

    while time.time() - start_time < 2:
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        master.mav.set_attitude_target_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            RATE_TARGET_TYPEMASK,
            [1, 0, 0, 0],
            0, 0, 0,
            0.5
        )

        message_count += 1
        time.sleep(0.02)

    print(f"✓ Hover complete ({message_count} messages sent)")

def read_final_status():
    """Read final motor outputs and battery status"""
    print("\n[4/4] Final Status Check...")

    # Get motor outputs
    motors = get_motor_outputs()
    if motors:
        print(f"✓ Motor PWM values: {motors}")
        thrust_values = [(pwm - 1000) / 1000.0 for pwm in motors]
        print(f"  Normalized thrust: {[f'{t:.3f}' for t in thrust_values]}")
    else:
        print("⚠ Could not read motor outputs")

    # Get battery status
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=2)
    if msg:
        print(f"✓ Battery: {msg.battery_remaining}% remaining")
    else:
        print("⚠ Could not read battery status")

# Main execution
print("=" * 50)
print("PX4 Rate Control Test")
print("=" * 50)
print("\nThis test sends angular velocity commands directly")
print("to the rate controller, bypassing the attitude controller.")
print("\nMake sure PX4 is running with:")
print("  - mc_rate_control enabled")
print("  - control_allocator enabled")
print("  - SIH simulator running")
print("=" * 50)

arm()
set_mode_offboard()
test_rate_control()
read_final_status()

print("\n" + "=" * 50)
print("Test complete!")
print("=" * 50)
