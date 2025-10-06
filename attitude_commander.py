#!/usr/bin/env python3
"""
PX4 Attitude Command Script
Sends attitude commands to flying drone via MAVLink
Requires: pymavlink library (pip install pymavlink)
"""

import time
import math
from pymavlink import mavutil

class AttitudeCommander:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        """Initialize MAVLink connection to PX4"""
        print(f"Connecting to PX4 on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")

        # Request data streams
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # Hz
            1    # Enable
        )

        # Switch to offboard mode for attitude control
        print("Switching to OFFBOARD mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6  # OFFBOARD mode
        )

    def send_attitude_command(self, roll_deg, pitch_deg, yaw_deg, thrust):
        """
        Send attitude command to drone

        Args:
            roll_deg: Roll angle in degrees (-30 to 30)
            pitch_deg: Pitch angle in degrees (-30 to 30)
            yaw_deg: Yaw angle in degrees (-180 to 180)
            thrust: Thrust value (0.0 to 1.0)
        """
        # Convert degrees to radians
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        # Send attitude setpoint
        try:
            self.master.mav.set_attitude_target_send(
                0,  # time_boot_ms (not used)
                self.master.target_system,
                self.master.target_component,
                0b00000000,  # type_mask (ignore nothing)
                [
                    math.cos(yaw_rad/2) * math.cos(pitch_rad/2) * math.cos(roll_rad/2) +
                    math.sin(yaw_rad/2) * math.sin(pitch_rad/2) * math.sin(roll_rad/2),  # qw
                    math.cos(yaw_rad/2) * math.cos(pitch_rad/2) * math.sin(roll_rad/2) -
                    math.sin(yaw_rad/2) * math.sin(pitch_rad/2) * math.cos(roll_rad/2),  # qx
                    math.cos(yaw_rad/2) * math.sin(pitch_rad/2) * math.cos(roll_rad/2) +
                    math.sin(yaw_rad/2) * math.cos(pitch_rad/2) * math.sin(roll_rad/2),  # qy
                    math.sin(yaw_rad/2) * math.cos(pitch_rad/2) * math.cos(roll_rad/2) -
                    math.cos(yaw_rad/2) * math.sin(pitch_rad/2) * math.sin(roll_rad/2)   # qz
                ],
                0,  # body_roll_rate
                0,  # body_pitch_rate
                0,  # body_yaw_rate
                thrust
            )
            print(f"Sent: Roll={roll_deg:+6.1f}°, Pitch={pitch_deg:+6.1f}°, Yaw={yaw_deg:+6.1f}°, Thrust={thrust:.2f}")
        except Exception as e:
            print(f"Error sending attitude command: {e}")

    def check_connection(self):
        """Check if we're receiving data from PX4"""
        print("Checking connection...")
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            print(f"✅ Receiving heartbeat from system {msg.get_srcSystem()}")
            return True
        else:
            print("❌ No heartbeat received")
            return False

    def get_attitude(self):
        """Get current attitude from drone"""
        msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            roll_deg = math.degrees(msg.roll)
            pitch_deg = math.degrees(msg.pitch)
            yaw_deg = math.degrees(msg.yaw)
            return roll_deg, pitch_deg, yaw_deg
        return None, None, None

def main():
    print("=" * 60)
    print("PX4 ATTITUDE COMMAND SCRIPT")
    print("=" * 60)
    print("SAFETY: Make sure drone is airborne and in Manual/Stabilized mode!")
    print("This script sends attitude commands to your flying drone.")
    print("")

    try:
        # Connect to PX4
        commander = AttitudeCommander()

        # Check connection
        if not commander.check_connection():
            print("❌ Failed to establish connection with PX4")
            print("Make sure PX4 SITL is running and try different ports:")
            print("  - Port 14550 (main MAVLink)")
            print("  - Port 14540 (onboard MAVLink)")
            return

        print("\nStarting attitude command sequence...")
        print("Press Ctrl+C to stop")
        print("")

        # Hover position (neutral attitude)
        hover_thrust = 0.6  # Adjust based on your drone

        print("Phase 1: Hover (5 seconds)")
        for i in range(50):  # 5 seconds at 10Hz
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("Phase 2: Roll left and right (10 seconds)")
        for i in range(100):  # 10 seconds
            roll_angle = 15 * math.sin(2 * math.pi * i / 50)  # ±15° sine wave
            commander.send_attitude_command(roll_angle, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("Phase 3: Pitch forward and back (10 seconds)")
        for i in range(100):  # 10 seconds
            pitch_angle = 10 * math.sin(2 * math.pi * i / 50)  # ±10° sine wave
            commander.send_attitude_command(0, pitch_angle, 0, hover_thrust)
            time.sleep(0.1)

        print("Phase 4: Yaw rotation (10 seconds)")
        for i in range(100):  # 10 seconds
            yaw_angle = 30 * math.sin(2 * math.pi * i / 50)  # ±30° sine wave
            commander.send_attitude_command(0, 0, yaw_angle, hover_thrust)
            time.sleep(0.1)

        print("Phase 5: Combined movements (15 seconds)")
        for i in range(150):  # 15 seconds
            roll_angle = 10 * math.sin(2 * math.pi * i / 30)
            pitch_angle = 8 * math.cos(2 * math.pi * i / 40)
            yaw_angle = 20 * math.sin(2 * math.pi * i / 60)
            commander.send_attitude_command(roll_angle, pitch_angle, yaw_angle, hover_thrust)
            time.sleep(0.1)

        print("Phase 6: Return to hover (5 seconds)")
        for i in range(50):  # 5 seconds
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\nAttitude command sequence complete!")
        print("You can now take manual control or land the drone.")

    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C)")
        print("Sending neutral attitude command...")
        try:
            # Send neutral attitude for safety
            for _ in range(10):
                commander.send_attitude_command(0, 0, 0, 0.6)
                time.sleep(0.1)
        except:
            pass

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure PX4 is running and pymavlink is installed:")
        print("pip install pymavlink")

if __name__ == "__main__":
    main()
