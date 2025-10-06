#!/usr/bin/env python3
"""
PX4 Attitude Command Script - SLOW VISIBLE VERSION
Sends SLOW attitude commands to flying drone via MAVLink
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
        """Send attitude command to drone"""
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
            print(f"🚁 Roll={roll_deg:+6.1f}°, Pitch={pitch_deg:+6.1f}°, Yaw={yaw_deg:+6.1f}°, Thrust={thrust:.2f}")
        except Exception as e:
            print(f"❌ Error: {e}")

def main():
    print("=" * 60)
    print("PX4 SLOW ATTITUDE COMMANDS - EASY TO OBSERVE")
    print("=" * 60)
    print("🚨 QGroundControl will lose control during execution!")
    print("🚁 Watch the drone carefully for movements")
    print("⏹️  Control returns when script ends")
    print("")

    try:
        commander = AttitudeCommander()
        hover_thrust = 0.6

        print("\n🟢 STARTING - Watch the drone!")

        print("\n📍 Phase 1: HOVER (3 seconds)")
        for i in range(30):
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n↔️  Phase 2: SLOW ROLL LEFT (5 seconds)")
        for i in range(50):
            roll_angle = -20  # Constant 20° left roll
            commander.send_attitude_command(roll_angle, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 3: RETURN TO CENTER (2 seconds)")
        for i in range(20):
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n↔️  Phase 4: SLOW ROLL RIGHT (5 seconds)")
        for i in range(50):
            roll_angle = 20  # Constant 20° right roll
            commander.send_attitude_command(roll_angle, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 5: RETURN TO CENTER (2 seconds)")
        for i in range(20):
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n⬆️  Phase 6: PITCH FORWARD (5 seconds)")
        for i in range(50):
            pitch_angle = 15  # Constant 15° forward pitch
            commander.send_attitude_command(0, pitch_angle, 0, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 7: RETURN TO CENTER (2 seconds)")
        for i in range(20):
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n🔄 Phase 8: SLOW YAW LEFT (5 seconds)")
        for i in range(50):
            yaw_angle = -30  # Constant 30° left yaw
            commander.send_attitude_command(0, 0, yaw_angle, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 9: FINAL HOVER (3 seconds)")
        for i in range(30):
            commander.send_attitude_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n✅ SEQUENCE COMPLETE!")
        print("🎮 QGroundControl should regain control now")

    except KeyboardInterrupt:
        print("\n⏹️  STOPPED by user")
        print("🔄 Returning control to QGroundControl...")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
