#!/usr/bin/env python3
"""
PX4 Angular Rate Command Script
Sends angular rate commands to flying drone via MAVLink
This should work better with attitude-only mode
"""

import time
import math
from pymavlink import mavutil

class RateCommander:
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

        # Switch to offboard mode for rate control
        print("Switching to OFFBOARD mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6  # OFFBOARD mode
        )

    def send_rate_command(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """Send angular rate command to drone"""
        try:
            # Use SET_ATTITUDE_TARGET with rates instead of attitude
            self.master.mav.set_attitude_target_send(
                0,  # time_boot_ms (not used)
                self.master.target_system,
                self.master.target_component,
                0b00000111,  # type_mask: ignore attitude quaternion (bits 0,1,2)
                [1, 0, 0, 0],  # quaternion (ignored due to type_mask)
                roll_rate,   # body_roll_rate (rad/s)
                pitch_rate,  # body_pitch_rate (rad/s)
                yaw_rate,    # body_yaw_rate (rad/s)
                thrust
            )
            print(f"🚁 Roll rate={roll_rate:+5.2f} rad/s, Pitch rate={pitch_rate:+5.2f} rad/s, Yaw rate={yaw_rate:+5.2f} rad/s, Thrust={thrust:.2f}")
        except Exception as e:
            print(f"❌ Error: {e}")

def main():
    print("=" * 60)
    print("PX4 ANGULAR RATE COMMANDS - Should Move Drone!")
    print("=" * 60)
    print("🚨 QGroundControl will lose control during execution!")
    print("🚁 Watch for OBVIOUS rotation movements")
    print("⏹️  Control returns when script ends")
    print("")

    try:
        commander = RateCommander()
        hover_thrust = 0.6

        print("\n🟢 STARTING - Watch the drone!")

        print("\n📍 Phase 1: STABLE HOVER (3 seconds)")
        for i in range(30):
            commander.send_rate_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n🔄 Phase 2: ROLL ROTATION LEFT (3 seconds) - SHOULD SEE ROLLING!")
        for i in range(30):
            roll_rate = -0.5  # -0.5 rad/s = ~28 deg/s roll left
            commander.send_rate_command(roll_rate, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 3: STOP ROLLING (2 seconds)")
        for i in range(20):
            commander.send_rate_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n🔄 Phase 4: ROLL ROTATION RIGHT (3 seconds) - SHOULD SEE ROLLING!")
        for i in range(30):
            roll_rate = 0.5  # +0.5 rad/s = ~28 deg/s roll right
            commander.send_rate_command(roll_rate, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 5: STOP ROLLING (2 seconds)")
        for i in range(20):
            commander.send_rate_command(0, 0, 0, hover_thrust)
            time.sleep(0.1)

        print("\n🔄 Phase 6: YAW ROTATION (5 seconds) - SHOULD SEE SPINNING!")
        for i in range(50):
            yaw_rate = 0.8  # 0.8 rad/s = ~45 deg/s yaw
            commander.send_rate_command(0, 0, yaw_rate, hover_thrust)
            time.sleep(0.1)

        print("\n📍 Phase 7: STOP ALL MOTION (3 seconds)")
        for i in range(30):
            commander.send_rate_command(0, 0, 0, hover_thrust)
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
