#!/usr/bin/env python3
"""
PX4 Manual Stick Commands
Simulates RC stick inputs to control drone
This should definitely work since manual control works!
"""

import time
import math
from pymavlink import mavutil

class StickCommander:
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

    def send_stick_command(self, roll_stick, pitch_stick, yaw_stick, throttle_stick):
        """Send RC stick inputs (values -1000 to +1000, center=0)"""
        try:
            # Convert to RC channel values (1000-2000 microseconds)
            roll_pwm = int(1500 + roll_stick)      # Channel 1 (Roll)
            pitch_pwm = int(1500 + pitch_stick)    # Channel 2 (Pitch)
            throttle_pwm = int(1000 + throttle_stick)  # Channel 3 (Throttle, 1000-2000)
            yaw_pwm = int(1500 + yaw_stick)        # Channel 4 (Yaw)

            # Send RC override
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                roll_pwm,     # Channel 1
                pitch_pwm,    # Channel 2
                throttle_pwm, # Channel 3
                yaw_pwm,      # Channel 4
                0,            # Channel 5
                0,            # Channel 6
                0,            # Channel 7
                0             # Channel 8
            )

            print(f"📡 Roll={roll_stick:+4.0f}, Pitch={pitch_stick:+4.0f}, Yaw={yaw_stick:+4.0f}, Throttle={throttle_stick:+4.0f}")
        except Exception as e:
            print(f"❌ Error: {e}")

    def stop_override(self):
        """Stop RC override and return control"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
        )
        print("🔄 RC override stopped - returning control")

def main():
    print("=" * 60)
    print("PX4 MANUAL STICK COMMANDS - RC Override")
    print("=" * 60)
    print("🎮 Simulating RC stick inputs")
    print("🚁 This should move the drone like manual control")
    print("⏹️  Control returns when script ends")
    print("")

    try:
        commander = StickCommander()

        # Hover throttle (adjust based on your drone)
        hover_throttle = 600  # About 60% throttle

        print("\n🟢 STARTING - Taking RC control!")

        print("\n📍 Phase 1: HOVER (3 seconds)")
        for i in range(30):
            commander.send_stick_command(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n↔️  Phase 2: ROLL LEFT (3 seconds) - SHOULD MOVE!")
        for i in range(30):
            commander.send_stick_command(-300, 0, 0, hover_throttle)  # 30% left roll
            time.sleep(0.1)

        print("\n📍 Phase 3: CENTER STICKS (2 seconds)")
        for i in range(20):
            commander.send_stick_command(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n↔️  Phase 4: ROLL RIGHT (3 seconds) - SHOULD MOVE!")
        for i in range(30):
            commander.send_stick_command(300, 0, 0, hover_throttle)  # 30% right roll
            time.sleep(0.1)

        print("\n📍 Phase 5: CENTER STICKS (2 seconds)")
        for i in range(20):
            commander.send_stick_command(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n⬆️  Phase 6: PITCH FORWARD (3 seconds) - SHOULD MOVE!")
        for i in range(30):
            commander.send_stick_command(0, 300, 0, hover_throttle)  # 30% forward pitch
            time.sleep(0.1)

        print("\n📍 Phase 7: CENTER STICKS (2 seconds)")
        for i in range(20):
            commander.send_stick_command(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n🔄 Phase 8: YAW LEFT (5 seconds) - SHOULD SPIN!")
        for i in range(50):
            commander.send_stick_command(0, 0, -400, hover_throttle)  # 40% left yaw
            time.sleep(0.1)

        print("\n📍 Phase 9: FINAL HOVER (3 seconds)")
        for i in range(30):
            commander.send_stick_command(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n✅ SEQUENCE COMPLETE!")
        commander.stop_override()
        print("🎮 Manual control returned")

    except KeyboardInterrupt:
        print("\n⏹️  STOPPED by user")
        commander.stop_override()

    except Exception as e:
        print(f"❌ Error: {e}")
        if 'commander' in locals():
            commander.stop_override()

if __name__ == "__main__":
    main()
