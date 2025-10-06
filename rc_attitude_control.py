#!/usr/bin/env python3
"""
PX4 RC Override Attitude Control
Controls drone attitude using RC channel override
This should work with attitude-only configuration!
"""

import time
import math
from pymavlink import mavutil

class RCAttitudeController:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        """Initialize MAVLink connection to PX4"""
        print(f"Connecting to PX4 on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"✅ Connected to system {self.master.target_system}, component {self.master.target_component}")

    def send_rc_attitude(self, roll_percent, pitch_percent, yaw_percent, throttle_percent):
        """Send RC attitude commands (percentage -100 to +100)"""
        try:
            # Convert percentages to PWM values (1000-2000 microseconds)
            roll_pwm = int(1500 + roll_percent * 4)      # ±400 range
            pitch_pwm = int(1500 + pitch_percent * 4)    # ±400 range
            yaw_pwm = int(1500 + yaw_percent * 4)        # ±400 range
            throttle_pwm = int(1000 + throttle_percent * 10)  # 0-1000 range

            # Send RC override
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                roll_pwm,     # Channel 1 (Roll)
                pitch_pwm,    # Channel 2 (Pitch)
                throttle_pwm, # Channel 3 (Throttle)
                yaw_pwm,      # Channel 4 (Yaw)
                0, 0, 0, 0    # Channels 5-8
            )

            print(f"🎮 Roll={roll_percent:+3.0f}%, Pitch={pitch_percent:+3.0f}%, Yaw={yaw_percent:+3.0f}%, Throttle={throttle_percent:3.0f}%")
        except Exception as e:
            print(f"❌ Error: {e}")

    def stop_override(self):
        """Stop RC override and return control"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
        )
        print("🔄 RC override stopped")

def main():
    print("=" * 60)
    print("PX4 RC ATTITUDE CONTROL - Works with Attitude-Only Mode!")
    print("=" * 60)
    print("🎮 Using RC channel override for attitude control")
    print("🚁 This bypasses OFFBOARD mode completely")
    print("📶 Make sure drone is flying first!")
    print("")

    try:
        controller = RCAttitudeController()

        # Hover throttle (60%)
        hover_throttle = 60

        print("🟢 STARTING RC ATTITUDE CONTROL!")
        print("⚠️  Make sure drone is already flying!")
        time.sleep(3)

        print("\n📍 Phase 1: STABLE HOVER (3 seconds)")
        for i in range(30):
            controller.send_rc_attitude(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n↔️  Phase 2: ROLL LEFT (3 seconds) - SHOULD MOVE!")
        for i in range(30):
            controller.send_rc_attitude(-30, 0, 0, hover_throttle)  # 30% left roll
            time.sleep(0.1)

        print("\n📍 Phase 3: CENTER (2 seconds)")
        for i in range(20):
            controller.send_rc_attitude(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n↔️  Phase 4: ROLL RIGHT (3 seconds) - SHOULD MOVE!")
        for i in range(30):
            controller.send_rc_attitude(30, 0, 0, hover_throttle)  # 30% right roll
            time.sleep(0.1)

        print("\n📍 Phase 5: CENTER (2 seconds)")
        for i in range(20):
            controller.send_rc_attitude(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n⬆️  Phase 6: PITCH FORWARD (3 seconds) - SHOULD MOVE!")
        for i in range(30):
            controller.send_rc_attitude(0, 30, 0, hover_throttle)  # 30% forward pitch
            time.sleep(0.1)

        print("\n📍 Phase 7: CENTER (2 seconds)")
        for i in range(20):
            controller.send_rc_attitude(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n🔄 Phase 8: YAW LEFT (5 seconds) - SHOULD SPIN!")
        for i in range(50):
            controller.send_rc_attitude(0, 0, -40, hover_throttle)  # 40% left yaw
            time.sleep(0.1)

        print("\n📍 Phase 9: FINAL HOVER (3 seconds)")
        for i in range(30):
            controller.send_rc_attitude(0, 0, 0, hover_throttle)
            time.sleep(0.1)

        print("\n✅ SEQUENCE COMPLETE!")
        controller.stop_override()
        print("🎮 Manual control returned")

    except KeyboardInterrupt:
        print("\n⏹️  STOPPED by user")
        if 'controller' in locals():
            controller.stop_override()

    except Exception as e:
        print(f"❌ Error: {e}")
        if 'controller' in locals():
            controller.stop_override()

if __name__ == "__main__":
    main()
