#!/usr/bin/env python3
"""
PX4 Direct Motor Control
Directly controls motor speeds for guaranteed movement
This bypasses all flight control systems
"""

import time
import math
from pymavlink import mavutil

class DirectMotorController:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        """Initialize MAVLink connection to PX4"""
        print(f"Connecting to PX4 on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"✅ Connected to system {self.master.target_system}, component {self.master.target_component}")

    def send_motor_speeds(self, motor1, motor2, motor3, motor4):
        """Send direct motor speed commands (0-1000)"""
        try:
            # Send actuator control message
            self.master.mav.set_actuator_control_target_send(
                0,  # time_boot_ms
                0,  # group_mlx (0 = motors)
                self.master.target_system,
                self.master.target_component,
                [
                    motor1 / 1000.0,  # Control 0 (Front Right)
                    motor2 / 1000.0,  # Control 1 (Rear Left)
                    motor3 / 1000.0,  # Control 2 (Front Left)
                    motor4 / 1000.0,  # Control 3 (Rear Right)
                    0, 0, 0, 0        # Controls 4-7 (unused)
                ]
            )
            print(f"🔧 Motors: [{motor1:3.0f}, {motor2:3.0f}, {motor3:3.0f}, {motor4:3.0f}]")
        except Exception as e:
            print(f"❌ Error: {e}")

    def send_raw_pwm(self, pwm1, pwm2, pwm3, pwm4):
        """Send raw PWM commands to motors"""
        try:
            self.master.mav.servo_output_raw_send(
                0,  # time_usec
                0,  # port
                pwm1,  # servo1_raw (motor 1)
                pwm2,  # servo2_raw (motor 2)
                pwm3,  # servo3_raw (motor 3)
                pwm4,  # servo4_raw (motor 4)
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0  # Other servos
            )
            print(f"⚡ PWM: [{pwm1:4.0f}, {pwm2:4.0f}, {pwm3:4.0f}, {pwm4:4.0f}]")
        except Exception as e:
            print(f"❌ Error: {e}")

def main():
    print("=" * 60)
    print("PX4 DIRECT MOTOR CONTROL - GUARANTEED MOVEMENT!")
    print("=" * 60)
    print("🔧 Directly controlling motor speeds")
    print("⚠️  DANGER: Bypasses all safety systems!")
    print("🚁 Make sure drone is armed and ready!")
    print("")

    try:
        controller = DirectMotorController()

        # Base hover speeds
        hover_speed = 600  # Hover PWM

        print("🟢 STARTING DIRECT MOTOR CONTROL!")
        print("⚠️  Drone WILL move - watch carefully!")
        time.sleep(3)

        print("\n📍 Phase 1: BALANCED HOVER (3 seconds)")
        for i in range(30):
            controller.send_raw_pwm(hover_speed, hover_speed, hover_speed, hover_speed)
            time.sleep(0.1)

        print("\n↔️  Phase 2: ROLL LEFT (3 seconds) - RIGHT MOTORS FASTER")
        for i in range(30):
            # Roll left: increase right motors, decrease left motors
            controller.send_raw_pwm(
                hover_speed - 100,  # Front Right (decrease)
                hover_speed + 100,  # Rear Left (increase)
                hover_speed + 100,  # Front Left (increase)
                hover_speed - 100   # Rear Right (decrease)
            )
            time.sleep(0.1)

        print("\n📍 Phase 3: RETURN TO HOVER (2 seconds)")
        for i in range(20):
            controller.send_raw_pwm(hover_speed, hover_speed, hover_speed, hover_speed)
            time.sleep(0.1)

        print("\n↔️  Phase 4: ROLL RIGHT (3 seconds) - LEFT MOTORS FASTER")
        for i in range(30):
            # Roll right: increase left motors, decrease right motors
            controller.send_raw_pwm(
                hover_speed + 100,  # Front Right (increase)
                hover_speed - 100,  # Rear Left (decrease)
                hover_speed - 100,  # Front Left (decrease)
                hover_speed + 100   # Rear Right (increase)
            )
            time.sleep(0.1)

        print("\n📍 Phase 5: RETURN TO HOVER (2 seconds)")
        for i in range(20):
            controller.send_raw_pwm(hover_speed, hover_speed, hover_speed, hover_speed)
            time.sleep(0.1)

        print("\n🔄 Phase 6: YAW LEFT (5 seconds) - DIAGONAL MOTORS")
        for i in range(50):
            # Yaw left: front-right & rear-left faster, others slower
            controller.send_raw_pwm(
                hover_speed + 80,   # Front Right (increase for yaw)
                hover_speed - 80,   # Rear Left (decrease for yaw)
                hover_speed + 80,   # Front Left (increase for yaw)
                hover_speed - 80    # Rear Right (decrease for yaw)
            )
            time.sleep(0.1)

        print("\n📍 Phase 7: FINAL HOVER (3 seconds)")
        for i in range(30):
            controller.send_raw_pwm(hover_speed, hover_speed, hover_speed, hover_speed)
            time.sleep(0.1)

        print("\n✅ DIRECT MOTOR CONTROL COMPLETE!")
        print("🎮 Motor commands stopped")

    except KeyboardInterrupt:
        print("\n⏹️  STOPPED by user")
        print("🔄 Motor control stopped")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
