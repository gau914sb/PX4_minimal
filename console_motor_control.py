#!/usr/bin/env python3
"""
PX4 Console Direct Commands
Uses PX4 console commands to directly control motors
This bypasses MAVLink entirely
"""

import time
from pymavlink import mavutil

def console_motor_control():
    print("=" * 60)
    print("PX4 CONSOLE DIRECT MOTOR CONTROL")
    print("=" * 60)
    print("🖥️  Sending direct commands to PX4 console")
    print("⚙️  This bypasses MAVLink entirely")
    print("")
    
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")
    
    def send_console_command(command):
        """Send command directly to PX4 console"""
        print(f"🖥️  Console: {command}")
        master.mav.serial_control_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
            mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
            0,
            (command + "\n").encode('utf-8')
        )
        time.sleep(0.5)
        
        # Try to read response
        for i in range(5):
            msg = master.recv_match(type='SERIAL_CONTROL', blocking=False)
            if msg:
                try:
                    response = msg.data.decode('utf-8').strip()
                    if response:
                        print(f"📤 Response: {response}")
                except:
                    pass
            time.sleep(0.1)
    
    print("🔍 Checking what modules are running...")
    send_console_command("ps")
    
    print("\n🔍 Checking mixer status...")
    send_console_command("mixer status")
    
    print("\n🔓 Arming via console...")
    send_console_command("commander arm")
    
    print("\n🧪 Testing PWM output directly...")
    send_console_command("pwm info")
    
    print("\n⚡ Setting PWM values directly...")
    send_console_command("pwm test -c 1234 -p 1200")
    time.sleep(2)
    send_console_command("pwm test -c 1234 -p 1400")
    time.sleep(2)
    send_console_command("pwm test -c 1234 -p 1600")
    time.sleep(2)
    
    print("\n🛑 Stopping PWM test...")
    send_console_command("pwm stop")
    
    print("\n🔓 Disarming...")
    send_console_command("commander disarm")
    
    print("\n✅ Console test complete!")
    print("📊 If you saw movement in jMAVSim during PWM test,")
    print("   then direct motor control works!")

if __name__ == "__main__":
    try:
        console_motor_control()
    except KeyboardInterrupt:
        print("\n⏹️  Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
