#!/usr/bin/env python3
"""
Multi-Drone Controller for PX4 Gazebo Simulation
Controls multiple drones simultaneously with individual altitude targets
"""

import time
from pymavlink import mavutil
import threading

class DroneController:
    """Controller for a single drone instance"""

    def __init__(self, instance_id, mavlink_port):
        self.instance_id = instance_id
        self.mavlink_port = mavlink_port
        self.master = None
        self.armed = False
        self.offboard = False
        self.target_altitude = 0.0

    def connect(self):
        """Connect to the drone via MAVLink"""
        connection_string = f'udp:127.0.0.1:{self.mavlink_port}'
        print(f"Drone {self.instance_id}: Connecting to {connection_string}...")

        try:
            self.master = mavutil.mavlink_connection(connection_string)
            print(f"Drone {self.instance_id}: Waiting for heartbeat (timeout: 15s)...")
            self.master.wait_heartbeat(timeout=15)
            print(f"Drone {self.instance_id}: ✓ Connected! System ID: {self.master.target_system}")
            return True
        except Exception as e:
            print(f"Drone {self.instance_id}: ✗ Connection failed: {e}")
            return False

    def arm(self):
        """Arm the drone"""
        if not self.master:
            return False

        print(f"Drone {self.instance_id}: Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
        self.armed = True
        return True

    def set_offboard_mode(self):
        """Switch to OFFBOARD mode"""
        if not self.master:
            return False

        print(f"Drone {self.instance_id}: Setting OFFBOARD mode...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 1, 6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
        self.offboard = True
        return True

    def send_attitude_target(self, thrust, roll_rate=0, pitch_rate=0, yaw_rate=0):
        """Send attitude and thrust command"""
        if not self.master:
            return

        self.master.mav.set_attitude_target_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            0b10000000,  # type_mask (ignore attitude, use rates)
            [1, 0, 0, 0],  # quaternion (ignored)
            roll_rate,
            pitch_rate,
            yaw_rate,
            thrust
        )

    def control_loop(self, duration=30, target_alt=3.0):
        """Simple altitude hold using constant thrust"""
        if not self.armed or not self.offboard:
            print(f"Drone {self.instance_id}: Not ready for control")
            return

        print(f"Drone {self.instance_id}: Starting control loop (target: {target_alt}m, duration: {duration}s)")

        # Simple hover thrust
        hover_thrust = 0.65

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.send_attitude_target(thrust=hover_thrust)
            time.sleep(0.02)  # 50Hz control loop

        print(f"Drone {self.instance_id}: Control loop completed")


class MultiDroneManager:
    """Manages multiple drone instances"""

    def __init__(self, num_drones=2):
        self.num_drones = num_drones
        self.drones = []

        # Create drone controllers
        for i in range(num_drones):
            mavlink_port = 14540 + i
            drone = DroneController(instance_id=i, mavlink_port=mavlink_port)
            self.drones.append(drone)

    def connect_all(self):
        """Connect to all drones"""
        print(f"\n{'='*60}")
        print(f" Connecting to {self.num_drones} drones...")
        print(f"{'='*60}\n")

        connected_count = 0
        for drone in self.drones:
            if drone.connect():
                connected_count += 1
            else:
                print(f"Warning: Drone {drone.instance_id} not available")

        print(f"\n✓ {connected_count}/{self.num_drones} drones connected successfully!\n")

        if connected_count == 0:
            return False

        # Update drone list to only include connected drones
        self.drones = [d for d in self.drones if d.master is not None]
        self.num_drones = len(self.drones)
        return True

    def arm_all(self):
        """Arm all drones"""
        print("Arming all drones...")
        for drone in self.drones:
            if drone.master:
                drone.arm()
        time.sleep(1)
        print("All connected drones armed\n")

    def set_all_offboard(self):
        """Set all drones to OFFBOARD mode"""
        print("Setting all drones to OFFBOARD mode...")
        for drone in self.drones:
            if drone.master:
                drone.set_offboard_mode()
        time.sleep(1)
        print("All connected drones in OFFBOARD mode\n")

    def start_all_control_loops(self, duration=30):
        """Start control loops for all drones with different target altitudes"""
        print(f"\n{'='*60}")
        print(" Starting coordinated flight")
        print(f"{'='*60}\n")

        # Different altitude targets for each drone: 2m, 3m, 4m, etc.
        altitudes = [2.0 + i for i in range(self.num_drones)]

        threads = []
        for i, drone in enumerate(self.drones):
            target_alt = altitudes[i] if i < len(altitudes) else 3.0
            print(f"Drone {i}: Target altitude = {target_alt}m")
            thread = threading.Thread(
                target=drone.control_loop,
                args=(duration, target_alt)
            )
            thread.start()
            threads.append(thread)

        # Wait for all control loops to finish
        for thread in threads:
            thread.join()

        print(f"\n{'='*60}")
        print(" All flights completed")
        print(f"{'='*60}\n")


def main():
    """Main function"""
    import sys

    # Get number of drones from command line
    num_drones = int(sys.argv[1]) if len(sys.argv) > 1 else 2

    print(f"\n{'='*60}")
    print(f" PX4 Multi-Drone Controller")
    print(f" Number of drones: {num_drones}")
    print(f"{'='*60}\n")

    # Create manager
    manager = MultiDroneManager(num_drones=num_drones)

    # Connect to all drones
    if not manager.connect_all():
        print("Failed to connect to all drones. Exiting.")
        return

    # Arm all drones
    manager.arm_all()

    # Set OFFBOARD mode
    manager.set_all_offboard()

    # Start flight (30 seconds)
    # Each drone will have a different target altitude: 2m, 3m, 4m, etc.
    manager.start_all_control_loops(duration=30)

    print("\nTest completed!")


if __name__ == "__main__":
    main()
