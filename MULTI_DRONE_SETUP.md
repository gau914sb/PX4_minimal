# Multi-Drone Simulation Setup

This documentation describes the multi-drone simulation system for PX4_minimal with Gazebo Harmonic.

## Overview

The system allows spawning and controlling multiple drones simultaneously using:
- **Gazebo Harmonic 8.10.0** for physics simulation and 3D visualization
- **PX4_minimal** custom lightweight autopilot (stripped-down PX4 with minimal_commander)
- **Python MAVLink** for coordinated multi-drone control

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Harmonic                          │
│  (Physics Simulation + 3D Visualization)                    │
│                                                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                │
│  │ Drone 0  │  │ Drone 1  │  │ Drone 2  │                │
│  │ (x500)   │  │ (x500)   │  │ (x500)   │                │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                │
└───────┼─────────────┼─────────────┼───────────────────────┘
        │             │             │
        │ MAVLink     │ MAVLink     │ MAVLink
        │ UDP         │ UDP         │ UDP
        │ :14540      │ :14541      │ :14542
        │             │             │
┌───────▼─────────────▼─────────────▼───────────────────────┐
│           PX4_minimal Instances (Processes)               │
│                                                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐              │
│  │Instance 0│  │Instance 1│  │Instance 2│              │
│  │ SysID: 1 │  │ SysID: 2 │  │ SysID: 3 │              │
│  ├──────────┤  ├──────────┤  ├──────────┤              │
│  │minimal_  │  │minimal_  │  │minimal_  │              │
│  │commander │  │commander │  │commander │              │
│  │EKF2      │  │EKF2      │  │EKF2      │              │
│  │mc_rate_  │  │mc_rate_  │  │mc_rate_  │              │
│  │control   │  │control   │  │control   │              │
│  │control_  │  │control_  │  │control_  │              │
│  │allocator │  │allocator │  │allocator │              │
│  └────▲─────┘  └────▲─────┘  └────▲─────┘              │
└───────┼─────────────┼─────────────┼───────────────────────┘
        │             │             │
        │ Commands    │ Commands    │ Commands
        │             │             │
┌───────▼─────────────▼─────────────▼───────────────────────┐
│      multi_drone_controller.py (Python)                   │
│                                                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐              │
│  │ Thread 0 │  │ Thread 1 │  │ Thread 2 │              │
│  │ Target:  │  │ Target:  │  │ Target:  │              │
│  │ 2m alt   │  │ 3m alt   │  │ 4m alt   │              │
│  └──────────┘  └──────────┘  └──────────┘              │
│                                                           │
│  Commands: ARM, OFFBOARD, Attitude Setpoints             │
└───────────────────────────────────────────────────────────┘
```

## Files

### 1. Launch Scripts

#### `launch_multi_drone.sh` (Custom Implementation)
**Location**: `/home/gaurav/PX4_minimal/launch_multi_drone.sh`

**Purpose**: Custom script to spawn multiple drones using PX4 multi-instance approach.

**Usage**:
```bash
./launch_multi_drone.sh [number_of_drones]

# Examples:
./launch_multi_drone.sh 2    # Launch 2 drones
./launch_multi_drone.sh 3    # Launch 3 drones
./launch_multi_drone.sh 5    # Launch 5 drones
```

**Key Features**:
- Kills existing PX4/Gazebo instances before starting
- Builds PX4_minimal before launching
- First drone (instance 0) spawns Gazebo world
- Additional drones connect to existing Gazebo using `PX4_GZ_STANDALONE=1`
- Drones positioned with 2.0m Y-axis spacing
- 20s wait for Gazebo initialization, 8s between additional spawns

**Environment Variables Used**:
- `PX4_SIM_MODEL=gz_x500` - Vehicle model type
- `PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"` - Spawn position
- `PX4_GZ_STANDALONE=1` - Connect to existing Gazebo (instances 1+)

**Instance Mapping**:
```
Instance 0 → System ID 1 → MAVLink UDP :14540 → Position (0, 0, 0.5)
Instance 1 → System ID 2 → MAVLink UDP :14541 → Position (0, 2, 0.5)
Instance 2 → System ID 3 → MAVLink UDP :14542 → Position (0, 4, 0.5)
...
```

---

#### `launch_multi_drone_official.sh` (PX4 Official Method)
**Location**: `/home/gaurav/PX4_minimal/launch_multi_drone_official.sh`

**Purpose**: Multi-drone launcher following official PX4 documentation.

**Usage**:
```bash
./launch_multi_drone_official.sh [number_of_drones]

# Examples:
./launch_multi_drone_official.sh 3
```

**Key Features**:
- Based on: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
- Uses `PX4_SYS_AUTOSTART=4001` (explicit x500 airframe)
- Instance numbering: 0, 1, 2, ...
- Logs output to `/tmp/drone_*.log` files
- First instance starts without `PX4_GZ_STANDALONE`

**Differences from Custom Script**:
- Explicitly sets `PX4_SYS_AUTOSTART=4001`
- Redirects output to log files
- Follows official PX4 naming conventions

**Log Files**:
```
/tmp/drone_0.log
/tmp/drone_1.log
/tmp/drone_2.log
```

---

### 2. Control Scripts

#### `multi_drone_controller.py` (MAVLink Controller)
**Location**: `/home/gaurav/PX4_minimal/multi_drone_controller.py`

**Purpose**: Python-based MAVLink controller for coordinated multi-drone operations.

**Usage**:
```bash
python3 multi_drone_controller.py [number_of_drones]

# Examples:
python3 multi_drone_controller.py 2    # Control 2 drones
python3 multi_drone_controller.py 3    # Control 3 drones
```

**Classes**:

##### `DroneController`
Single drone management class.

**Methods**:
- `__init__(instance_id, mavlink_port)` - Initialize drone controller
- `connect()` - Connect to PX4 via MAVLink UDP
  - Timeout: 15 seconds
  - Non-blocking: continues if connection fails
- `arm()` - Arm the drone motors
- `set_offboard_mode()` - Enable OFFBOARD flight mode
- `send_attitude_target(roll, pitch, yaw_rate, thrust)` - Send attitude setpoint
- `control_loop(duration, target_altitude)` - Main control loop for altitude hold

**Attributes**:
- `instance_id`: Drone instance number (0, 1, 2, ...)
- `mavlink_port`: UDP port (14540 + instance_id)
- `master`: MAVLink connection object
- `armed`: Arming status flag

##### `MultiDroneManager`
Multi-drone coordinator class.

**Methods**:
- `__init__(num_drones)` - Create DroneController instances
- `connect_all()` - Connect to all drones
  - Non-fatal failures: continues with available drones
  - Returns: True if at least 1 drone connected
- `arm_all()` - Arm all connected drones
- `set_all_offboard()` - Set OFFBOARD mode on all drones
- `start_all_control_loops(duration)` - Start threaded control loops
  - Creates separate thread for each drone
  - Different altitude targets: 2m, 3m, 4m, ...
  - Waits for all threads to complete

**Control Flow**:
```
1. Create manager with N drones
2. Connect to all drones (parallel attempts)
3. Arm all successfully connected drones
4. Set OFFBOARD mode on all drones
5. Start N control threads (one per drone)
   - Thread 0: Hold 2m altitude for 30s
   - Thread 1: Hold 3m altitude for 30s
   - Thread 2: Hold 4m altitude for 30s
6. Wait for all threads to complete
7. Exit (drones remain armed)
```

**MAVLink Commands Used**:
- `MAV_CMD_COMPONENT_ARM_DISARM` (400) - Arm/disarm motors
- `MAV_CMD_DO_SET_MODE` (176) - Set flight mode
- `SET_ATTITUDE_TARGET` - Send attitude/thrust setpoints

**Control Algorithm**:
```python
# Simple altitude hold with rate control
roll = 0.0
pitch = 0.0
yaw_rate = 0.0
thrust = 0.6  # Normalized (0.0 to 1.0)

# Send at 10 Hz
for 30 seconds:
    send_attitude_target(roll, pitch, yaw_rate, thrust)
    sleep(0.1)
```

---

## System Requirements

### Software
- **Ubuntu 22.04** (or compatible Linux)
- **Gazebo Harmonic 8.10.0** (gz sim)
- **Python 3** with pymavlink
- **PX4_minimal** custom build
- **bc** (bash calculator for floating point math)

### Python Dependencies
```bash
pip3 install pymavlink
```

### Build Requirements
```bash
cd /home/gaurav/PX4_minimal
make px4_sitl_default
```

---

## Usage Workflow

### Quick Start (3 Drones)

**Terminal 1 - Launch Simulation**:
```bash
cd /home/gaurav/PX4_minimal
./launch_multi_drone.sh 3
```

Wait for:
- Gazebo window to appear (~20 seconds)
- All 3 drones to spawn (~40 seconds total)
- Console messages showing "Starting Drone X"

**Terminal 2 - Run Controller**:
```bash
cd /home/gaurav/PX4_minimal
python3 multi_drone_controller.py 3
```

Expected output:
```
============================================================
 PX4 Multi-Drone Controller
 Number of drones: 3
============================================================

Connecting to 3 drones...

Drone 0: Connecting to udp:127.0.0.1:14540...
Drone 0: ✓ Connected! System ID: 1
Drone 1: Connecting to udp:127.0.0.1:14541...
Drone 1: ✓ Connected! System ID: 2
Drone 2: Connecting to udp:127.0.0.1:14542...
Drone 2: ✓ Connected! System ID: 3

✓ 3/3 drones connected successfully!

Arming all drones...
Setting all drones to OFFBOARD mode...
Starting coordinated flight

Drone 0: Target altitude = 2.0m
Drone 1: Target altitude = 3.0m
Drone 2: Target altitude = 4.0m
```

Observe in Gazebo:
- All 3 drones arm (propellers spinning)
- Drones lift off to different altitudes
- Hold position for 30 seconds

**Cleanup**:
```bash
# Kill all instances
pkill -9 px4
pkill -9 -f 'gz sim'
rm -rf /tmp/px4*
```

---

## Advanced Usage

### Custom Number of Drones
```bash
# Launch 5 drones
./launch_multi_drone.sh 5

# Control 5 drones
python3 multi_drone_controller.py 5
```

### Debugging Single Drone
```bash
# Launch just 1 drone for testing
./launch_multi_drone.sh 1

# Manual MAVLink connection
python3
>>> from pymavlink import mavutil
>>> master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
>>> master.wait_heartbeat()
>>> print(f"System ID: {master.target_system}")
```

### Check Logs (Official Script)
```bash
# View drone 2 log
cat /tmp/drone_2.log

# Monitor in real-time
tail -f /tmp/drone_2.log

# Check for errors
grep ERROR /tmp/drone_*.log
```

---

## PX4 Multi-Instance Mechanism

### How It Works

PX4 uses the `-i` flag to run multiple instances:
```bash
./build/px4_sitl_default/bin/px4 -i 0    # Instance 0
./build/px4_sitl_default/bin/px4 -i 1    # Instance 1
./build/px4_sitl_default/bin/px4 -i 2    # Instance 2
```

Each instance automatically:
1. **Sets unique System ID**: `MAV_SYS_ID = instance + 1`
2. **Calculates unique ports**:
   - MAVLink UDP: `14540 + instance`
   - MAVLink TCP: `4560 + instance`
   - Simulator: `14560 + instance`
3. **Creates separate workspace**: `/tmp/px4_instance_<N>`
4. **Spawns unique model**: `${PX4_SIM_MODEL}_<instance>`

### Environment Variables

| Variable | Purpose | Example |
|----------|---------|---------|
| `PX4_SIM_MODEL` | Vehicle model type | `gz_x500` |
| `PX4_SYS_AUTOSTART` | Airframe ID | `4001` (x500) |
| `PX4_GZ_MODEL_POSE` | Spawn position | `"0,2,0.5,0,0,0"` |
| `PX4_GZ_STANDALONE` | Connect to existing Gazebo | `1` |

### rcS Startup Script

The PX4 startup script (`ROMFS/px4fmu_common/init.d-posix/rcS`) handles multi-instance setup:

```bash
# From rcS line 140:
param set MAV_SYS_ID $((px4_instance+1))
param set UXRCE_DDS_KEY $((px4_instance+1))
```

---

## Port Mapping Reference

| Instance | System ID | MAVLink UDP | MAVLink TCP | Simulator |
|----------|-----------|-------------|-------------|-----------|
| 0        | 1         | 14540       | 4560        | 14560     |
| 1        | 2         | 14541       | 4561        | 14561     |
| 2        | 3         | 14542       | 4562        | 14562     |
| 3        | 4         | 14543       | 4563        | 14563     |
| 4        | 5         | 14544       | 4564        | 14564     |

Formula:
- System ID = `instance + 1`
- MAVLink UDP = `14540 + instance`
- MAVLink TCP = `4560 + instance`
- Simulator = `14560 + instance`

---

## Troubleshooting

### Issue: Only 1 drone spawns in Gazebo
**Cause**: Additional instances not using `PX4_GZ_STANDALONE=1`

**Solution**: Ensure launch script sets `PX4_GZ_STANDALONE=1` for instances 1+

### Issue: Drones spawn at same position
**Cause**: `PX4_GZ_MODEL_POSE` not set or incorrectly formatted

**Solution**: Set unique positions:
```bash
PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0"    # Drone 0
PX4_GZ_MODEL_POSE="0,2,0.5,0,0,0"    # Drone 1
PX4_GZ_MODEL_POSE="0,4,0.5,0,0,0"    # Drone 2
```

### Issue: Controller can't connect to drones
**Cause**: PX4 instances not fully started or wrong ports

**Solution**: 
1. Check PX4 processes: `ps aux | grep px4`
2. Verify ports: `netstat -an | grep 14540`
3. Check logs: `cat /tmp/drone_*.log | grep ERROR`

### Issue: Gazebo crashes with multiple drones
**Cause**: Insufficient system resources

**Solution**: 
1. Reduce number of drones
2. Close other applications
3. Check system resources: `htop`

### Issue: Drones don't arm
**Cause**: Pre-arm checks failing or not in OFFBOARD mode

**Solution**:
1. Check minimal_commander logs
2. Ensure OFFBOARD mode is set before arming
3. Verify MAVLink heartbeat: send commands at >2Hz

---

## Performance Notes

### System Requirements Per Drone
- **CPU**: ~15% per drone (quad-core system)
- **RAM**: ~200MB per PX4 instance
- **GPU**: Shared by all drones in Gazebo

### Recommended Limits
- **Desktop (8GB RAM)**: Up to 3-4 drones
- **Workstation (16GB+ RAM)**: Up to 8-10 drones
- **High-end (32GB+ RAM)**: 15+ drones

### Timing Considerations
- **Gazebo startup**: 15-20 seconds
- **Per-drone spawn**: 8 seconds
- **MAVLink connection**: 1-2 seconds per drone
- **Total for 3 drones**: ~45 seconds

---

## References

- **PX4 Official Docs**: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
- **Gazebo Sim**: https://gazebosim.org/
- **MAVLink Protocol**: https://mavlink.io/en/
- **PyMAVLink**: https://github.com/ArduPilot/pymavlink

---

## Future Enhancements

### Planned Features
- [ ] QGroundControl multi-vehicle support
- [ ] Collision avoidance between drones
- [ ] Formation flight patterns
- [ ] Waypoint-based missions
- [ ] ROS 2 integration for advanced control

### Potential Improvements
- GUI for launch configuration
- Auto-detection of system limits
- Health monitoring dashboard
- Replay/recording capabilities

---

**Last Updated**: December 23, 2025
**Author**: Gaurav
**Version**: 1.0
