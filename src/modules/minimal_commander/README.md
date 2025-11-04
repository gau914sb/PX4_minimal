# Minimal Commander - User Manual

## Overview

The **Minimal Commander** is a simplified PX4 commander module designed for rapid development and testing with external control systems (ROS2, MAVLink, etc.). It bypasses complex validation checks (GPS, magnetometer, attitude estimators) to enable quick prototyping and research.

### Key Features
- ✅ **4-State Machine:** INIT → DISARMED ↔ ARMED, EMERGENCY
- ✅ **Minimal Safety Checks:** Battery (≥20%, ≥10V), Power (5V ≥4.5V), Emergency Stop
- ✅ **Multiple Arming Methods:** MAVLink commands, RC sticks, offboard auto-arm
- ✅ **Mode Switching:** OFFBOARD, MANUAL, STABILIZED
- ✅ **Offboard Timeout:** 500ms safety disconnect
- ✅ **Console Commands:** Direct control via PX4 console
- ✅ **10 Hz Operation:** Low-overhead monitoring

---

## Architecture

### State Machine
```
INIT (startup)
  ↓
DISARMED (safe, motors off)
  ↔ (arm/disarm commands)
ARMED (motors active)
  ↓ (critical battery/emergency)
EMERGENCY (forced disarm)
  ↓
DISARMED
```

### Module Structure
```
minimal_commander/
├── minimal_commander.cpp/hpp     # Main state machine (10 Hz)
├── minimal_safety_checks.cpp/hpp # Pre-arm validation
├── Safety.hpp                    # Safety interfaces
├── worker_thread.hpp             # Threading helpers
├── failsafe/                     # Failsafe stub (minimal)
├── failure_detector/             # Failure detector stub (minimal)
└── *_params.c                    # PX4 parameters (6 files)
```

---

## Getting Started

### 1. Build and Deploy

Build PX4 with minimal commander:
```bash
cd /path/to/PX4_minimal
make px4_sitl_default
```

### 2. Start SITL Simulation

```bash
make px4_sitl gazebo-classic
```

### 3. Start Minimal Commander

In PX4 console:
```bash
minimal_commander start
```

Verify it's running:
```bash
minimal_commander status
```

---

## Usage

### Console Commands

#### Start/Stop Module
```bash
minimal_commander start    # Start the module
minimal_commander stop     # Stop the module
```

#### Arming/Disarming
```bash
minimal_commander arm      # Arm the vehicle
minimal_commander disarm   # Disarm the vehicle
```

#### Takeoff (Offboard Mode)
```bash
minimal_commander takeoff  # Enable offboard mode for external control
```

#### Status Check
```bash
minimal_commander status   # Detailed status information
```

**Example Status Output:**
```
=== Minimal Commander Status ===
State: ARMED
Navigation: OFFBOARD
Battery: 12.50V (100.0%)
Arm/Disarm cycles: 5
Emergency stops: 0
Armed for: 1234 ms
```

---

## Arming Methods

### Method 1: Console Command (Manual)
```bash
minimal_commander arm
```

### Method 2: MAVLink Command (Python Script)

Using `pymavlink`:
```python
from pymavlink import mavutil

# Connect to vehicle
vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14540')
vehicle.wait_heartbeat()

# Send ARM command
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    1,  # param1: 1=ARM, 0=DISARM
    0, 0, 0, 0, 0, 0
)

# Wait for ACK
msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Result: {msg.result}")
```

### Method 3: RC Sticks (Optional)

**Arming:** Throttle LOW + Yaw RIGHT (hold for 1 second)
```
Throttle < 10%  AND  Yaw > 80%
```

**Disarming:** Throttle LOW + Yaw LEFT (hold for 1 second)
```
Throttle < 10%  AND  Yaw < -80%
```

### Method 4: Offboard Auto-Arm

Automatically arms when offboard control messages are received:
```python
# Send offboard_control_mode message
# Vehicle auto-arms if safety checks pass
```

---

## Safety Checks

Minimal Commander performs **3 essential safety checks** before arming:

| Check | Requirement | Bypass Option |
|-------|-------------|---------------|
| **Battery Voltage** | ≥10V | ❌ Critical |
| **Battery Charge** | ≥20% | ❌ Critical |
| **Power Rail** | 5V rail ≥4.5V | ❌ Critical |
| **Emergency Stop** | Button not pressed | ⚠️ Always passes (stub) |

**Arming Blocked Examples:**
```
❌ Arming BLOCKED - Essential safety check failed
   → Battery: 9.5V (below 10V threshold)
   → Charge: 15% (below 20% threshold)
```

---

## Mode Switching

### Offboard Mode (External Control)

Enable offboard mode for MAVLink/ROS2 control:
```bash
minimal_commander takeoff
```

Or via MAVLink:
```python
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
    0, 0, 0, 0, 0
)
```

### Manual Mode

Switch to manual RC control:
```python
# MAV_CMD_DO_SET_MODE with custom_mode = 1 (MANUAL)
```

### Stabilized Mode

Self-leveling mode with RC input:
```python
# MAV_CMD_DO_SET_MODE with custom_mode = 7 (STABILIZED)
```

---

## Offboard Control

### Requirements
1. **Arming:** Vehicle must be armed
2. **Heartbeat:** Send offboard messages continuously
3. **Timeout:** Max gap between messages: **500ms**

### Offboard Timeout Behavior

If offboard messages stop for >500ms while armed:
```
⚠️ DISARMED - Offboard control timeout (>500ms)
```

**Prevention:** Maintain ≥2 Hz message rate (recommended: 10-50 Hz)

### Example: Attitude Control

```python
from pymavlink import mavutil
import time

vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14540')
vehicle.wait_heartbeat()

# Arm vehicle first
vehicle.mav.command_long_send(...)  # ARM command

# Enable offboard mode
vehicle.mav.command_long_send(...)  # SET_MODE to OFFBOARD

# Send attitude setpoint at 10 Hz
while True:
    vehicle.mav.set_attitude_target_send(
        0,  # time_boot_ms
        vehicle.target_system,
        vehicle.target_component,
        0b00000111,  # type_mask (ignore rates)
        [1, 0, 0, 0],  # quaternion [w, x, y, z]
        0, 0, 0,  # body roll/pitch/yaw rates
        0.5  # thrust (0-1)
    )

    # Send offboard_control_mode
    vehicle.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(...)
    )

    time.sleep(0.1)  # 10 Hz
```

---

## Battery Monitoring

### Auto-Disarm on Low Battery

If enabled (`COM_LOW_BAT_ACT` parameter), the commander will:

| Battery Level | Action |
|---------------|--------|
| **WARNING_LOW** | Auto-disarm (controlled) |
| **WARNING_CRITICAL** | Emergency stop (immediate) |

**Log Output:**
```
⚠️ LOW BATTERY - Auto disarm! Voltage: 10.5V
⚠️ CRITICAL BATTERY - Emergency disarm! Voltage: 9.8V
```

### Battery Status Logging

Battery status is logged every 5 seconds:
```
INFO: Battery: 12.50V (95.0%), Warning: 0
INFO: Battery: 11.80V (75.0%), Warning: 0
INFO: Battery: 10.50V (45.0%), Warning: 1  ← LOW
```

---

## Parameters

### Commander Parameters (`minimal_commander_params.c`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `COM_DISARM_LAND` | 2.0s | Auto-disarm delay after landing |
| `COM_RC_LOSS_T` | 0.5s | RC loss timeout (if enabled) |
| `COM_LOW_BAT_ACT` | 1 | Low battery action (0=warn, 1=land, 2=RTL) |

### Position Control Parameters (`mpc_*.c`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MPC_XY_P` | 0.95 | Position gain (X/Y) |
| `MPC_Z_P` | 1.0 | Position gain (Z) |
| `MPC_Z_VEL_MAX_UP` | 3.0 m/s | Max climb rate |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | Max descent rate |
| `MPC_TILTMAX_AIR` | 45° | Max tilt angle in air |
| `MPC_MAN_TILT_MAX` | 35° | Max tilt in manual mode |

Modify parameters via:
```bash
param set MPC_Z_VEL_MAX_UP 5.0
param save
```

---

## Testing Scripts

### 1. Simple Arming Test
```bash
python3 test_arm_simple.py
```
Tests basic ARM command via MAVLink.

### 2. Arm and Takeoff
```bash
python3 arm_and_takeoff.py
```
Arms vehicle and enables offboard mode.

### 3. Ground Station Simulator
```bash
python3 mimic_ground_station.py
```
Sends periodic offboard heartbeats to maintain connection.

### 4. Console Command Test
```bash
./test_console_commands.sh
```
Tests all console commands (arm, disarm, takeoff, status).

### 5. Console Demo
```bash
python3 test_console_demo.py
```
Automated console command demonstration.

---

## Troubleshooting

### Issue: Arming Rejected

**Symptom:**
```
❌ Arming BLOCKED - Essential safety check failed
```

**Solutions:**
1. Check battery voltage: `listener battery_status`
2. Check battery charge: Ensure ≥20%
3. Check power rail: `listener system_power`
4. Review logs: Look for specific failure reason

### Issue: Offboard Timeout

**Symptom:**
```
⚠️ DISARMED - Offboard control timeout (>500ms)
```

**Solutions:**
1. Increase message rate: Send offboard messages at ≥10 Hz
2. Check network latency: Verify MAVLink connection stability
3. Monitor messages: `listener offboard_control_mode`

### Issue: Commands Not Processed

**Symptom:** ARM command sent but no response

**Solutions:**
1. Verify commander is running: `minimal_commander status`
2. Check target system ID: Ensure MAVLink addresses correct system
3. Monitor command ACKs: `listener vehicle_command_ack`
4. Check process_commands() logs: Should see command reception

### Issue: Vehicle Not Responding to Setpoints

**Symptom:** Offboard mode active but vehicle doesn't move

**Solutions:**
1. Check control mode flags: `listener vehicle_control_mode`
2. Verify controller is running: `mc_rate_control status`, `mc_att_control status`
3. Monitor setpoints: `listener vehicle_attitude_setpoint`
4. Check actuator outputs: `listener actuator_outputs`

---

## Performance

### Resource Usage

| Metric | Value |
|--------|-------|
| **CPU Usage** | ~0.5% (10 Hz operation) |
| **Memory** | ~50 KB |
| **Message Rate** | 3 publications/cycle (30 Hz total) |
| **Latency** | <1ms per cycle |

### Comparison: Minimal vs Full Commander

| Feature | Minimal Commander | Full Commander |
|---------|-------------------|----------------|
| **States** | 4 | 31+ navigation states |
| **Safety Checks** | 3 essential | 20+ comprehensive |
| **Code Size** | ~1,000 lines | ~10,000+ lines |
| **Dependencies** | Minimal | Heavy (GPS, sensors) |
| **Arming Delay** | <10ms | 100-500ms |
| **Use Case** | Rapid prototyping | Production flights |

---

## Advanced Usage

### Custom Safety Checks

Modify `minimal_safety_checks.cpp` to add custom validation:

```cpp
bool MinimalSafetyChecks::checkCustom() {
    // Add your custom check here
    if (my_sensor_value < threshold) {
        return false;  // Block arming
    }
    return true;
}
```

Then call it in `checkAndUpdateArmingState()`.

### State Machine Extension

Add new states in `minimal_commander.hpp`:

```cpp
enum class MinimalCommanderState {
    INIT,
    DISARMED,
    ARMED,
    EMERGENCY,
    MY_CUSTOM_STATE  // ← New state
};
```

Update transitions in `MinimalStateMachine::can_transition()`.

### Custom Mode Implementation

Add mode in `process_commands()`:

```cpp
case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
    if (custom_mode == MY_CUSTOM_MODE) {
        _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
        // ... handle custom mode
    }
    break;
```

---

## Integration Examples

### ROS2 Integration

See PX4-ROS2 bridge documentation for offboard control via ROS2 topics.

### Python MAVLink Integration

Full example in `arm_and_takeoff.py` and `test_arm_simple.py`.

### C++ External Controller

Use uORB subscription to `vehicle_status` and publication to `vehicle_attitude_setpoint`.

---

## Safety Notes

⚠️ **This is a research/development module:**
- NOT for production flights
- Bypasses critical safety checks
- Use ONLY in simulation or controlled test environments
- Always have emergency stop ready
- Monitor battery levels manually

✅ **Safe for:**
- Algorithm development
- Controller testing
- Simulation experiments
- Academic research

❌ **NOT safe for:**
- Real outdoor flights
- Untested hardware
- Public demonstrations
- Production systems

---

## Support

### Logs and Debugging

Enable verbose logging:
```bash
minimal_commander start -d  # Debug mode
```

View real-time logs:
```bash
dmesg -f  # Follow mode
```

Check specific topics:
```bash
listener vehicle_status
listener actuator_armed
listener vehicle_control_mode
listener offboard_control_mode
```

### Common Log Messages

| Message | Meaning |
|---------|---------|
| `process_commands() alive` | Commander running normally (every 5s) |
| `ARMED via command` | Arming successful |
| `DISARMED via command` | Disarming successful |
| `Arming BLOCKED` | Safety check failed |
| `Offboard control timeout` | Lost offboard connection |
| `EMERGENCY STOP activated` | Critical failure, immediate stop |

---

## Contributing

When modifying minimal_commander:

1. **Test in SITL first:** Always validate in simulation
2. **Maintain minimal philosophy:** Keep it simple, avoid feature creep
3. **Document changes:** Update this README
4. **Test safety checks:** Ensure arming/disarming still works
5. **Check performance:** Maintain 10 Hz operation

---

## Version History

- **v1.0** (2025-03-01): Initial minimal commander implementation
  - 4-state machine
  - 3 essential safety checks
  - MAVLink, RC, and offboard arming
  - 10 Hz operation

---

## License

BSD 3-Clause License (same as PX4-Autopilot)

---

## References

- [PX4 Developer Guide](https://docs.px4.io/main/en/)
- [MAVLink Protocol](https://mavlink.io/en/)
- [PX4 uORB Messaging](https://docs.px4.io/main/en/middleware/uorb.html)
- [PX4 Modules](https://docs.px4.io/main/en/modules/modules_main.html)

---

**For questions or issues, check the troubleshooting section above or review the PX4 forums.**
