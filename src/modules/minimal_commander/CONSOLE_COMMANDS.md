# Minimal Commander Console Commands

## Overview
The `minimal_commander` module now includes built-in console commands for direct vehicle control from the PX4 shell. This makes testing and manual control much easier without needing external scripts.

## Available Commands

### 1. Status Command
Shows detailed system status including:
- Current state (INIT, DISARMED, ARMED, EMERGENCY)
- Navigation mode (MANUAL, OFFBOARD, etc.)
- Battery voltage and percentage
- Arm/disarm cycle count
- Emergency stop count
- Time armed (if currently armed)

```bash
pxh> minimal_commander status
```

**Example Output:**
```
=== Minimal Commander Status ===
State: DISARMED
Navigation: MANUAL
Battery: 16.20V (100.0%)
Arm/Disarm cycles: 0
Emergency stops: 0
```

### 2. Arm Command
Arms the vehicle for flight. Performs safety checks before arming.

```bash
pxh> minimal_commander arm
```

**Success:**
```
✅ ARMED via console command
```

**Failure:**
```
❌ Arming BLOCKED - Essential safety check failed
```

### 3. Takeoff Command
Enables takeoff/offboard mode. Vehicle must be armed first.

```bash
pxh> minimal_commander takeoff
```

**Success:**
```
✅ TAKEOFF mode enabled - External controller active
   Send offboard setpoints (attitude/thrust) to control the vehicle
```

**Failure:**
```
❌ Takeoff rejected - Vehicle not armed
   Run: minimal_commander arm
```

### 4. Disarm Command
Disarms the vehicle, stopping all motors.

```bash
pxh> minimal_commander disarm
```

**Success:**
```
✅ DISARMED via console command
```

## Usage Examples

### Basic Flight Sequence

1. **Start PX4 SITL:**
   ```bash
   cd /path/to/PX4-Autopilot
   ./build/px4_sitl_default/bin/px4
   ```

2. **Check initial status:**
   ```
   pxh> minimal_commander status
   ```

3. **Arm the vehicle:**
   ```
   pxh> minimal_commander arm
   ```

4. **Enable takeoff mode:**
   ```
   pxh> minimal_commander takeoff
   ```

5. **At this point, send offboard control commands** (attitude/thrust setpoints) via MAVLink or another method

6. **Disarm when done:**
   ```
   pxh> minimal_commander disarm
   ```

### Testing Workflow

```bash
# Start PX4
./build/px4_sitl_default/bin/px4

# In PX4 console (pxh> prompt):
pxh> minimal_commander status      # Check current state
pxh> minimal_commander arm         # Arm vehicle
pxh> minimal_commander status      # Verify armed
pxh> minimal_commander takeoff     # Enable offboard mode
pxh> minimal_commander status      # Check navigation state
# ... send offboard commands here ...
pxh> minimal_commander disarm      # Disarm when done
```

## Integration with MAVLink

The console commands complement MAVLink control:

- **Console commands**: Direct control from PX4 shell (good for testing/debugging)
- **MAVLink commands**: Remote control via GCS or scripts (good for automation)

Both methods work simultaneously and use the same underlying state machine.

## Command Reference

| Command | Description | Requires Armed | Changes State |
|---------|-------------|----------------|---------------|
| `status` | Show system status | No | No |
| `arm` | Arm vehicle | No | DISARMED → ARMED |
| `takeoff` | Enable offboard mode | Yes | Sets nav_state=OFFBOARD |
| `disarm` | Disarm vehicle | No | ARMED → DISARMED |

## Help

To see all available commands and usage:
```bash
pxh> minimal_commander help
```

## Notes

- All commands provide immediate feedback (✅ success or ❌ error)
- Commands perform the same safety checks as MAVLink commands
- State changes are logged and can be monitored
- Battery status updates in real-time
- Compatible with both interactive and automated testing

## Troubleshooting

**"minimal_commander is not running"**
- The module hasn't started yet. It should auto-start with PX4.
- Check: `minimal_commander status` to see if it's running

**"Already armed"**
- Vehicle is already in the requested state
- Check current state with `minimal_commander status`

**"Arming BLOCKED"**
- Safety checks failed (usually battery-related)
- Check battery voltage: `minimal_commander status`
- Verify battery is above minimum threshold

**"Takeoff rejected - Vehicle not armed"**
- Must arm first: `minimal_commander arm`
- Then retry: `minimal_commander takeoff`
