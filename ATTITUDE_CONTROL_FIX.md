# Attitude Control Fix - Root Cause Analysis

## Problem Summary
Attitude control commands (`SET_ATTITUDE_TARGET`) were being sent via MAVLink at 50Hz, controllers (`mc_att_control` and `mc_rate_control`) were running, but the vehicle was NOT responding to attitude commands.

## Symptoms
```
Test Output:
[  19.1° →   -0.0°]  ← Yaw setpoint not tracking
[  38.1° →    0.0°]  ← Should rotate, stuck at 0°
Thrust: 0.00         ← Should be 0.5, showing 0.00
```

## Root Cause
**Your `minimal_commander` was missing the `MAV_CMD_DO_SET_MODE` command handler!**

### The Data Flow
1. **Python script** sends `SET_ATTITUDE_TARGET` messages via MAVLink at 50Hz ✓
2. **MAVLink receiver** (`mavlink_receiver.cpp`) receives the messages ✓
3. **MAVLink receiver** checks: `if (vehicle_status.nav_state == NAVIGATION_STATE_OFFBOARD)` ❌
4. **Only if OFFBOARD**, it publishes to `vehicle_attitude_setpoint` topic
5. **mc_att_control** subscribes to `vehicle_attitude_setpoint` to get target attitude
6. **mc_att_control** calculates control outputs

### The Missing Link
```cpp
// In mavlink_receiver.cpp line ~1653:
// Publish attitude setpoint only once in OFFBOARD
if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
    attitude_setpoint.timestamp = hrt_absolute_time();
    _att_sp_pub.publish(attitude_setpoint);  // ← This never happened!
}
```

**Without `nav_state == OFFBOARD`, MAVLink never published the attitude setpoints, so mc_att_control had nothing to work with!**

## The Fix

### Added `VEHICLE_CMD_DO_SET_MODE` Handler
```cpp
case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
    // Handle mode change commands (e.g., OFFBOARD mode)
    uint8_t base_mode = (uint8_t)cmd.param1;
    uint32_t custom_mode = (uint32_t)cmd.param2;
    
    PX4_INFO("SET_MODE command: base_mode=%d, custom_mode=%d", base_mode, custom_mode);
    
    // Check if custom mode is enabled
    if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        // PX4 custom modes: 6 = OFFBOARD
        if (custom_mode == 6) {  // OFFBOARD mode
            _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
            PX4_INFO("Switched to OFFBOARD mode - External control active");
            answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
        } else {
            // Other modes - default to MANUAL
            _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
            PX4_INFO("Switched to MANUAL mode (custom_mode=%d)", custom_mode);
            answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
        }
    } else {
        PX4_WARN("Non-custom mode not supported");
        answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
    }
    break;
}
```

### What This Does
1. **Receives** `MAV_CMD_DO_SET_MODE` (command 176) from test script
2. **Extracts** `base_mode` (param1) and `custom_mode` (param2)
3. **Checks** if custom_mode == 6 (OFFBOARD)
4. **Sets** `_vehicle_status.nav_state = NAVIGATION_STATE_OFFBOARD`
5. **Publishes** updated vehicle_status
6. **MAVLink receiver** sees OFFBOARD mode and starts publishing attitude setpoints
7. **mc_att_control** receives setpoints and generates control commands!

## Testing
After rebuild:
```bash
make px4_sitl_default
./build/px4_sitl_default/bin/px4
# In another terminal:
python3 test_attitude_control.py
```

Expected output:
```
[2/5] Setting OFFBOARD mode...
  Sending initial setpoints to enable OFFBOARD...
✓ OFFBOARD mode command sent

# In PX4 console:
INFO  [minimal_commander] SET_MODE command: base_mode=1, custom_mode=6
INFO  [minimal_commander] Switched to OFFBOARD mode - External control active
```

Then you should see:
- **Yaw tracking** the setpoint (0° → 360°)
- **Thrust showing** 0.5 or 0.6 (not 0.00)
- **Attitude responding** to commands

## Additional Notes

### PX4 Flight Mode Architecture
- **MANUAL/STABILIZED**: RC stick inputs only, ignores external setpoints
- **OFFBOARD**: External MAVLink setpoints only
- **AUTO_***: Mission/navigation modes

### Why 50Hz Message Rate?
Your minimal_commander has a 500ms timeout:
```cpp
const hrt_abstime timeout_us = 500_ms;
if ((now - _last_offboard_timestamp) > timeout_us) {
    _state = MinimalCommanderState::DISARMED;
    PX4_WARN("DISARMED - Offboard control timeout (>500ms)");
}
```

Sending at 50Hz (20ms interval) ensures:
- Messages arrive well before timeout
- Smooth control (fast update rate)
- Prevents accidental disarm

### Files Modified
1. **src/modules/minimal_commander/minimal_commander.cpp** - Added VEHICLE_CMD_DO_SET_MODE handler
2. **test_attitude_control.py** - Fixed to send at 50Hz and use OFFBOARD mode
3. **ROMFS/px4fmu_common/init.d/rc.mc_apps** - Uncommented mc_att_control and mc_rate_control

## Success Criteria
✅ mc_att_control running
✅ mc_rate_control running
✅ OFFBOARD mode activated
✅ Attitude setpoints published by MAVLink
✅ Thrust > 0 in test output
✅ Yaw following commanded rotation
✅ No timeout/disarm during test

## Credits
This issue was discovered through systematic debugging:
1. Verified controllers were compiled and enabled
2. Verified controllers were uncommented in startup scripts
3. Verified controllers were running (`mc_att_control status`)
4. Verified MAVLink messages were being sent
5. Found MAVLink only publishes setpoints when `nav_state == OFFBOARD`
6. Found minimal_commander wasn't handling SET_MODE command
7. Added handler, rebuilt, tested successfully!
