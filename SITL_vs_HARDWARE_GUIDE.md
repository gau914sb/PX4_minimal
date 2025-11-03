# SITL vs Real Hardware - Required Changes for PX4_minimal

## 📋 Overview

Your minimal build is **already 95% ready for real hardware!** The main differences are in the build target and startup configuration.

---

## 🔧 Changes Required for Real Hardware

### **1. Build Target Changes**

#### Current (SITL):
```bash
make px4_sitl_default
```

#### For Real Hardware (example: Pixhawk 4):
```bash
make px4_fmu-v5_default      # Pixhawk 4
# OR
make px4_fmu-v6x_default     # Pixhawk 6X
# OR
make px4_fmu-v5x_default     # CUAV V5+
```

**Files affected**: NONE in your code! Just different build target.

---

### **2. Board Configuration File**

#### Current:
`boards/px4/sitl/default.px4board`

#### For Real Hardware:
`boards/px4/fmu-v5/default.px4board` (or your specific board)

**Changes needed**:
- Copy your minimal module configuration to the hardware board config
- Same modules enabled: minimal_commander, mc_rate_control, control_allocator, etc.

---

### **3. Startup Script Changes**

#### Current (SITL startup):
`ROMFS/px4fmu_common/init.d-posix/rcS`

#### For Real Hardware:
`ROMFS/px4fmu_common/init.d/rcS`

**Key differences**:

| Aspect | SITL | Real Hardware |
|--------|------|---------------|
| **Simulator** | SIH module loads | No simulator |
| **Sensors** | Simulated | Real IMU, gyro, mag, baro |
| **PWM output** | Simulated | Real PWM to ESCs |
| **Environment vars** | `PX4_SIM_MODEL`, `PX4_SIMULATOR` | Not needed |
| **Serial ports** | UDP MAVLink | Physical UART/USB |

---

### **4. Module Differences**

#### Modules to REMOVE for real hardware:
```cmake
# In boards/px4/fmu-v5/default.px4board
CONFIG_MODULES_SIMULATOR_SIH=n          # ❌ Remove SIH
CONFIG_MODULES_SIMULATOR_SENSOR_AIRSPEED_SIM=n  # ❌ Remove sim modules
```

#### Modules to ADD for real hardware:
```cmake
# Sensor drivers (required!)
CONFIG_DRIVERS_IMU=y                    # ✅ Real IMU
CONFIG_DRIVERS_BAROMETER=y              # ✅ Barometer
CONFIG_DRIVERS_MAGNETOMETER=y           # ✅ Magnetometer (optional)
CONFIG_DRIVERS_GPS=y                    # ✅ GPS (optional)
CONFIG_DRIVERS_PWM_OUT=y                # ✅ PWM output to ESCs
CONFIG_DRIVERS_RC_INPUT=y               # ✅ RC receiver input

# Already have these (keep them):
CONFIG_MODULES_MINIMAL_COMMANDER=y      # ✅ Your custom commander
CONFIG_MODULES_MC_RATE_CONTROL=y        # ✅ Rate controller
CONFIG_MODULES_CONTROL_ALLOCATOR=y      # ✅ Motor mixer
CONFIG_MODULES_EKF2=y                   # ✅ State estimator
CONFIG_MODULES_SENSORS=y                # ✅ Sensor processing
CONFIG_MODULES_MAVLINK=y                # ✅ Communication
```

---

### **5. Startup Script for Hardware**

Create: `ROMFS/px4fmu_common/init.d/rc.minimal_mc`

```bash
#!/bin/sh
#
# @name Minimal Multicopter Configuration
#

# Load airframe parameters
. ${R}etc/init.d/rc.mc_defaults

# Start essential drivers
gps start -d /dev/ttyS1          # GPS on UART1 (optional)
#rc_input start                   # RC receiver (optional for offboard-only)

# Start sensors module
sensors start

# Start EKF2 estimator
ekf2 start

# Start minimal commander
minimal_commander start

# Start rate controller (no attitude controller!)
mc_rate_control start

# Start control allocator
control_allocator start

# Start MAVLink
mavlink start -d /dev/ttyUSB0 -b 921600 -m onboard  # Companion computer
mavlink start -d /dev/ttyS2 -b 57600 -m normal      # Telemetry radio

# Start logger
logger start -b 200
```

---

### **6. Code Changes Required**

#### ✅ **NO CODE CHANGES needed in:**
- `minimal_commander.cpp` - Works as-is ✓
- `mc_rate_control` - Works as-is ✓
- `control_allocator` - Works as-is ✓
- MAVLink communication - Works as-is ✓

#### ⚠️ **Minor adjustments:**

**In minimal_commander.cpp**, remove SIH-specific code:
```cpp
// REMOVE these lines (around line 299):
if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD &&
    (offboard_mode.attitude || offboard_mode.body_rate)) {
    // ... SIH-specific logging
}
```

---

### **7. Safety Considerations for Real Hardware**

#### Add to minimal_commander.cpp:

```cpp
// Pre-arm checks (add to minimal_commander.cpp)
bool check_safety() {
    // Check battery voltage
    if (_battery_status.voltage_v < 10.5f) {  // 3S LiPo minimum
        PX4_ERR("Battery voltage too low: %.2fV", _battery_status.voltage_v);
        return false;
    }

    // Check IMU calibration
    if (!_sensor_gyro_valid || !_sensor_accel_valid) {
        PX4_ERR("IMU not calibrated or invalid");
        return false;
    }

    // Check RC or offboard connection
    if (!_offboard_control_mode_valid) {
        PX4_ERR("No offboard control input");
        return false;
    }

    return true;
}
```

---

### **8. Hardware Setup Steps**

#### Hardware needed:
1. **Flight controller**: Pixhawk 4, Pixhawk 6X, CUAV V5+, etc.
2. **Frame**: Quadcopter frame with motors and ESCs
3. **Power**: LiPo battery + power module
4. **Companion computer** (optional): Raspberry Pi, Jetson Nano
5. **Telemetry** (optional): Radio for QGroundControl

#### Wiring:
```
Flight Controller:
├── TELEM1 → Companion computer (UART, 921600 baud)
├── TELEM2 → Telemetry radio (57600 baud)
├── GPS1   → GPS module (optional)
├── MAIN1-4 → ESCs (PWM outputs)
├── POWER  → Power module (battery monitoring)
└── USB    → Computer (for uploading firmware)

Companion Computer:
├── USB/UART → Flight controller TELEM1
└── WiFi → Your laptop (SSH for test scripts)
```

---

### **9. Upload Process**

```bash
# 1. Build for your hardware
cd /Users/gauravsinghbhati/Documents/PX4_minimal
make px4_fmu-v5_default

# 2. Connect flight controller via USB

# 3. Upload firmware
make px4_fmu-v5_default upload

# 4. Connect via QGroundControl to configure parameters
```

---

### **10. Test Script Changes**

Your `test_rate_control.py` needs **ZERO changes**!

Just change the connection:
```python
# For companion computer connected to flight controller:
connection_string = 'udp:192.168.1.100:14540'  # Companion IP
# OR
connection_string = '/dev/ttyUSB0:921600'  # Direct serial
```

---

## 📊 **Comparison Summary**

| Component | SITL + SIH | Real Hardware | Code Changes? |
|-----------|------------|---------------|---------------|
| **Build target** | `px4_sitl_default` | `px4_fmu-v5_default` | ❌ None |
| **minimal_commander** | Works | Works | ❌ None |
| **mc_rate_control** | Works | Works | ❌ None |
| **control_allocator** | Works | Works | ❌ None |
| **Sensors** | Simulated (SIH) | Real (IMU drivers) | ✅ Enable drivers |
| **Motor output** | Simulated PWM | Real PWM | ❌ None |
| **MAVLink** | UDP | Serial/UDP | ❌ None (just config) |
| **Test scripts** | Python works | Python works | ❌ None |
| **Safety checks** | Minimal | Need pre-arm checks | ✅ Add safety |

---

## 🎯 **Bottom Line**

Your minimal build is **95% ready for real hardware**!

**Changes needed:**
1. ✅ Change build target: `px4_fmu-v5_default`
2. ✅ Enable sensor drivers in board config
3. ✅ Disable SIH module
4. ✅ Adjust startup script (remove SIH, add sensor drivers)
5. ✅ Add safety checks (battery, IMU, etc.)
6. ❌ **NO changes to your core code** (minimal_commander, controllers, etc.)

**Your architecture is solid for real hardware! 🚀**

The modular design you built (minimal_commander → rate_control → allocator) is exactly how real flight works.

---

## ⚠️ **Important Safety Notes**

1. **Start with props off!** Test rate commands first
2. **Use safety switch** on flight controller
3. **Test in MANUAL mode** before OFFBOARD
4. **Have a kill switch** (RC transmitter)
5. **Calibrate IMU and magnetometer** before flight
6. **Start with low thrust** (0.3-0.4) in tests
7. **Use tether** for initial flights

