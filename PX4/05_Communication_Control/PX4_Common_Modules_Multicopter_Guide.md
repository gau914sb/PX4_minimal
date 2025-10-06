# **PX4 Common Modules for Multicopter Vehicles**

*Comprehensive guide to core modules used across all multicopter vehicle types*

---

## **Table of Contents**

1. [Overview](#overview)
2. [Core System Modules](#core-system-modules)
3. [Sensor and Data Processing Modules](#sensor-and-data-processing-modules)
4. [Communication and Interface Modules](#communication-and-interface-modules)
5. [Navigation and Control Support Modules](#navigation-and-control-support-modules)
6. [Safety and Monitoring Modules](#safety-and-monitoring-modules)
7. [Utility and Management Modules](#utility-and-management-modules)

---

## **Overview**

This guide focuses on **common modules** that are essential for **multicopter** operations (quadcopter, hexacopter, octocopter, etc.). These modules provide fundamental functionality regardless of the specific multicopter configuration.

**Excluded from this guide:**
- Vehicle-specific modules (fw_*, rover_*, uuv_*, vtol_*, airship_*)
- Hardware-specific modules (px4iofirmware, muorb)
- Specialized application modules (zenoh, uxrce_dds_client)
---

## **Core System Modules**

### **Essential Foundation Modules**

| **Module** | **Purpose** | **Function** | **Used By** |
|------------|-------------|--------------|-------------|
| **commander** | System state management | Vehicle arming, mode switching, failsafe handling | All multicopters |
| **sensors** | Sensor data processing | IMU, GPS, barometer data fusion and publishing | All multicopters |
| **ekf2** | State estimation | Position, velocity, attitude estimation using Extended Kalman Filter | All multicopters |
| **navigator** | Mission and waypoint management | Autonomous flight planning, geofencing, RTL | All multicopters |
| **mavlink** | Communication protocol | Ground control station communication, telemetry | All multicopters |

---

## **Sensor and Data Processing Modules**

### **Data Collection and Processing**

| **Module** | **Purpose** | **Function** | **Used By** |
|------------|-------------|--------------|-------------|
| **battery_status** | Power monitoring | Battery voltage, current, capacity monitoring | All multicopters |
| **airspeed_selector** | Airspeed validation | Multiple airspeed sensor validation and selection | Multicopters with airspeed sensors |
| **temperature_compensation** | Sensor calibration | Temperature-based sensor bias compensation | All multicopters |
| **gyro_calibration** | Sensor calibration | Gyroscope bias and scale factor calibration | All multicopters |

---

## **Communication and Interface Modules**

### **External Communication**

| **Module** | **Purpose** | **Function** | **Used By** |
|------------|-------------|--------------|-------------|
| **manual_control** | Pilot input processing | RC transmitter input processing and failsafe | All multicopters |
| **rc_update** | RC signal processing | Radio control signal decoding and mapping | All multicopters |
| **camera_feedback** | Camera integration | Camera trigger feedback and logging | Multicopters with cameras |

---

## **Navigation and Control Support Modules**

### **Flight Assistance**

| **Module** | **Purpose** | **Function** | **Used By** |
|------------|-------------|--------------|-------------|
| **land_detector** | Landing detection | Automatic detection of landed state | All multicopters |
| **flight_mode_manager** | Mode management | Flight mode transitions and validation | All multicopters |
| **control_allocator** | Actuator management | Motor/servo output distribution and mixing | All multicopters |

---

## **Safety and Monitoring Modules**

### **System Health and Safety**

| **Module** | **Purpose** | **Function** | **Used By** |
|------------|-------------|--------------|-------------|
| **load_mon** | System monitoring | CPU load, memory usage, task monitoring | All multicopters |
| **events** | Event management | System events, alarms, LED status indication | All multicopters |

---

## **Utility and Management Modules**

### **Data and System Management**

| **Module** | **Purpose** | **Function** | **Used By** |
|------------|-------------|--------------|-------------|
| **logger** | Data logging | Flight data recording and storage | All multicopters |
| **dataman** | Data persistence | Mission, parameter, and configuration storage | All multicopters |
| **time_persistor** | Time management | RTC synchronization and time persistence | All multicopters |
| **replay** | Log analysis | Flight log replay for analysis and debugging | Development/analysis |

---

## **Module Dependencies and Interaction**

```mermaid
flowchart TB
    subgraph "Hardware Layer"
        SENSORS[Sensors Hardware]
        RC[RC Receiver]
        MOTORS[Motors/ESCs]
    end

    subgraph "Core Processing"
        SENS[sensors]
        EKF2[ekf2]
        CMD[commander]
    end

    subgraph "Input Processing"
        RC_UP[rc_update]
        MAN[manual_control]
    end

    subgraph "Flight Management"
        NAV[navigator]
        FMM[flight_mode_manager]
        CTRL_ALLOC[control_allocator]
    end

    subgraph "Communication"
        MAV[mavlink]
        LOG[logger]
    end

    subgraph "Safety & Monitoring"
        LAND[land_detector]
        EVENTS[events]
        LOAD[load_mon]
    end

    SENSORS --> SENS
    RC --> RC_UP
    RC_UP --> MAN
    SENS --> EKF2
    EKF2 --> CMD
    CMD --> FMM
    MAN --> CMD
    FMM --> NAV
    NAV --> CTRL_ALLOC
    CTRL_ALLOC --> MOTORS

    CMD --> MAV
    SENS --> LOG
    EKF2 --> LOG

    CMD --> LAND
    CMD --> EVENTS
    LOAD --> EVENTS

    style CMD fill:#ff9999
    style EKF2 fill:#99ccff
    style SENS fill:#99ff99
```

---

---

## **Core System Modules - Detailed**

### **1. Commander Module**

**Location**: `src/modules/commander/`
**Main Class**: `Commander`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
The **Commander** is the central state machine and decision-making module for the entire flight system. It acts as the "brain" that coordinates all other modules.

```cpp
class Commander : public ModuleBase<Commander>, public ModuleParams
{
    // Core state management
    vehicle_status_s _vehicle_status{};
    vehicle_control_mode_s _control_mode{};
    actuator_armed_s _armed{};

    // Safety and failsafe systems
    FailureDetector _failure_detector;
    HealthAndArmingChecks _health_and_arming_checks;
    Failsafe _failsafe;
};
```

#### **Key Responsibilities**
| **Function** | **Description** | **uORB Topics** |
|--------------|-----------------|-----------------|
| **Vehicle Arming** | Controls motor arming/disarming based on safety checks | `actuator_armed` (pub) |
| **Flight Mode Management** | Manages transitions between flight modes (Manual, Stabilized, Mission, etc.) | `vehicle_control_mode` (pub) |
| **Failsafe Handling** | Monitors system health and triggers appropriate failsafe responses | `vehicle_status` (pub) |
| **Safety Checks** | Pre-flight checks, sensor validation, battery monitoring | `vehicle_command` (sub) |
| **State Broadcasting** | Publishes vehicle state to all other modules | `vehicle_status` (pub) |

#### **Critical State Machine**
```cpp
// Main vehicle states
typedef enum {
    VEHICLE_STATE_PREFLIGHT = 0,
    VEHICLE_STATE_STANDBY,
    VEHICLE_STATE_ARMED,
    VEHICLE_STATE_TAKEOFF,
    VEHICLE_STATE_LOITER,
    VEHICLE_STATE_MISSION,
    VEHICLE_STATE_RTL,
    VEHICLE_STATE_LAND,
    VEHICLE_STATE_EMERGENCY
} vehicle_state_t;
```

#### **Failsafe System Integration**
```cpp
// Failsafe triggers monitored by Commander
- RC Loss: No RC signal for > RC_LOSS_TH seconds
- Data Link Loss: No MAVLink for > DL_LOSS_TH seconds
- GPS Loss: GPS accuracy below GPS_LOSS_TH
- Battery Low: Battery below BAT_LOW_TH voltage
- Geofence Violation: Vehicle outside defined boundaries
- Engine Failure: Motor/actuator failures detected
```

---

### **2. Sensors Module**

**Location**: `src/modules/sensors/`
**Main Class**: `Sensors`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
The **Sensors** module is responsible for collecting, processing, and publishing all sensor data from hardware sensors to standardized uORB topics.

```cpp
class Sensors : public ModuleParams, public px4::ScheduledWorkItem
{
    // Sensor processors
    VehicleAcceleration _vehicle_acceleration;
    VehicleAngularVelocity _vehicle_angular_velocity;
    VehicleAirData _vehicle_air_data;
    VehicleGpsPosition _vehicle_gps_position;
    VehicleMagnetometer _vehicle_magnetometer;
};
```

#### **Sensor Data Processing Pipeline**
```mermaid
flowchart LR
    subgraph "Raw Sensors"
        IMU[IMU Sensors]
        GPS[GPS Receiver]
        BARO[Barometer]
        MAG[Magnetometer]
    end

    subgraph "Sensors Module"
        CALIB[Calibration]
        FILTER[Filtering]
        VALID[Validation]
        FUSION[Multi-sensor Fusion]
    end

    subgraph "Published Topics"
        ACCEL[vehicle_acceleration]
        GYRO[vehicle_angular_velocity]
        GPS_POS[vehicle_gps_position]
        AIR[vehicle_air_data]
        MAG_FIELD[vehicle_magnetometer]
    end

    IMU --> CALIB
    GPS --> VALID
    BARO --> FILTER
    MAG --> CALIB

    CALIB --> FUSION
    FILTER --> FUSION
    VALID --> FUSION

    FUSION --> ACCEL
    FUSION --> GYRO
    FUSION --> GPS_POS
    FUSION --> AIR
    FUSION --> MAG_FIELD
```

#### **Key Processing Functions**
| **Sensor Type** | **Processing** | **Output Topic** | **Frequency** |
|-----------------|----------------|------------------|---------------|
| **IMU (Accel/Gyro)** | Temperature compensation, bias removal, filtering | `sensor_combined` | 1000 Hz |
| **GPS** | Position validation, dilution of precision checks | `vehicle_gps_position` | 10 Hz |
| **Barometer** | Altitude calculation, weather compensation | `vehicle_air_data` | 100 Hz |
| **Magnetometer** | Hard/soft iron correction, declination | `vehicle_magnetometer` | 100 Hz |

#### **Multi-Sensor Selection**
```cpp
// Example: Multiple IMU handling
void Sensors::Run()
{
    // Poll all available IMU instances
    for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
        if (_sensor_accel_sub[i].updated()) {
            sensor_accel_s accel_report;
            _sensor_accel_sub[i].copy(&accel_report);

            // Apply calibration and temperature compensation
            _vehicle_acceleration.ProcessSensorAccel(accel_report);
        }
    }

    // Publish fused acceleration data
    _vehicle_acceleration.Publish();
}
```

---

### **3. EKF2 Module**

**Location**: `src/modules/ekf2/`
**Main Class**: `EKF2`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
**EKF2** (Extended Kalman Filter 2) is PX4's primary state estimator that fuses multiple sensor inputs to provide accurate estimates of vehicle position, velocity, and attitude.

```cpp
class EKF2 : public ModuleBase<EKF2>, public ModuleParams, public px4::ScheduledWorkItem
{
    Ekf _ekf{};                    // Core EKF algorithm
    EKF2Selector _selector{};      // Multi-EKF selection logic

    // State publications
    uORB::Publication<vehicle_attitude_s> _attitude_pub;
    uORB::Publication<vehicle_local_position_s> _local_position_pub;
    uORB::Publication<vehicle_global_position_s> _global_position_pub;
    uORB::Publication<vehicle_odometry_s> _odometry_pub;
};
```

#### **EKF State Vector**
```cpp
// 24-state Extended Kalman Filter
typedef struct {
    Vector3f pos;        // Position (North, East, Down) [m]
    Vector3f vel;        // Velocity (North, East, Down) [m/s]
    Quatf quat;          // Attitude quaternion [-]
    Vector3f gyro_bias;  // Gyroscope bias [rad/s]
    Vector3f accel_bias; // Accelerometer bias [m/s²]
    Vector3f mag_I;      // Earth magnetic field [gauss]
    Vector3f mag_B;      // Body magnetic field [gauss]
    Vector2f wind_vel;   // Wind velocity (North, East) [m/s]
} ekf_state_t;
```

#### **Sensor Fusion Process**
```cpp
// EKF prediction and update cycle
void Ekf::update()
{
    // 1. PREDICTION STEP (using IMU)
    predictCovariance();    // Predict state covariance
    predictState();         // Predict state using IMU

    // 2. UPDATE STEP (using external sensors)
    if (gps_data_ready) {
        fuseGpsVelPos();    // Fuse GPS velocity and position
    }

    if (mag_data_ready) {
        fuseMag();          // Fuse magnetometer for heading
    }

    if (baro_data_ready) {
        fuseBaroHgt();      // Fuse barometer for altitude
    }

    // 3. PUBLISH RESULTS
    publishAttitude();      // Publish attitude estimate
    publishPosition();      // Publish position estimate
}
```

#### **Multi-EKF Architecture**
```mermaid
flowchart TB
    subgraph "Sensor Inputs"
        IMU[IMU Data]
        GPS[GPS Data]
        BARO[Barometer]
        MAG[Magnetometer]
    end

    subgraph "Multiple EKF Instances"
        EKF1[EKF Instance 1]
        EKF2[EKF Instance 2]
        EKF3[EKF Instance 3]
    end

    subgraph "EKF2 Selector"
        HEALTH[Health Checks]
        SELECT[Best EKF Selection]
        SWITCH[Switching Logic]
    end

    subgraph "Output"
        ATT[vehicle_attitude]
        POS[vehicle_local_position]
        GLOBAL[vehicle_global_position]
    end

    IMU --> EKF1
    GPS --> EKF1
    BARO --> EKF1
    MAG --> EKF1

    IMU --> EKF2
    GPS --> EKF2
    BARO --> EKF2
    MAG --> EKF2

    IMU --> EKF3
    GPS --> EKF3
    BARO --> EKF3
    MAG --> EKF3

    EKF1 --> HEALTH
    EKF2 --> HEALTH
    EKF3 --> HEALTH

    HEALTH --> SELECT
    SELECT --> SWITCH

    SWITCH --> ATT
    SWITCH --> POS
    SWITCH --> GLOBAL
```

#### **EKF Health Monitoring**
```cpp
// EKF health metrics
typedef struct {
    float innovation_variance;  // Innovation test ratio
    float position_accuracy;    // Position estimate accuracy
    float velocity_accuracy;    // Velocity estimate accuracy
    bool gps_check_pass;       // GPS consistency check
    bool mag_check_pass;       // Magnetometer consistency check
    bool height_check_pass;    // Height sensor consistency check
} ekf_health_t;
```

---

### **4. Navigator Module**

**Location**: `src/modules/navigator/`
**Main Class**: `Navigator`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
The **Navigator** module handles autonomous flight missions, waypoint navigation, geofencing, and return-to-launch (RTL) operations.

```cpp
class Navigator : public ModuleBase<Navigator>, public ModuleParams
{
    // Navigation modes
    NavigatorMode *_navigation_mode{nullptr};
    MissionBlock *_mission_block{nullptr};

    // Navigation states
    position_setpoint_triplet_s _pos_sp_triplet{};
    mission_s _mission{};
    home_position_s _home_pos{};
};
```

#### **Navigation Mode State Machine**
```mermaid
stateDiagram-v2
    [*] --> MANUAL
    MANUAL --> MISSION : Auto mode
    MISSION --> MISSION_ITEM : Execute waypoint
    MISSION_ITEM --> LOITER : Hold position
    LOITER --> RTL : Return home
    RTL --> LAND : Final approach
    LAND --> [*] : Landing complete

    MISSION --> RTL : Failsafe trigger
    LOITER --> RTL : Low battery
    MISSION_ITEM --> RTL : Geofence violation
```

#### **Key Navigation Functions**
| **Function** | **Description** | **Trigger** | **Output** |
|--------------|-----------------|-------------|------------|
| **Mission Execution** | Follow pre-planned waypoints | Auto mission mode | `position_setpoint_triplet` |
| **Return-to-Launch** | Automated return to home | Failsafe, low battery, manual | RTL path planning |
| **Loiter** | Hold position at current location | Loiter mode, waypoint hold | Position hold setpoint |
| **Geofencing** | Enforce flight boundaries | Position monitoring | Geofence violations |
| **Takeoff/Landing** | Automated takeoff and landing sequences | Mode commands | Vertical motion profiles |

#### **Mission Waypoint Processing**
```cpp
// Mission item execution
void Navigator::run_mission()
{
    if (_mission.current_seq >= 0 && _mission.current_seq < _mission.count) {
        mission_item_s mission_item = _mission.items[_mission.current_seq];

        switch (mission_item.nav_cmd) {
            case NAV_CMD_WAYPOINT:
                set_position_setpoint(mission_item.lat, mission_item.lon, mission_item.altitude);
                break;

            case NAV_CMD_LOITER_UNLIMITED:
                set_loiter_setpoint(mission_item.lat, mission_item.lon, mission_item.altitude);
                break;

            case NAV_CMD_RETURN_TO_LAUNCH:
                start_rtl_mission();
                break;

            case NAV_CMD_LAND:
                start_land_mission();
                break;
        }
    }
}
```

#### **Geofence Implementation**
```cpp
// Geofence monitoring
typedef struct {
    float max_horizontal_distance;  // Maximum distance from home [m]
    float max_altitude;            // Maximum altitude AGL [m]
    bool action_on_breach;         // Action: RTL, Land, Hold
    geofence_polygon_t polygons[]; // Custom polygon boundaries
} geofence_t;

bool Navigator::check_geofence_violation()
{
    Vector2f current_pos = get_current_position();
    Vector2f home_pos = get_home_position();

    // Check horizontal distance
    float distance_from_home = (current_pos - home_pos).length();
    if (distance_from_home > _geofence.max_horizontal_distance) {
        trigger_geofence_action();
        return true;
    }

    // Check altitude limit
    if (get_current_altitude() > _geofence.max_altitude) {
        trigger_geofence_action();
        return true;
    }

    return false;
}
```

---

### **5. MAVLink Module**

**Location**: `src/modules/mavlink/`
**Main Class**: `Mavlink`
**Work Queue**: `mavlink`

#### **Purpose and Functionality**
The **MAVLink** module implements the MAVLink protocol for communication with ground control stations (GCS), companion computers, and other MAVLink-enabled devices.

```cpp
class Mavlink : public ModuleParams
{
    // Communication streams
    MavlinkStream *_streams{nullptr};
    MavlinkCommandsHandler _command_handler;

    // Protocol configuration
    mavlink_channel_t _channel{MAVLINK_COMM_0};
    Protocol _protocol{Protocol::SERIAL};
    int _socket_fd{-1};
};
```

#### **MAVLink Communication Architecture**
```mermaid
flowchart TB
    subgraph "PX4 Internal (uORB)"
        TOPICS[uORB Topics]
        CMD[vehicle_command]
        STATUS[vehicle_status]
        POS[vehicle_global_position]
    end

    subgraph "MAVLink Module"
        RECV[Message Receiver]
        SEND[Message Sender]
        STREAMS[Data Streams]
        CMDS[Command Handler]
    end

    subgraph "External (MAVLink Protocol)"
        GCS[Ground Control Station]
        COMP[Companion Computer]
        RADIO[Radio Telemetry]
    end

    TOPICS --> SEND
    SEND --> STREAMS
    STREAMS --> GCS
    STREAMS --> COMP
    STREAMS --> RADIO

    GCS --> RECV
    COMP --> RECV
    RADIO --> RECV
    RECV --> CMDS
    CMDS --> CMD

    STATUS --> SEND
    POS --> SEND
```

#### **Key MAVLink Message Handling**
| **Message Type** | **Direction** | **Purpose** | **uORB Topic** |
|------------------|---------------|-------------|----------------|
| **HEARTBEAT** | Outbound | System status and mode | `vehicle_status` |
| **GLOBAL_POSITION_INT** | Outbound | GPS position and altitude | `vehicle_global_position` |
| **ATTITUDE** | Outbound | Vehicle orientation | `vehicle_attitude` |
| **COMMAND_LONG** | Inbound | Commands from GCS | `vehicle_command` |
| **SET_POSITION_TARGET** | Inbound | Position setpoints | `position_setpoint_triplet` |
| **MISSION_ITEM** | Bidirectional | Mission waypoints | `mission` |

#### **MAVLink Streaming System**
```cpp
// High-rate telemetry stream configuration
class StreamVehicleAttitude : public MavlinkStream
{
    const char *get_name() const override { return "ATTITUDE"; }
    uint16_t get_id() override { return MAVLINK_MSG_ID_ATTITUDE; }

    bool send() override {
        vehicle_attitude_s att;
        if (_att_sub.copy(&att)) {
            mavlink_attitude_t msg{};
            msg.time_boot_ms = att.timestamp / 1000;
            msg.roll = att.roll;
            msg.pitch = att.pitch;
            msg.yaw = att.yaw;
            msg.rollspeed = att.rollspeed;
            msg.pitchspeed = att.pitchspeed;
            msg.yawspeed = att.yawspeed;

            mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }
        return false;
    }
};
```

#### **Command Processing Pipeline**
```cpp
// Example: ARM/DISARM command handling
void MavlinkCommandsHandler::handle_command_component_arm_disarm(
    const mavlink_command_long_t &cmd)
{
    vehicle_command_s vcmd{};
    vcmd.timestamp = hrt_absolute_time();
    vcmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    vcmd.param1 = cmd.param1;  // 1.0 = arm, 0.0 = disarm
    vcmd.source_system = cmd.source_system;
    vcmd.source_component = cmd.source_component;
    vcmd.target_system = cmd.target_system;
    vcmd.target_component = cmd.target_component;

    // Publish to uORB for Commander to process
    _vehicle_command_pub.publish(vcmd);

    // Send acknowledgment back to GCS
    vehicle_command_ack_s ack{};
    ack.command = vcmd.command;
    ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
    _command_ack_pub.publish(ack);
}
```

---

## **Core Module Integration Summary**

These five core modules work together to provide the essential foundation for multicopter operations:

1. **Commander** ← Central decision maker, coordinates all modules
2. **Sensors** ← Provides sensor data to EKF2 and Commander
3. **EKF2** ← Provides state estimates to Navigator and Commander
4. **Navigator** ← Provides navigation commands to flight controllers
5. **MAVLink** ← Enables external communication and control

---

## **Sensor and Data Processing Modules - Detailed**

### **1. Battery Status Module**

**Location**: `src/modules/battery_status/`
**Main Class**: `BatteryStatus`
**Work Queue**: `wq_lp_default`

#### **Purpose and Functionality**
The **Battery Status** module monitors battery health, estimates remaining capacity, and provides critical power management information for safe flight operations.

```cpp
class BatteryStatus : public ModuleBase<BatteryStatus>, public ModuleParams
{
    // Battery monitoring
    AnalogBattery _analog_batteries[MAX_INSTANCES];

    // Publications
    uORB::Publication<battery_status_s> _battery_status_pub;

    // Battery parameters
    float _bat_v_charged{4.2f};     // Fully charged voltage
    float _bat_v_empty{3.4f};       // Empty voltage
    float _bat_capacity{5000.0f};   // Battery capacity [mAh]
};
```

#### **Battery Monitoring Pipeline**
```mermaid
flowchart LR
    subgraph "Hardware"
        ADC[ADC Voltage Reading]
        CURR[Current Sensor]
        TEMP[Temperature Sensor]
    end

    subgraph "Battery Status Module"
        FILTER[Voltage Filtering]
        CALIB[Calibration]
        SOC[State of Charge]
        HEALTH[Health Monitor]
    end

    subgraph "Output"
        STATUS[battery_status]
        WARNING[Low Battery Events]
        FAILSAFE[Critical Battery Failsafe]
    end

    ADC --> FILTER
    CURR --> CALIB
    TEMP --> HEALTH

    FILTER --> SOC
    CALIB --> SOC
    HEALTH --> SOC

    SOC --> STATUS
    SOC --> WARNING
    SOC --> FAILSAFE
```

#### **State of Charge Estimation**
```cpp
// Battery state estimation algorithms
void AnalogBattery::updateBatteryStatus()
{
    // 1. VOLTAGE-BASED SOC (Primary method)
    float voltage_filtered = _voltage_filter.apply(_raw_voltage);
    float soc_voltage = calculateVoltageBasedSOC(voltage_filtered);

    // 2. COULOMB COUNTING (Secondary method)
    float current_filtered = _current_filter.apply(_raw_current);
    _discharged_mah += (current_filtered * dt) / 3600.0f;  // mAh
    float soc_coulomb = 1.0f - (_discharged_mah / _bat_capacity);

    // 3. HYBRID ESTIMATION (Weighted combination)
    float weight_voltage = 0.7f;
    float weight_coulomb = 0.3f;
    _state_of_charge = (weight_voltage * soc_voltage) + (weight_coulomb * soc_coulomb);

    // 4. REMAINING TIME ESTIMATION
    if (current_filtered > 0.1f) {  // Discharging
        _remaining_time = (_state_of_charge * _bat_capacity) / current_filtered;  // hours
    }
}

float AnalogBattery::calculateVoltageBasedSOC(float voltage)
{
    // LiPo discharge curve approximation
    if (voltage >= _bat_v_charged) {
        return 1.0f;  // 100%
    } else if (voltage <= _bat_v_empty) {
        return 0.0f;  // 0%
    } else {
        // Non-linear discharge curve
        float voltage_range = _bat_v_charged - _bat_v_empty;
        float voltage_normalized = (voltage - _bat_v_empty) / voltage_range;

        // Polynomial approximation for LiPo curve
        return 0.2f * voltage_normalized + 0.8f * powf(voltage_normalized, 2.5f);
    }
}
```

#### **Battery Health Monitoring**
```cpp
// Battery health indicators
typedef struct {
    float internal_resistance;      // Estimated internal resistance [Ω]
    float temperature;             // Battery temperature [°C]
    uint32_t cycle_count;          // Charge/discharge cycles
    float capacity_degradation;    // Capacity loss percentage
    bool overheating;              // Temperature warning
    bool undervoltage;             // Low voltage protection
    bool overcurrent;              // High current warning
} battery_health_t;

void AnalogBattery::assessBatteryHealth()
{
    // Internal resistance estimation (voltage drop under load)
    if (_current_filtered > 1.0f) {
        float voltage_drop = _voltage_no_load - _voltage_filtered;
        _internal_resistance = voltage_drop / _current_filtered;

        // Health degradation indicator
        if (_internal_resistance > _resistance_threshold) {
            _health.capacity_degradation += 0.01f;  // 1% degradation
        }
    }

    // Temperature monitoring
    if (_temperature > BAT_TEMP_MAX) {
        _health.overheating = true;
        publish_warning(events::ID("battery_overheating"));
    }

    // Undervoltage protection
    if (_voltage_filtered < _bat_v_empty) {
        _health.undervoltage = true;
        trigger_emergency_landing();
    }
}
```

#### **Multi-Battery Support**
```cpp
// Multiple battery configuration
void BatteryStatus::Run()
{
    for (int i = 0; i < MAX_INSTANCES; i++) {
        if (_analog_batteries[i].is_valid()) {
            _analog_batteries[i].updateBatteryStatus();

            battery_status_s battery_status{};
            battery_status.timestamp = hrt_absolute_time();
            battery_status.voltage_v = _analog_batteries[i].voltage();
            battery_status.current_a = _analog_batteries[i].current();
            battery_status.remaining = _analog_batteries[i].state_of_charge();
            battery_status.discharged_mah = _analog_batteries[i].discharged_mah();
            battery_status.temperature = _analog_batteries[i].temperature();
            battery_status.cell_count = _analog_batteries[i].cell_count();

            _battery_status_pub.publish(battery_status);
        }
    }
}
```

---

### **2. Airspeed Selector Module**

**Location**: `src/modules/airspeed_selector/`
**Main Class**: `AirspeedSelector`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
The **Airspeed Selector** validates and selects the best airspeed measurement from multiple airspeed sensors, crucial for fixed-wing flight and advanced multicopter operations.

```cpp
class AirspeedSelector : public ModuleBase<AirspeedSelector>, public ModuleParams
{
    // Multiple airspeed validators
    AirspeedValidator _airspeed_validator[AIRSPEED_MAX_SENSORS];

    // Selected airspeed output
    uORB::Publication<airspeed_validated_s> _airspeed_validated_pub;

    // Validation parameters
    int _selected_airspeed_index{0};
    bool _airspeed_valid{false};
};
```

#### **Airspeed Validation Process**
```mermaid
flowchart TB
    subgraph "Airspeed Sensors"
        PITOT1[Pitot Tube 1]
        PITOT2[Pitot Tube 2]
        SYN[Synthetic Airspeed]
    end

    subgraph "Validation Tests"
        RANGE[Range Check]
        INNO[Innovation Check]
        CONSIS[Consistency Check]
        HEALTH[Health Monitor]
    end

    subgraph "Selection Logic"
        VOTE[Voting Algorithm]
        SWITCH[Switching Logic]
        FALLBACK[Fallback Strategy]
    end

    subgraph "Output"
        VALID[airspeed_validated]
        WIND[wind_estimate]
    end

    PITOT1 --> RANGE
    PITOT2 --> RANGE
    SYN --> RANGE

    RANGE --> INNO
    INNO --> CONSIS
    CONSIS --> HEALTH

    HEALTH --> VOTE
    VOTE --> SWITCH
    SWITCH --> FALLBACK

    FALLBACK --> VALID
    FALLBACK --> WIND
```

#### **Airspeed Validation Algorithm**
```cpp
class AirspeedValidator
{
public:
    void update_airspeed_validator(const airspeed_s &airspeed,
                                   const vehicle_local_position_s &local_pos,
                                   const vehicle_land_detected_s &land_detected)
    {
        // 1. RANGE VALIDATION
        bool range_valid = (airspeed.indicated_airspeed_m_s >= AIRSPEED_MIN) &&
                          (airspeed.indicated_airspeed_m_s <= AIRSPEED_MAX);

        // 2. INNOVATION TEST (Compare with ground speed)
        Vector2f ground_velocity(local_pos.vx, local_pos.vy);
        float ground_speed = ground_velocity.norm();
        float airspeed_innovation = fabsf(airspeed.indicated_airspeed_m_s - ground_speed);
        bool innovation_valid = airspeed_innovation < _innovation_threshold;

        // 3. CONSISTENCY CHECK (Multiple sensors)
        bool consistency_valid = true;
        for (int i = 0; i < _num_sensors; i++) {
            if (i != _sensor_index) {
                float difference = fabsf(airspeed.indicated_airspeed_m_s -
                                       _other_airspeed[i].indicated_airspeed_m_s);
                if (difference > _consistency_threshold) {
                    consistency_valid = false;
                    break;
                }
            }
        }

        // 4. HEALTH ASSESSMENT
        _checks_passed = range_valid && innovation_valid && consistency_valid;

        // 5. UPDATE HEALTH COUNTER
        if (_checks_passed) {
            _health_counter = math::min(_health_counter + 1, HEALTH_COUNTER_MAX);
        } else {
            _health_counter = math::max(_health_counter - 3, 0);  // Faster degradation
        }

        _sensor_valid = (_health_counter > HEALTH_THRESHOLD);
    }

private:
    static constexpr float AIRSPEED_MIN = 0.0f;    // m/s
    static constexpr float AIRSPEED_MAX = 100.0f;  // m/s
    static constexpr int HEALTH_COUNTER_MAX = 100;
    static constexpr int HEALTH_THRESHOLD = 50;
};
```

#### **Sensor Selection Strategy**
```cpp
void AirspeedSelector::select_airspeed()
{
    int valid_sensor_count = 0;
    int best_sensor_index = -1;
    float best_health_score = 0.0f;

    // 1. COUNT VALID SENSORS
    for (int i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_airspeed_validator[i].get_airspeed_valid()) {
            valid_sensor_count++;

            float health_score = _airspeed_validator[i].get_health_score();
            if (health_score > best_health_score) {
                best_health_score = health_score;
                best_sensor_index = i;
            }
        }
    }

    // 2. SELECTION LOGIC
    if (valid_sensor_count == 0) {
        // NO VALID SENSORS - Use synthetic airspeed
        _selected_airspeed_index = SYNTHETIC_AIRSPEED_INDEX;
        _airspeed_valid = false;
        use_synthetic_airspeed();

    } else if (valid_sensor_count == 1) {
        // SINGLE VALID SENSOR - Use it
        _selected_airspeed_index = best_sensor_index;
        _airspeed_valid = true;

    } else {
        // MULTIPLE VALID SENSORS - Use voting algorithm
        _selected_airspeed_index = run_airspeed_voting();
        _airspeed_valid = true;
    }
}

void AirspeedSelector::use_synthetic_airspeed()
{
    // Synthetic airspeed from ground speed and wind estimate
    vehicle_local_position_s local_pos;
    wind_s wind_estimate;

    if (_local_pos_sub.copy(&local_pos) && _wind_sub.copy(&wind_estimate)) {
        Vector2f ground_velocity(local_pos.vx, local_pos.vy);
        Vector2f wind_velocity(wind_estimate.windspeed_north, wind_estimate.windspeed_east);
        Vector2f air_velocity = ground_velocity - wind_velocity;

        _synthetic_airspeed = air_velocity.norm();
    }
}
```

#### **Wind Estimation Integration**
```cpp
// Wind estimation using multiple airspeed sources
void AirspeedSelector::estimate_wind()
{
    if (_airspeed_valid && _local_pos_valid) {
        Vector2f ground_velocity(_local_pos.vx, _local_pos.vy);
        float true_airspeed = _selected_airspeed * _air_density_ratio;

        // Wind triangle calculation
        float heading = _vehicle_attitude.yaw;
        Vector2f airspeed_vector(true_airspeed * cosf(heading),
                                true_airspeed * sinf(heading));
        Vector2f wind_velocity = ground_velocity - airspeed_vector;

        // Publish wind estimate
        wind_s wind{};
        wind.timestamp = hrt_absolute_time();
        wind.windspeed_north = wind_velocity(0);
        wind.windspeed_east = wind_velocity(1);
        wind.variance_north = _wind_variance;
        wind.variance_east = _wind_variance;

        _wind_pub.publish(wind);
    }
}
```

---

### **3. Temperature Compensation Module**

**Location**: `src/modules/temperature_compensation/`
**Main Class**: `TemperatureCompensation`
**Work Queue**: `wq_lp_default`

#### **Purpose and Functionality**
The **Temperature Compensation** module corrects sensor measurements for temperature-induced bias and scale factor variations, critical for maintaining accuracy across different operating conditions.

```cpp
namespace temperature_compensation
{
class TemperatureCompensation : public ModuleBase<TemperatureCompensation>, public ModuleParams
{
    // Compensation parameters for each sensor
    struct SensorCalibration {
        float ref_temp;           // Reference temperature [°C]
        float bias_coeffs[3][4];  // Polynomial coefficients for bias
        float scale_coeffs[3][4]; // Polynomial coefficients for scale
        bool valid;               // Calibration validity
    };

    SensorCalibration _accel_cal[MAX_ACCEL_COUNT];
    SensorCalibration _gyro_cal[MAX_GYRO_COUNT];
    SensorCalibration _baro_cal[MAX_BARO_COUNT];
};
}
```

#### **Temperature Compensation Process**
```mermaid
flowchart LR
    subgraph "Raw Sensor Data"
        ACCEL[Accelerometer]
        GYRO[Gyroscope]
        BARO[Barometer]
        TEMP[Temperature]
    end

    subgraph "Compensation Models"
        POLY[Polynomial Models]
        BIAS[Bias Correction]
        SCALE[Scale Correction]
        LOOKUP[Lookup Tables]
    end

    subgraph "Compensated Output"
        ACCEL_C[Corrected Accelerometer]
        GYRO_C[Corrected Gyroscope]
        BARO_C[Corrected Barometer]
    end

    ACCEL --> POLY
    GYRO --> POLY
    BARO --> POLY
    TEMP --> BIAS
    TEMP --> SCALE

    POLY --> BIAS
    BIAS --> SCALE
    SCALE --> LOOKUP

    LOOKUP --> ACCEL_C
    LOOKUP --> GYRO_C
    LOOKUP --> BARO_C
```

#### **Polynomial Compensation Model**
```cpp
// Temperature compensation using polynomial models
Vector3f TemperatureCompensation::compensate_accelerometer(
    const Vector3f &accel_raw, float temperature, int sensor_index)
{
    if (!_accel_cal[sensor_index].valid) {
        return accel_raw;  // No compensation available
    }

    const SensorCalibration &cal = _accel_cal[sensor_index];
    float temp_delta = temperature - cal.ref_temp;

    Vector3f accel_compensated;

    for (int axis = 0; axis < 3; axis++) {
        // 1. BIAS COMPENSATION (4th order polynomial)
        float bias_correction = cal.bias_coeffs[axis][0] +
                               cal.bias_coeffs[axis][1] * temp_delta +
                               cal.bias_coeffs[axis][2] * temp_delta * temp_delta +
                               cal.bias_coeffs[axis][3] * temp_delta * temp_delta * temp_delta;

        // 2. SCALE FACTOR COMPENSATION
        float scale_correction = 1.0f +
                                cal.scale_coeffs[axis][0] * temp_delta +
                                cal.scale_coeffs[axis][1] * temp_delta * temp_delta;

        // 3. APPLY CORRECTIONS
        accel_compensated(axis) = (accel_raw(axis) - bias_correction) * scale_correction;
    }

    return accel_compensated;
}

Vector3f TemperatureCompensation::compensate_gyroscope(
    const Vector3f &gyro_raw, float temperature, int sensor_index)
{
    if (!_gyro_cal[sensor_index].valid) {
        return gyro_raw;
    }

    const SensorCalibration &cal = _gyro_cal[sensor_index];
    float temp_delta = temperature - cal.ref_temp;

    Vector3f gyro_compensated;

    for (int axis = 0; axis < 3; axis++) {
        // Gyroscope bias is highly temperature dependent
        float bias_correction = cal.bias_coeffs[axis][0] +
                               cal.bias_coeffs[axis][1] * temp_delta +
                               cal.bias_coeffs[axis][2] * temp_delta * temp_delta +
                               cal.bias_coeffs[axis][3] * temp_delta * temp_delta * temp_delta;

        // Scale factor typically less temperature sensitive for gyros
        float scale_correction = 1.0f + cal.scale_coeffs[axis][0] * temp_delta;

        gyro_compensated(axis) = (gyro_raw(axis) - bias_correction) * scale_correction;
    }

    return gyro_compensated;
}
```

#### **Calibration Data Management**
```cpp
// Calibration parameter loading and validation
int TemperatureCompensation::initialize_parameter_handles(ParameterHandles &parameter_handles)
{
    // Load calibration parameters from system parameters
    for (int sensor = 0; sensor < MAX_ACCEL_COUNT; sensor++) {
        char param_name[17];

        // Reference temperature
        snprintf(param_name, sizeof(param_name), "TC_A%d_TREF", sensor);
        parameter_handles.accel_ref_temp[sensor] = param_find(param_name);

        // Bias polynomial coefficients
        for (int axis = 0; axis < 3; axis++) {
            for (int coeff = 0; coeff < 4; coeff++) {
                snprintf(param_name, sizeof(param_name), "TC_A%d_X%d_%d",
                        sensor, axis, coeff);
                parameter_handles.accel_bias_coeffs[sensor][axis][coeff] = param_find(param_name);
            }
        }

        // Scale factor coefficients
        for (int axis = 0; axis < 3; axis++) {
            for (int coeff = 0; coeff < 4; coeff++) {
                snprintf(param_name, sizeof(param_name), "TC_A%d_SCL%d_%d",
                        sensor, axis, coeff);
                parameter_handles.accel_scale_coeffs[sensor][axis][coeff] = param_find(param_name);
            }
        }
    }

    return PX4_OK;
}

bool TemperatureCompensation::validate_calibration_data(int sensor_index, SensorType sensor_type)
{
    switch (sensor_type) {
        case SensorType::Accelerometer: {
            const SensorCalibration &cal = _accel_cal[sensor_index];

            // Check if reference temperature is reasonable
            if (cal.ref_temp < -40.0f || cal.ref_temp > 85.0f) {
                return false;
            }

            // Check if bias coefficients are within reasonable bounds
            for (int axis = 0; axis < 3; axis++) {
                if (fabsf(cal.bias_coeffs[axis][0]) > 2.0f) {  // > 2 m/s² bias
                    return false;
                }
            }

            // Check if scale factors are reasonable (±10%)
            for (int axis = 0; axis < 3; axis++) {
                if (fabsf(cal.scale_coeffs[axis][0]) > 0.1f) {
                    return false;
                }
            }

            return true;
        }

        case SensorType::Gyroscope: {
            // Similar validation for gyroscope...
            return true;
        }
    }

    return false;
}
```

#### **Real-Time Compensation Pipeline**
```cpp
void TemperatureCompensation::Run()
{
    // Process all accelerometer instances
    for (int i = 0; i < MAX_ACCEL_COUNT; i++) {
        sensor_accel_s accel_report;
        if (_sensor_accel_subs[i].copy(&accel_report)) {

            // Apply temperature compensation
            Vector3f accel_raw(accel_report.x, accel_report.y, accel_report.z);
            Vector3f accel_compensated = compensate_accelerometer(
                accel_raw, accel_report.temperature, i);

            // Update compensated values
            accel_report.x = accel_compensated(0);
            accel_report.y = accel_compensated(1);
            accel_report.z = accel_compensated(2);
            accel_report.temperature_available = true;

            // Publish compensated data
            _sensor_accel_compensated_pubs[i].publish(accel_report);
        }
    }

    // Process all gyroscope instances
    for (int i = 0; i < MAX_GYRO_COUNT; i++) {
        sensor_gyro_s gyro_report;
        if (_sensor_gyro_subs[i].copy(&gyro_report)) {

            Vector3f gyro_raw(gyro_report.x, gyro_report.y, gyro_report.z);
            Vector3f gyro_compensated = compensate_gyroscope(
                gyro_raw, gyro_report.temperature, i);

            gyro_report.x = gyro_compensated(0);
            gyro_report.y = gyro_compensated(1);
            gyro_report.z = gyro_compensated(2);

            _sensor_gyro_compensated_pubs[i].publish(gyro_report);
        }
    }
}
```

---

### **4. Gyro Calibration Module**

**Location**: `src/modules/gyro_calibration/`
**Main Class**: `GyroCalibration`
**Work Queue**: `wq_lp_default`

#### **Purpose and Functionality**
The **Gyro Calibration** module provides automated and manual gyroscope bias calibration procedures, essential for accurate attitude estimation and stable flight.

```cpp
class GyroCalibration : public ModuleBase<GyroCalibration>, public ModuleParams
{
    // Calibration state machine
    enum class CalibrationState {
        IDLE,
        COLLECTING_SAMPLES,
        CALCULATING_BIAS,
        APPLYING_CALIBRATION,
        COMPLETE,
        FAILED
    } _state{CalibrationState::IDLE};

    // Sample collection
    static constexpr int SAMPLES_REQUIRED = 3000;  // 3 seconds at 1kHz
    Vector3f _sample_sum[MAX_GYRO_COUNT];
    int _sample_count[MAX_GYRO_COUNT];

    // Calibration results
    Vector3f _gyro_bias[MAX_GYRO_COUNT];
    float _bias_variance[MAX_GYRO_COUNT];
};
```

#### **Gyro Calibration State Machine**
```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> COLLECTING : Start calibration
    COLLECTING --> CALCULATING : Samples complete
    CALCULATING --> APPLYING : Bias computed
    APPLYING --> COMPLETE : Calibration saved
    COMPLETE --> IDLE : Reset

    COLLECTING --> FAILED : Motion detected
    CALCULATING --> FAILED : High variance
    APPLYING --> FAILED : Save error
    FAILED --> IDLE : Retry

    note right of COLLECTING : Vehicle must be stationary
    note right of CALCULATING : Compute bias and validate
    note right of APPLYING : Save to parameters
```

#### **Calibration Process Implementation**
```cpp
void GyroCalibration::Run()
{
    switch (_state) {
        case CalibrationState::IDLE:
            // Wait for calibration command
            handle_calibration_command();
            break;

        case CalibrationState::COLLECTING_SAMPLES:
            collect_gyro_samples();
            break;

        case CalibrationState::CALCULATING_BIAS:
            calculate_gyro_bias();
            break;

        case CalibrationState::APPLYING_CALIBRATION:
            apply_calibration();
            break;

        case CalibrationState::COMPLETE:
            publish_calibration_complete();
            _state = CalibrationState::IDLE;
            break;

        case CalibrationState::FAILED:
            publish_calibration_failed();
            _state = CalibrationState::IDLE;
            break;
    }
}

void GyroCalibration::collect_gyro_samples()
{
    // Check vehicle is stationary
    vehicle_local_position_s local_pos;
    if (_local_pos_sub.copy(&local_pos)) {
        Vector3f velocity(local_pos.vx, local_pos.vy, local_pos.vz);
        if (velocity.norm() > MAX_VELOCITY_FOR_CALIBRATION) {
            PX4_ERR("Vehicle moving during gyro calibration");
            _state = CalibrationState::FAILED;
            return;
        }
    }

    // Collect samples from all gyroscopes
    bool all_sensors_complete = true;

    for (int i = 0; i < MAX_GYRO_COUNT; i++) {
        sensor_gyro_s gyro_data;
        if (_sensor_gyro_subs[i].copy(&gyro_data)) {

            if (_sample_count[i] < SAMPLES_REQUIRED) {
                // Accumulate samples
                _sample_sum[i] += Vector3f(gyro_data.x, gyro_data.y, gyro_data.z);
                _sample_count[i]++;

                // Check for excessive motion during calibration
                Vector3f current_sample(gyro_data.x, gyro_data.y, gyro_data.z);
                if (current_sample.norm() > MAX_GYRO_MOVEMENT) {
                    PX4_WARN("Excessive gyro movement detected");
                    reset_calibration();
                    return;
                }

                all_sensors_complete = false;
            }
        }
    }

    // Progress indication
    float progress = (float)_sample_count[0] / SAMPLES_REQUIRED;
    publish_calibration_progress(progress);

    if (all_sensors_complete) {
        _state = CalibrationState::CALCULATING_BIAS;
    }
}

void GyroCalibration::calculate_gyro_bias()
{
    bool all_calibrations_valid = true;

    for (int i = 0; i < MAX_GYRO_COUNT; i++) {
        if (_sample_count[i] >= SAMPLES_REQUIRED) {

            // Calculate mean bias
            _gyro_bias[i] = _sample_sum[i] / (float)_sample_count[i];

            // Calculate variance to validate calibration quality
            Vector3f variance_sum(0.0f, 0.0f, 0.0f);
            int variance_samples = 0;

            // Re-read samples to calculate variance
            for (int sample = 0; sample < SAMPLES_REQUIRED; sample++) {
                sensor_gyro_s gyro_data;
                if (_sensor_gyro_subs[i].copy(&gyro_data)) {
                    Vector3f sample_vec(gyro_data.x, gyro_data.y, gyro_data.z);
                    Vector3f deviation = sample_vec - _gyro_bias[i];
                    variance_sum += deviation.emult(deviation);  // Element-wise multiplication
                    variance_samples++;
                }
            }

            Vector3f variance = variance_sum / (float)variance_samples;
            _bias_variance[i] = variance.norm();

            // Validate calibration quality
            if (_bias_variance[i] > MAX_BIAS_VARIANCE) {
                PX4_ERR("Gyro %d calibration variance too high: %.6f", i, (double)_bias_variance[i]);
                all_calibrations_valid = false;
            }

            // Validate bias magnitude
            if (_gyro_bias[i].norm() > MAX_BIAS_MAGNITUDE) {
                PX4_ERR("Gyro %d bias too large: %.6f rad/s", i, (double)_gyro_bias[i].norm());
                all_calibrations_valid = false;
            }

            PX4_INFO("Gyro %d bias: [%.6f, %.6f, %.6f] rad/s, variance: %.6f",
                    i, (double)_gyro_bias[i](0), (double)_gyro_bias[i](1),
                    (double)_gyro_bias[i](2), (double)_bias_variance[i]);
        }
    }

    if (all_calibrations_valid) {
        _state = CalibrationState::APPLYING_CALIBRATION;
    } else {
        _state = CalibrationState::FAILED;
    }
}

void GyroCalibration::apply_calibration()
{
    bool all_parameters_saved = true;

    for (int i = 0; i < MAX_GYRO_COUNT; i++) {
        if (_sample_count[i] >= SAMPLES_REQUIRED) {

            // Save bias parameters
            char param_name[16];

            snprintf(param_name, sizeof(param_name), "CAL_GYRO%d_XOFF", i);
            if (param_set_no_notification(param_find(param_name), &_gyro_bias[i](0)) != PX4_OK) {
                all_parameters_saved = false;
            }

            snprintf(param_name, sizeof(param_name), "CAL_GYRO%d_YOFF", i);
            if (param_set_no_notification(param_find(param_name), &_gyro_bias[i](1)) != PX4_OK) {
                all_parameters_saved = false;
            }

            snprintf(param_name, sizeof(param_name), "CAL_GYRO%d_ZOFF", i);
            if (param_set_no_notification(param_find(param_name), &_gyro_bias[i](2)) != PX4_OK) {
                all_parameters_saved = false;
            }

            // Save calibration timestamp
            uint64_t calibration_time = hrt_absolute_time();
            snprintf(param_name, sizeof(param_name), "CAL_GYRO%d_ID", i);
            if (param_set_no_notification(param_find(param_name), &calibration_time) != PX4_OK) {
                all_parameters_saved = false;
            }
        }
    }

    if (all_parameters_saved) {
        // Save parameters to storage
        param_notify_changes();
        _state = CalibrationState::COMPLETE;
        PX4_INFO("Gyroscope calibration completed successfully");
    } else {
        _state = CalibrationState::FAILED;
        PX4_ERR("Failed to save gyroscope calibration parameters");
    }
}
```

#### **Automatic vs Manual Calibration**
```cpp
// Different calibration modes
enum class CalibrationType {
    AUTOMATIC,      // Triggered during startup if needed
    MANUAL,         // User-initiated calibration
    TEMPERATURE     // Part of temperature compensation
};

void GyroCalibration::handle_calibration_command()
{
    vehicle_command_s cmd;
    if (_vehicle_command_sub.copy(&cmd)) {

        if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION) {

            if (cmd.param1 > 0.5f) {  // Gyro calibration requested

                // Check pre-conditions
                if (!check_calibration_preconditions()) {
                    publish_calibration_failed();
                    return;
                }

                // Determine calibration type
                CalibrationType cal_type = (cmd.param7 > 0.5f) ?
                    CalibrationType::AUTOMATIC : CalibrationType::MANUAL;

                start_calibration(cal_type);
            }
        }
    }
}

bool GyroCalibration::check_calibration_preconditions()
{
    // 1. Vehicle must be disarmed
    actuator_armed_s armed;
    if (_actuator_armed_sub.copy(&armed) && armed.armed) {
        PX4_ERR("Cannot calibrate while armed");
        return false;
    }

    // 2. Vehicle must be on ground
    vehicle_land_detected_s land_detected;
    if (_land_detected_sub.copy(&land_detected) && !land_detected.landed) {
        PX4_ERR("Cannot calibrate while airborne");
        return false;
    }

    // 3. Vehicle must be stationary
    vehicle_local_position_s local_pos;
    if (_local_pos_sub.copy(&local_pos)) {
        Vector3f velocity(local_pos.vx, local_pos.vy, local_pos.vz);
        if (velocity.norm() > MAX_VELOCITY_FOR_CALIBRATION) {
            PX4_ERR("Vehicle must be stationary for calibration");
            return false;
        }
    }

    return true;
}
```

---

## **Sensor Module Integration Summary**

These sensor processing modules work together to provide clean, calibrated data:

1. **Battery Status** ← Critical for flight safety and mission planning
2. **Airspeed Selector** ← Multi-sensor validation for reliable airspeed
3. **Temperature Compensation** ← Maintains sensor accuracy across conditions
4. **Gyro Calibration** ← Ensures accurate attitude estimation foundation

---

## **Communication and Interface Modules - Detailed**

### **1. Manual Control Module**

**Location**: `src/modules/manual_control/`
**Main Class**: `ManualControl`
**Work Queue**: `wq_lp_default`

#### **Purpose and Functionality**
The **Manual Control** module processes pilot input from RC transmitters, validates control signals, and manages manual flight mode transitions and emergency procedures.

```cpp
class ManualControl : public ModuleBase<ManualControl>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Input processing
    uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
    uORB::Subscription _manual_control_input_sub{ORB_ID(manual_control_input)};

    // Output publications
    uORB::Publication<manual_control_setpoint_s> _manual_control_setpoint_pub{ORB_ID(manual_control_setpoint)};
    uORB::Publication<manual_control_switches_s> _manual_control_switches_pub{ORB_ID(manual_control_switches)};
    uORB::Publication<action_request_s> _action_request_pub{ORB_ID(action_request)};

    // State management
    manual_control_setpoint_s _manual_control_setpoint{};
    manual_control_switches_s _manual_control_switches{};

    // Failsafe and validation
    systemlib::Hysteresis _rc_signal_lost_hysteresis{false};
    hrt_abstime _last_valid_manual_control_setpoint{0};
};
```

#### **RC Input Processing Pipeline**
```mermaid
flowchart LR
    subgraph "RC Hardware"
        TX[RC Transmitter]
        RX[RC Receiver]
        PPM[PPM/SBUS Signal]
    end

    subgraph "Signal Processing"
        DECODE[Signal Decoding]
        CALIB[Channel Calibration]
        MAP[Function Mapping]
        FAILSAFE[Failsafe Detection]
    end

    subgraph "Manual Control Module"
        VALID[Input Validation]
        SWITCH[Switch Processing]
        SETPOINT[Setpoint Generation]
        ACTION[Action Requests]
    end

    subgraph "Output"
        MANUAL_SP[manual_control_setpoint]
        SWITCHES[manual_control_switches]
        ACTIONS[action_request]
    end

    TX --> RX
    RX --> PPM
    PPM --> DECODE
    DECODE --> CALIB
    CALIB --> MAP
    MAP --> FAILSAFE

    FAILSAFE --> VALID
    VALID --> SWITCH
    SWITCH --> SETPOINT
    SETPOINT --> ACTION

    SETPOINT --> MANUAL_SP
    SWITCH --> SWITCHES
    ACTION --> ACTIONS
```

#### **RC Channel Processing and Validation**
```cpp
void ManualControl::Run()
{
    // 1. READ RC INPUT
    input_rc_s input_rc;
    manual_control_input_s manual_control_input;

    bool rc_updated = _input_rc_sub.copy(&input_rc);
    bool manual_updated = _manual_control_input_sub.copy(&manual_control_input);

    if (rc_updated || manual_updated) {

        // 2. VALIDATE INPUT SIGNAL
        bool input_valid = validate_manual_control_input(input_rc, manual_control_input);

        if (input_valid) {
            // 3. PROCESS CONTROL STICKS
            process_control_sticks(input_rc, manual_control_input);

            // 4. PROCESS SWITCHES
            process_control_switches(input_rc, manual_control_input);

            // 5. GENERATE ACTION REQUESTS
            generate_action_requests();

            // 6. PUBLISH OUTPUTS
            publish_manual_control_setpoint();
            publish_manual_control_switches();

            _last_valid_manual_control_setpoint = hrt_absolute_time();
            _rc_signal_lost_hysteresis.set_state_and_update(false, hrt_absolute_time());

        } else {
            // 7. HANDLE RC LOSS
            handle_rc_signal_loss();
        }
    } else {
        // 8. CHECK FOR RC TIMEOUT
        check_rc_timeout();
    }
}

bool ManualControl::validate_manual_control_input(const input_rc_s &input_rc,
                                                  const manual_control_input_s &manual_input)
{
    // RC input validation
    if (input_rc.timestamp > 0) {
        // Check signal quality
        if (input_rc.rssi < RC_RSSI_MIN_THRESHOLD) {
            PX4_DEBUG("RC RSSI too low: %d", input_rc.rssi);
            return false;
        }

        // Check for reasonable channel values
        for (int i = 0; i < input_rc.channel_count; i++) {
            if (input_rc.values[i] < RC_CHANNEL_MIN || input_rc.values[i] > RC_CHANNEL_MAX) {
                PX4_DEBUG("RC channel %d out of range: %d", i, input_rc.values[i]);
                return false;
            }
        }

        // Check minimum required channels
        if (input_rc.channel_count < RC_MIN_CHANNELS_REQUIRED) {
            PX4_DEBUG("Insufficient RC channels: %d", input_rc.channel_count);
            return false;
        }

        return true;
    }

    // Manual control input validation (companion computer, joystick)
    if (manual_input.timestamp > 0) {
        // Check data age
        hrt_abstime age = hrt_absolute_time() - manual_input.timestamp;
        if (age > MANUAL_INPUT_TIMEOUT_US) {
            return false;
        }

        // Validate stick ranges
        if (fabsf(manual_input.x) > 1.0f || fabsf(manual_input.y) > 1.0f ||
            fabsf(manual_input.z) > 1.0f || fabsf(manual_input.r) > 1.0f) {
            return false;
        }

        return true;
    }

    return false;
}
```

#### **Control Stick Processing with Dead Zones**
```cpp
void ManualControl::process_control_sticks(const input_rc_s &input_rc,
                                          const manual_control_input_s &manual_input)
{
    if (input_rc.timestamp > 0) {
        // RC stick processing
        _manual_control_setpoint.x = apply_deadzone_and_scaling(
            input_rc.values[RC_CHANNEL_ROLL],
            _param_rc_dz_roll.get(),
            _param_rc_scale_roll.get()
        );

        _manual_control_setpoint.y = apply_deadzone_and_scaling(
            input_rc.values[RC_CHANNEL_PITCH],
            _param_rc_dz_pitch.get(),
            _param_rc_scale_pitch.get()
        );

        _manual_control_setpoint.z = apply_deadzone_and_scaling(
            input_rc.values[RC_CHANNEL_THROTTLE],
            _param_rc_dz_throttle.get(),
            _param_rc_scale_throttle.get()
        );

        _manual_control_setpoint.r = apply_deadzone_and_scaling(
            input_rc.values[RC_CHANNEL_YAW],
            _param_rc_dz_yaw.get(),
            _param_rc_scale_yaw.get()
        );

    } else if (manual_input.timestamp > 0) {
        // Direct manual input (already normalized)
        _manual_control_setpoint.x = manual_input.x;
        _manual_control_setpoint.y = manual_input.y;
        _manual_control_setpoint.z = manual_input.z;
        _manual_control_setpoint.r = manual_input.r;
    }

    // Apply exponential curves for fine control
    _manual_control_setpoint.x = apply_expo_curve(_manual_control_setpoint.x, _param_rc_expo_roll.get());
    _manual_control_setpoint.y = apply_expo_curve(_manual_control_setpoint.y, _param_rc_expo_pitch.get());
    _manual_control_setpoint.r = apply_expo_curve(_manual_control_setpoint.r, _param_rc_expo_yaw.get());
}

float ManualControl::apply_deadzone_and_scaling(uint16_t raw_value, float deadzone, float scale)
{
    // Convert to normalized range [-1, 1]
    float normalized = ((float)raw_value - RC_CHANNEL_CENTER) / (RC_CHANNEL_RANGE / 2.0f);

    // Apply deadzone
    if (fabsf(normalized) < deadzone) {
        return 0.0f;
    }

    // Remove deadzone and rescale
    if (normalized > 0.0f) {
        normalized = (normalized - deadzone) / (1.0f - deadzone);
    } else {
        normalized = (normalized + deadzone) / (1.0f - deadzone);
    }

    // Apply scaling factor
    normalized *= scale;

    // Ensure bounds
    return math::constrain(normalized, -1.0f, 1.0f);
}

float ManualControl::apply_expo_curve(float input, float expo_factor)
{
    // Exponential curve for fine control around center
    // expo_factor: 0 = linear, 1 = full exponential
    if (expo_factor > 0.01f) {
        float expo_input = fabsf(input);
        float expo_output = expo_factor * expo_input * expo_input * expo_input +
                           (1.0f - expo_factor) * expo_input;
        return (input >= 0.0f) ? expo_output : -expo_output;
    }

    return input;
}
```

#### **Switch Processing and Mode Management**
```cpp
void ManualControl::process_control_switches(const input_rc_s &input_rc,
                                            const manual_control_input_s &manual_input)
{
    manual_control_switches_s switches{};

    if (input_rc.timestamp > 0) {
        // Flight mode switch (3-position)
        uint16_t mode_switch_value = input_rc.values[RC_CHANNEL_MODE];
        if (mode_switch_value < RC_MODE_MANUAL_THRESHOLD) {
            switches.mode_slot = manual_control_switches_s::MODE_SLOT_1;  // Manual
        } else if (mode_switch_value < RC_MODE_AUTO_THRESHOLD) {
            switches.mode_slot = manual_control_switches_s::MODE_SLOT_2;  // Stabilized
        } else {
            switches.mode_slot = manual_control_switches_s::MODE_SLOT_3;  // Auto
        }

        // Return switch (2-position)
        switches.return_switch = (input_rc.values[RC_CHANNEL_RETURN] > RC_SWITCH_THRESHOLD) ?
            manual_control_switches_s::SWITCH_POS_ON : manual_control_switches_s::SWITCH_POS_OFF;

        // Kill switch (2-position)
        switches.kill_switch = (input_rc.values[RC_CHANNEL_KILL] > RC_SWITCH_THRESHOLD) ?
            manual_control_switches_s::SWITCH_POS_ON : manual_control_switches_s::SWITCH_POS_OFF;

        // Arm switch (2-position)
        switches.arm_switch = (input_rc.values[RC_CHANNEL_ARM] > RC_SWITCH_THRESHOLD) ?
            manual_control_switches_s::SWITCH_POS_ON : manual_control_switches_s::SWITCH_POS_OFF;

        // Gear switch (landing gear)
        switches.gear_switch = (input_rc.values[RC_CHANNEL_GEAR] > RC_SWITCH_THRESHOLD) ?
            manual_control_switches_s::SWITCH_POS_ON : manual_control_switches_s::SWITCH_POS_OFF;

        // Camera trigger
        switches.photo_trigger = detect_switch_transition(
            input_rc.values[RC_CHANNEL_PHOTO], RC_SWITCH_THRESHOLD);

    } else if (manual_input.timestamp > 0) {
        // Process switches from manual control input
        switches.mode_slot = manual_input.mode_slot;
        switches.return_switch = manual_input.return_switch;
        switches.kill_switch = manual_input.kill_switch;
        // ... other switches
    }

    // Detect switch changes and generate events
    if (switches.return_switch != _manual_control_switches.return_switch) {
        if (switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
            generate_action_request(action_request_s::ACTION_RTL);
        }
    }

    if (switches.kill_switch != _manual_control_switches.kill_switch) {
        if (switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON) {
            generate_action_request(action_request_s::ACTION_KILL);
        }
    }

    _manual_control_switches = switches;
}

bool ManualControl::detect_switch_transition(uint16_t current_value, uint16_t threshold)
{
    static uint16_t previous_value = RC_CHANNEL_CENTER;
    static bool previous_state = false;

    bool current_state = (current_value > threshold);
    bool transition = (current_state && !previous_state);  // Rising edge

    previous_value = current_value;
    previous_state = current_state;

    return transition;
}
```

#### **RC Signal Loss Handling**
```cpp
void ManualControl::handle_rc_signal_loss()
{
    // Update signal loss hysteresis
    _rc_signal_lost_hysteresis.set_state_and_update(true, hrt_absolute_time());

    if (_rc_signal_lost_hysteresis.get_state()) {
        // RC signal definitively lost

        // Clear control setpoints for safety
        _manual_control_setpoint.x = 0.0f;
        _manual_control_setpoint.y = 0.0f;
        _manual_control_setpoint.z = 0.0f;  // Throttle to 0
        _manual_control_setpoint.r = 0.0f;
        _manual_control_setpoint.valid = false;

        // Trigger RC loss action based on parameters
        switch (_param_rc_loss_action.get()) {
            case RC_LOSS_ACTION_RTL:
                generate_action_request(action_request_s::ACTION_RTL);
                break;

            case RC_LOSS_ACTION_LAND:
                generate_action_request(action_request_s::ACTION_LAND);
                break;

            case RC_LOSS_ACTION_HOLD:
                generate_action_request(action_request_s::ACTION_LOITER);
                break;

            case RC_LOSS_ACTION_TERMINATE:
                generate_action_request(action_request_s::ACTION_KILL);
                break;
        }

        // Publish event
        events::send<uint8_t>(events::ID("manual_control_lost"),
                             events::Log::Alert,
                             "Manual control lost, executing RC loss action: {1}",
                             _param_rc_loss_action.get());
    }
}

void ManualControl::check_rc_timeout()
{
    hrt_abstime now = hrt_absolute_time();
    hrt_abstime time_since_last_valid = now - _last_valid_manual_control_setpoint;

    if (time_since_last_valid > RC_TIMEOUT_US) {
        handle_rc_signal_loss();
    }
}
```

---

### **2. RC Update Module**

**Location**: `src/modules/rc_update/`
**Main Class**: `RCUpdate`
**Work Queue**: `wq_lp_default`

#### **Purpose and Functionality**
The **RC Update** module handles low-level RC signal processing, channel mapping, calibration, and conversion of raw RC signals into standardized control inputs.

```cpp
namespace rc_update
{
class RCUpdate : public ModuleBase<RCUpdate>, public ModuleParams, public px4::ScheduledWorkItem
{
    // RC input subscription
    uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

    // Output publications
    uORB::Publication<manual_control_input_s> _manual_control_input_pub{ORB_ID(manual_control_input)};
    uORB::Publication<manual_control_switches_s> _manual_control_switches_pub{ORB_ID(manual_control_switches)};

    // RC parameters and calibration
    struct RCChannelConfig {
        float min;
        float max;
        float center;
        float deadzone;
        float scale;
        int8_t function;
        bool reverse;
    } _rc_channels[RC_MAX_CHANNELS];

    // Signal processing
    perf_counter_t _loop_perf;
    bool _rc_calibration_mode{false};
};
}
```

#### **RC Signal Processing Architecture**
```mermaid
flowchart TB
    subgraph "Hardware Layer"
        SBUS[SBUS Protocol]
        PPM[PPM Signal]
        DSM[DSM/DSMX]
        CRSF[CRSF/ELRS]
    end

    subgraph "Driver Layer"
        RC_DRV[RC Input Driver]
        UART[UART Interface]
        GPIO[GPIO Timer]
    end

    subgraph "RC Update Module"
        RAW[Raw Channel Processing]
        CAL[Channel Calibration]
        MAP[Function Mapping]
        FILTER[Signal Filtering]
    end

    subgraph "Output Processing"
        STICKS[Stick Commands]
        SWITCHES[Switch States]
        AUX[Auxiliary Channels]
    end

    SBUS --> RC_DRV
    PPM --> RC_DRV
    DSM --> RC_DRV
    CRSF --> RC_DRV

    RC_DRV --> UART
    RC_DRV --> GPIO

    UART --> RAW
    GPIO --> RAW

    RAW --> CAL
    CAL --> MAP
    MAP --> FILTER

    FILTER --> STICKS
    FILTER --> SWITCHES
    FILTER --> AUX
```

#### **RC Channel Calibration and Mapping**
```cpp
void RCUpdate::Run()
{
    input_rc_s input_rc;

    if (_input_rc_sub.copy(&input_rc)) {

        // 1. UPDATE RC PARAMETERS
        update_rc_parameters();

        // 2. PROCESS RAW CHANNELS
        process_raw_rc_channels(input_rc);

        // 3. APPLY CALIBRATION
        apply_channel_calibration(input_rc);

        // 4. MAP TO FUNCTIONS
        map_channels_to_functions(input_rc);

        // 5. GENERATE OUTPUTS
        generate_manual_control_input(input_rc);
        generate_manual_control_switches(input_rc);

        // 6. HANDLE SPECIAL MODES
        if (_rc_calibration_mode) {
            handle_calibration_mode(input_rc);
        }
    }
}

void RCUpdate::apply_channel_calibration(input_rc_s &input_rc)
{
    for (int i = 0; i < input_rc.channel_count && i < RC_MAX_CHANNELS; i++) {
        RCChannelConfig &config = _rc_channels[i];

        // Get raw value
        float raw_value = (float)input_rc.values[i];

        // Apply reverse if configured
        if (config.reverse) {
            raw_value = config.max + config.min - raw_value;
        }

        // Convert to normalized range [-1, 1] or [0, 1] for throttle
        float normalized;

        if (i == RC_FUNCTION_THROTTLE) {
            // Throttle: [0, 1] range
            normalized = (raw_value - config.min) / (config.max - config.min);
            normalized = math::constrain(normalized, 0.0f, 1.0f);

        } else {
            // Other channels: [-1, 1] range
            if (raw_value > config.center) {
                normalized = (raw_value - config.center) / (config.max - config.center);
            } else {
                normalized = (raw_value - config.center) / (config.center - config.min);
            }
            normalized = math::constrain(normalized, -1.0f, 1.0f);

            // Apply deadzone
            if (fabsf(normalized) < config.deadzone) {
                normalized = 0.0f;
            } else {
                // Scale beyond deadzone
                if (normalized > 0.0f) {
                    normalized = (normalized - config.deadzone) / (1.0f - config.deadzone);
                } else {
                    normalized = (normalized + config.deadzone) / (1.0f - config.deadzone);
                }
            }
        }

        // Apply scale factor
        normalized *= config.scale;

        // Store calibrated value (convert back to uint16_t for compatibility)
        input_rc.values[i] = (uint16_t)((normalized + 1.0f) * 500.0f + 1000.0f);
    }
}

void RCUpdate::map_channels_to_functions(const input_rc_s &input_rc)
{
    // Clear function mappings
    for (int func = 0; func < RC_FUNCTION_MAX; func++) {
        _rc_function_values[func] = UINT16_MAX;  // Invalid
    }

    // Map channels to functions based on parameter configuration
    for (int ch = 0; ch < input_rc.channel_count && ch < RC_MAX_CHANNELS; ch++) {
        int8_t function = _rc_channels[ch].function;

        if (function >= 0 && function < RC_FUNCTION_MAX) {
            _rc_function_values[function] = input_rc.values[ch];
        }
    }

    // Validate minimum required functions
    bool required_functions_mapped =
        (_rc_function_values[RC_FUNCTION_ROLL] != UINT16_MAX) &&
        (_rc_function_values[RC_FUNCTION_PITCH] != UINT16_MAX) &&
        (_rc_function_values[RC_FUNCTION_THROTTLE] != UINT16_MAX) &&
        (_rc_function_values[RC_FUNCTION_YAW] != UINT16_MAX);

    if (!required_functions_mapped) {
        PX4_ERR("Required RC functions not mapped");
        // Fall back to default channel mapping
        use_default_channel_mapping();
    }
}
```

#### **Advanced RC Protocol Support**
```cpp
// Support for different RC protocols with protocol-specific processing
void RCUpdate::process_rc_protocol(const input_rc_s &input_rc)
{
    switch (input_rc.input_source) {
        case input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM:
            process_ppm_input(input_rc);
            break;

        case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS:
            process_sbus_input(input_rc);
            break;

        case input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM:
            process_dsm_input(input_rc);
            break;

        case input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24:
            process_st24_input(input_rc);
            break;

        case input_rc_s::RC_INPUT_SOURCE_MAVLINK:
            process_mavlink_rc_input(input_rc);
            break;

        default:
            PX4_DEBUG("Unknown RC input source: %d", input_rc.input_source);
            break;
    }
}

void RCUpdate::process_sbus_input(const input_rc_s &input_rc)
{
    // SBUS-specific processing

    // Check for SBUS failsafe flag
    if (input_rc.rc_failsafe) {
        handle_sbus_failsafe();
        return;
    }

    // Check for frame drop indication
    if (input_rc.rc_lost_frame_count > _last_frame_count + 1) {
        _frame_drops += (input_rc.rc_lost_frame_count - _last_frame_count - 1);
        PX4_DEBUG("SBUS frame drops detected: %d", _frame_drops);
    }
    _last_frame_count = input_rc.rc_lost_frame_count;

    // Process channels (SBUS provides 16 channels)
    for (int i = 0; i < math::min(input_rc.channel_count, 16); i++) {
        // SBUS channels are 11-bit (0-2047)
        uint16_t sbus_value = input_rc.values[i];

        // Convert to standard PWM range (1000-2000µs)
        uint16_t pwm_value = (sbus_value * 875 / 2047) + 1000;
        input_rc.values[i] = pwm_value;
    }
}

void RCUpdate::process_crsf_input(const input_rc_s &input_rc)
{
    // CRSF (Crossfire/ExpressLRS) specific processing

    // CRSF provides higher resolution (11-bit or 12-bit)
    for (int i = 0; i < input_rc.channel_count; i++) {
        uint16_t crsf_value = input_rc.values[i];

        // CRSF range is typically 172-1811 for 11-bit
        // Convert to standard PWM range
        uint16_t pwm_value = ((crsf_value - 172) * 1000 / (1811 - 172)) + 1000;
        input_rc.values[i] = math::constrain(pwm_value, (uint16_t)1000, (uint16_t)2000);
    }

    // Process CRSF telemetry data if available
    if (input_rc.rssi_dbm != 0) {
        process_crsf_telemetry(input_rc);
    }
}
```

#### **RC Calibration Mode**
```cpp
void RCUpdate::handle_calibration_mode(const input_rc_s &input_rc)
{
    static uint16_t channel_min[RC_MAX_CHANNELS];
    static uint16_t channel_max[RC_MAX_CHANNELS];
    static uint16_t channel_center[RC_MAX_CHANNELS];
    static bool calibration_initialized = false;

    if (!calibration_initialized) {
        // Initialize calibration arrays
        for (int i = 0; i < RC_MAX_CHANNELS; i++) {
            channel_min[i] = UINT16_MAX;
            channel_max[i] = 0;
            channel_center[i] = 1500;  // Default center
        }
        calibration_initialized = true;
        PX4_INFO("RC calibration mode started");
    }

    // Update min/max values for each channel
    for (int i = 0; i < input_rc.channel_count && i < RC_MAX_CHANNELS; i++) {
        uint16_t value = input_rc.values[i];

        if (value < channel_min[i]) {
            channel_min[i] = value;
        }
        if (value > channel_max[i]) {
            channel_max[i] = value;
        }
    }

    // Detect stick centers (when sticks are neutral)
    detect_stick_centers(input_rc, channel_center);

    // Publish calibration progress
    publish_calibration_data(channel_min, channel_max, channel_center);
}

void RCUpdate::detect_stick_centers(const input_rc_s &input_rc, uint16_t *channel_center)
{
    static uint16_t center_samples[RC_MAX_CHANNELS][CENTER_SAMPLE_COUNT];
    static int sample_index = 0;
    static int samples_collected = 0;

    // Check if sticks are likely in center position (low movement)
    bool sticks_centered = check_sticks_stationary(input_rc);

    if (sticks_centered && samples_collected < CENTER_SAMPLE_COUNT) {
        // Collect center samples
        for (int i = 0; i < input_rc.channel_count && i < RC_MAX_CHANNELS; i++) {
            center_samples[i][sample_index] = input_rc.values[i];
        }

        sample_index = (sample_index + 1) % CENTER_SAMPLE_COUNT;
        samples_collected++;

        if (samples_collected == CENTER_SAMPLE_COUNT) {
            // Calculate center values from samples
            for (int i = 0; i < RC_MAX_CHANNELS; i++) {
                uint32_t sum = 0;
                for (int j = 0; j < CENTER_SAMPLE_COUNT; j++) {
                    sum += center_samples[i][j];
                }
                channel_center[i] = sum / CENTER_SAMPLE_COUNT;
            }

            PX4_INFO("RC stick centers calibrated");
        }
    }
}

void RCUpdate::save_calibration_parameters()
{
    for (int i = 0; i < RC_MAX_CHANNELS; i++) {
        char param_name[16];

        // Save min values
        snprintf(param_name, sizeof(param_name), "RC%d_MIN", i + 1);
        param_set_no_notification(param_find(param_name), &_rc_channels[i].min);

        // Save max values
        snprintf(param_name, sizeof(param_name), "RC%d_MAX", i + 1);
        param_set_no_notification(param_find(param_name), &_rc_channels[i].max);

        // Save center values
        snprintf(param_name, sizeof(param_name), "RC%d_TRIM", i + 1);
        param_set_no_notification(param_find(param_name), &_rc_channels[i].center);

        // Save deadzone
        snprintf(param_name, sizeof(param_name), "RC%d_DZ", i + 1);
        param_set_no_notification(param_find(param_name), &_rc_channels[i].deadzone);
    }

    // Save parameters to storage
    param_notify_changes();
    PX4_INFO("RC calibration parameters saved");
}
```

---

### **3. Camera Feedback Module**

**Location**: `src/modules/camera_feedback/`
**Main Class**: `CameraFeedback`
**Work Queue**: `wq_lp_default`

#### **Purpose and Functionality**
The **Camera Feedback** module handles camera trigger coordination, photo logging, and integration with mission planning for aerial photography and mapping applications.

```cpp
class CameraFeedback : public ModuleBase<CameraFeedback>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Camera trigger subscription
    uORB::Subscription _camera_trigger_sub{ORB_ID(camera_trigger)};
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

    // Position and attitude for geotagging
    uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

    // Camera feedback publication
    uORB::Publication<camera_capture_s> _camera_capture_pub{ORB_ID(camera_capture)};

    // Photo sequence tracking
    uint32_t _capture_sequence{0};
    bool _trigger_enabled{false};

    // Geotagging data storage
    struct CaptureData {
        uint64_t timestamp;
        double lat, lon;
        float alt_msl, alt_rel;
        float roll, pitch, yaw;
        uint32_t seq;
    };

    static constexpr int MAX_CAPTURE_BUFFER = 50;
    CaptureData _capture_buffer[MAX_CAPTURE_BUFFER];
    int _capture_buffer_head{0};
};
```

#### **Camera Integration Architecture**
```mermaid
flowchart TB
    subgraph "Trigger Sources"
        RC[RC Switch]
        MAV[MAVLink Command]
        MISSION[Mission Item]
        INTERVAL[Time Interval]
        DISTANCE[Distance Based]
    end

    subgraph "Camera Feedback Module"
        TRIGGER[Trigger Processing]
        GEOTAG[Geotagging]
        LOG[Photo Logging]
        SEQUENCE[Sequence Management]
    end

    subgraph "Hardware Interface"
        GPIO_OUT[GPIO Trigger Output]
        GPIO_IN[GPIO Feedback Input]
        PWM[PWM Signal]
        SERIAL[Serial Interface]
    end

    subgraph "Data Sources"
        GPS[GPS Position]
        ATT[Vehicle Attitude]
        ALT[Altitude Data]
        TIME[Timestamp]
    end

    subgraph "Outputs"
        CAPTURE[camera_capture]
        LOG_FILE[Photo Log File]
        MISSION_FB[Mission Feedback]
    end

    RC --> TRIGGER
    MAV --> TRIGGER
    MISSION --> TRIGGER
    INTERVAL --> TRIGGER
    DISTANCE --> TRIGGER

    TRIGGER --> GPIO_OUT
    GPIO_IN --> GEOTAG
    PWM --> GEOTAG
    SERIAL --> GEOTAG

    GPS --> GEOTAG
    ATT --> GEOTAG
    ALT --> GEOTAG
    TIME --> GEOTAG

    GEOTAG --> LOG
    LOG --> SEQUENCE

    SEQUENCE --> CAPTURE
    SEQUENCE --> LOG_FILE
    SEQUENCE --> MISSION_FB
```

#### **Camera Trigger Processing**
```cpp
void CameraFeedback::Run()
{
    // 1. PROCESS CAMERA TRIGGERS
    camera_trigger_s camera_trigger;
    if (_camera_trigger_sub.copy(&camera_trigger)) {
        process_camera_trigger(camera_trigger);
    }

    // 2. PROCESS CAMERA COMMANDS
    vehicle_command_s cmd;
    if (_vehicle_command_sub.copy(&cmd)) {
        if (cmd.command == vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE ||
            cmd.command == vehicle_command_s::VEHICLE_CMD_IMAGE_STOP_CAPTURE) {
            process_camera_command(cmd);
        }
    }

    // 3. HANDLE PERIODIC TRIGGERS
    if (_trigger_enabled) {
        handle_interval_trigger();
        handle_distance_trigger();
    }

    // 4. PROCESS FEEDBACK SIGNALS
    process_camera_feedback();
}

void CameraFeedback::process_camera_trigger(const camera_trigger_s &trigger)
{
    // Collect position and attitude data at trigger time
    vehicle_global_position_s global_pos;
    vehicle_attitude_s attitude;
    vehicle_local_position_s local_pos;

    _vehicle_global_position_sub.copy(&global_pos);
    _vehicle_attitude_sub.copy(&attitude);
    _vehicle_local_position_sub.copy(&local_pos);

    // Create capture data entry
    CaptureData capture;
    capture.timestamp = trigger.timestamp;
    capture.seq = trigger.seq;

    // Position data
    if (global_pos.timestamp > 0) {
        capture.lat = global_pos.lat;
        capture.lon = global_pos.lon;
        capture.alt_msl = global_pos.alt;
    } else {
        // Use invalid markers if GPS not available
        capture.lat = capture.lon = NAN;
        capture.alt_msl = NAN;
    }

    // Relative altitude from local position
    if (local_pos.timestamp > 0) {
        capture.alt_rel = -local_pos.z;  // NED frame, negative Z is up
    } else {
        capture.alt_rel = NAN;
    }

    // Attitude data
    if (attitude.timestamp > 0) {
        capture.roll = attitude.roll;
        capture.pitch = attitude.pitch;
        capture.yaw = attitude.yaw;
    } else {
        capture.roll = capture.pitch = capture.yaw = NAN;
    }

    // Store in buffer for feedback correlation
    store_capture_data(capture);

    // Publish immediate capture event
    publish_camera_capture(capture);

    PX4_DEBUG("Camera trigger processed: seq=%u, lat=%.7f, lon=%.7f, alt=%.2f",
             capture.seq, capture.lat, capture.lon, capture.alt_rel);
}

void CameraFeedback::store_capture_data(const CaptureData &capture)
{
    _capture_buffer[_capture_buffer_head] = capture;
    _capture_buffer_head = (_capture_buffer_head + 1) % MAX_CAPTURE_BUFFER;
}
```

#### **Interval and Distance-Based Triggering**
```cpp
void CameraFeedback::handle_interval_trigger()
{
    static hrt_abstime last_trigger_time = 0;

    float trigger_interval = _param_camera_interval.get();  // seconds

    if (trigger_interval > 0.1f) {  // Minimum 0.1 second interval
        hrt_abstime now = hrt_absolute_time();

        if (now - last_trigger_time > (hrt_abstime)(trigger_interval * 1000000)) {
            // Time to trigger
            send_camera_trigger();
            last_trigger_time = now;
        }
    }
}

void CameraFeedback::handle_distance_trigger()
{
    static Vector2f last_trigger_position{NAN, NAN};

    float trigger_distance = _param_camera_distance.get();  // meters

    if (trigger_distance > 0.5f) {  // Minimum 0.5 meter spacing
        vehicle_local_position_s local_pos;

        if (_vehicle_local_position_sub.copy(&local_pos) && local_pos.xy_valid) {
            Vector2f current_position(local_pos.x, local_pos.y);

            if (!isnan(last_trigger_position(0))) {
                float distance = (current_position - last_trigger_position).length();

                if (distance >= trigger_distance) {
                    send_camera_trigger();
                    last_trigger_position = current_position;
                }
            } else {
                // First position
                last_trigger_position = current_position;
            }
        }
    }
}

void CameraFeedback::send_camera_trigger()
{
    // Send trigger signal via appropriate interface
    switch (_param_camera_interface.get()) {
        case CAMERA_INTERFACE_GPIO:
            send_gpio_trigger();
            break;

        case CAMERA_INTERFACE_PWM:
            send_pwm_trigger();
            break;

        case CAMERA_INTERFACE_MAVLINK:
            send_mavlink_trigger();
            break;

        case CAMERA_INTERFACE_SERIAL:
            send_serial_trigger();
            break;
    }

    _capture_sequence++;
}
```

#### **Camera Feedback and Correlation**
```cpp
void CameraFeedback::process_camera_feedback()
{
    // Process hardware feedback signals (GPIO, serial, etc.)

    // Example: GPIO feedback processing
    if (check_gpio_feedback_signal()) {
        hrt_abstime feedback_time = hrt_absolute_time();

        // Correlate with trigger buffer to find matching capture
        CaptureData *matching_capture = find_matching_capture(feedback_time);

        if (matching_capture != nullptr) {
            // Update capture with confirmed timestamp
            matching_capture->timestamp = feedback_time;

            // Generate final capture event
            publish_camera_capture(*matching_capture);

            // Log to file for post-processing
            log_capture_to_file(*matching_capture);

            PX4_DEBUG("Camera feedback confirmed: seq=%u, delay=%.3fms",
                     matching_capture->seq,
                     (feedback_time - matching_capture->timestamp) / 1000.0f);
        }
    }
}

CameraFeedback::CaptureData* CameraFeedback::find_matching_capture(hrt_abstime feedback_time)
{
    // Search for capture within reasonable time window
    constexpr hrt_abstime MAX_FEEDBACK_DELAY = 500000;  // 500ms

    for (int i = 0; i < MAX_CAPTURE_BUFFER; i++) {
        CaptureData &capture = _capture_buffer[i];

        if (capture.timestamp > 0) {
            hrt_abstime time_diff = hrt_elapsed_time(&capture.timestamp);

            if (time_diff < MAX_FEEDBACK_DELAY) {
                // Found matching capture
                return &capture;
            }
        }
    }

    return nullptr;  // No matching capture found
}

void CameraFeedback::publish_camera_capture(const CaptureData &capture)
{
    camera_capture_s camera_capture{};

    camera_capture.timestamp = capture.timestamp;
    camera_capture.seq = capture.seq;
    camera_capture.lat = capture.lat;
    camera_capture.lon = capture.lon;
    camera_capture.alt = capture.alt_msl;
    camera_capture.ground_distance = capture.alt_rel;
    camera_capture.q[0] = cosf(capture.roll/2) * cosf(capture.pitch/2) * cosf(capture.yaw/2) +
                         sinf(capture.roll/2) * sinf(capture.pitch/2) * sinf(capture.yaw/2);
    camera_capture.q[1] = sinf(capture.roll/2) * cosf(capture.pitch/2) * cosf(capture.yaw/2) -
                         cosf(capture.roll/2) * sinf(capture.pitch/2) * sinf(capture.yaw/2);
    camera_capture.q[2] = cosf(capture.roll/2) * sinf(capture.pitch/2) * cosf(capture.yaw/2) +
                         sinf(capture.roll/2) * cosf(capture.pitch/2) * sinf(capture.yaw/2);
    camera_capture.q[3] = cosf(capture.roll/2) * cosf(capture.pitch/2) * sinf(capture.yaw/2) -
                         sinf(capture.roll/2) * sinf(capture.pitch/2) * cosf(capture.yaw/2);
    camera_capture.result = 1;  // Success

    _camera_capture_pub.publish(camera_capture);
}

void CameraFeedback::log_capture_to_file(const CaptureData &capture)
{
    // Write to camera log file in standard format (CSV, KML, etc.)
    static FILE *log_file = nullptr;

    if (log_file == nullptr) {
        log_file = fopen(CAMERA_LOG_FILE_PATH, "w");
        if (log_file != nullptr) {
            // Write CSV header
            fprintf(log_file, "Sequence,Timestamp,Latitude,Longitude,Altitude_MSL,Altitude_REL,Roll,Pitch,Yaw\n");
        }
    }

    if (log_file != nullptr) {
        fprintf(log_file, "%u,%llu,%.9f,%.9f,%.3f,%.3f,%.6f,%.6f,%.6f\n",
               capture.seq, capture.timestamp,
               capture.lat, capture.lon,
               capture.alt_msl, capture.alt_rel,
               capture.roll, capture.pitch, capture.yaw);

        fflush(log_file);  // Ensure data is written
    }
}
```

---

## **Communication Module Integration Summary**

These communication and interface modules handle all external interactions:

1. **Manual Control** ← Processes pilot RC input, validates signals, manages failsafe
2. **RC Update** ← Low-level RC protocol handling, calibration, channel mapping
3. **Camera Feedback** ← Camera integration, geotagging, photo logging, mission coordination

---

## **Navigation and Control Support Modules - Detailed**

### **1. Land Detector Module**

**Location**: `src/modules/land_detector/`
**Main Class**: `LandDetector`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
The **Land Detector** module automatically detects when the vehicle has landed or taken off, providing critical state information for flight mode transitions, motor disarming, and safety systems.

```cpp
namespace land_detector
{
class LandDetector : public ModuleBase<LandDetector>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Vehicle-specific land detection
    LandDetectorInterface *_land_detector{nullptr};

    // Publications
    uORB::Publication<vehicle_land_detected_s> _vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};

    // State tracking
    bool _landed{true};
    bool _freefall{false};
    bool _ground_contact{false};
    bool _maybe_landed{false};

    // Timing
    hrt_abstime _land_detection_start{0};
    hrt_abstime _takeoff_time{0};
};
}
```

#### **Multi-Vehicle Land Detection Architecture**
```mermaid
flowchart TB
    subgraph "Vehicle Types"
        MC[Multicopter]
        FW[Fixed Wing]
        VTOL[VTOL]
        ROVER[Rover]
    end

    subgraph "Detection Algorithms"
        MC_DETECT[Multicopter Detector]
        FW_DETECT[Fixed Wing Detector]
        VTOL_DETECT[VTOL Detector]
        ROVER_DETECT[Rover Detector]
    end

    subgraph "Sensor Inputs"
        ACCEL[Accelerometer]
        GYRO[Gyroscope]
        BARO[Barometer]
        GPS[GPS]
        THRUST[Thrust/Throttle]
        AIRSPEED[Airspeed]
    end

    subgraph "Detection States"
        LANDED[Landed]
        AIRBORNE[Airborne]
        GROUND_CONTACT[Ground Contact]
        FREEFALL[Freefall]
        MAYBE_LANDED[Maybe Landed]
    end

    MC --> MC_DETECT
    FW --> FW_DETECT
    VTOL --> VTOL_DETECT
    ROVER --> ROVER_DETECT

    ACCEL --> MC_DETECT
    GYRO --> MC_DETECT
    BARO --> MC_DETECT
    GPS --> MC_DETECT
    THRUST --> MC_DETECT
    AIRSPEED --> FW_DETECT

    MC_DETECT --> LANDED
    MC_DETECT --> AIRBORNE
    MC_DETECT --> GROUND_CONTACT
    MC_DETECT --> FREEFALL
    MC_DETECT --> MAYBE_LANDED
```

#### **Multicopter Land Detection Algorithm**
```cpp
class MulticopterLandDetector : public LandDetectorInterface
{
private:
    // Detection parameters
    float _param_land_speed{0.5f};          // Max land speed [m/s]
    float _param_land_acc_threshold{2.0f};   // Acceleration threshold [m/s²]
    float _param_land_thrust_threshold{0.3f}; // Thrust threshold (normalized)
    float _param_land_time{1.0f};           // Min time to confirm landing [s]

    // State hysteresis
    systemlib::Hysteresis _landed_hysteresis{false};
    systemlib::Hysteresis _ground_contact_hysteresis{false};
    systemlib::Hysteresis _freefall_hysteresis{false};

public:
    bool update() override
    {
        // Get current sensor data
        vehicle_local_position_s local_pos;
        vehicle_attitude_s attitude;
        actuator_controls_s actuator_controls;
        vehicle_acceleration_s acceleration;

        _local_position_sub.copy(&local_pos);
        _vehicle_attitude_sub.copy(&attitude);
        _actuator_controls_sub.copy(&actuator_controls);
        _vehicle_acceleration_sub.copy(&acceleration);

        // 1. CHECK BASIC LANDED CONDITIONS
        bool basic_conditions = check_basic_landed_conditions(local_pos, actuator_controls);

        // 2. CHECK ACCELERATION SIGNATURE
        bool low_acceleration = check_acceleration_signature(acceleration);

        // 3. CHECK ANGULAR RATES
        bool low_angular_rates = check_angular_rates(attitude);

        // 4. CHECK VERTICAL MOVEMENT
        bool minimal_vertical_movement = check_vertical_movement(local_pos);

        // 5. COMBINE CONDITIONS WITH HYSTERESIS
        bool landed_candidate = basic_conditions && low_acceleration &&
                               low_angular_rates && minimal_vertical_movement;

        _landed_hysteresis.set_state_and_update(landed_candidate, hrt_absolute_time());

        // 6. CHECK FOR FREEFALL
        bool freefall_detected = detect_freefall(acceleration);
        _freefall_hysteresis.set_state_and_update(freefall_detected, hrt_absolute_time());

        // 7. CHECK FOR GROUND CONTACT
        bool ground_contact = detect_ground_contact(acceleration, actuator_controls);
        _ground_contact_hysteresis.set_state_and_update(ground_contact, hrt_absolute_time());

        // Update detection states
        _landed = _landed_hysteresis.get_state();
        _freefall = _freefall_hysteresis.get_state();
        _ground_contact = _ground_contact_hysteresis.get_state();

        return true;
    }

private:
    bool check_basic_landed_conditions(const vehicle_local_position_s &local_pos,
                                      const actuator_controls_s &actuator_controls)
    {
        // 1. LOW THRUST CHECK
        float thrust_z = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
        bool low_thrust = thrust_z < _param_land_thrust_threshold;

        // 2. LOW HORIZONTAL SPEED
        bool low_horizontal_speed = true;
        if (local_pos.v_xy_valid) {
            float horizontal_speed = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
            low_horizontal_speed = horizontal_speed < _param_land_speed;
        }

        // 3. LOW VERTICAL SPEED
        bool low_vertical_speed = true;
        if (local_pos.v_z_valid) {
            low_vertical_speed = fabsf(local_pos.vz) < _param_land_speed;
        }

        return low_thrust && low_horizontal_speed && low_vertical_speed;
    }

    bool check_acceleration_signature(const vehicle_acceleration_s &acceleration)
    {
        // Check for characteristic "landed" acceleration signature
        Vector3f accel_body(acceleration.xyz[0], acceleration.xyz[1], acceleration.xyz[2]);

        // When landed, acceleration should be close to gravity in body frame
        float accel_magnitude = accel_body.norm();
        float gravity_magnitude = CONSTANTS_ONE_G;

        // Check if acceleration magnitude is close to 1G (within threshold)
        bool near_gravity = fabsf(accel_magnitude - gravity_magnitude) < _param_land_acc_threshold;

        // Check for low acceleration variance (stable)
        static Vector3f accel_history[ACCEL_HISTORY_SIZE];
        static int history_index = 0;

        accel_history[history_index] = accel_body;
        history_index = (history_index + 1) % ACCEL_HISTORY_SIZE;

        // Calculate variance
        Vector3f mean_accel(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < ACCEL_HISTORY_SIZE; i++) {
            mean_accel += accel_history[i];
        }
        mean_accel /= (float)ACCEL_HISTORY_SIZE;

        float variance = 0.0f;
        for (int i = 0; i < ACCEL_HISTORY_SIZE; i++) {
            Vector3f diff = accel_history[i] - mean_accel;
            variance += diff.norm_squared();
        }
        variance /= (float)ACCEL_HISTORY_SIZE;

        bool low_variance = variance < ACCEL_VARIANCE_THRESHOLD;

        return near_gravity && low_variance;
    }

    bool check_angular_rates(const vehicle_attitude_s &attitude)
    {
        // Check for low angular rates (vehicle not rotating)
        Vector3f angular_velocity(attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed);
        float angular_rate_magnitude = angular_velocity.norm();

        return angular_rate_magnitude < ANGULAR_RATE_THRESHOLD;
    }

    bool check_vertical_movement(const vehicle_local_position_s &local_pos)
    {
        // Track altitude changes over time
        static float altitude_history[ALTITUDE_HISTORY_SIZE];
        static int altitude_index = 0;
        static bool history_initialized = false;

        if (!local_pos.z_valid) {
            return true;  // Assume no movement if no altitude data
        }

        float current_altitude = -local_pos.z;  // Convert NED to altitude

        if (!history_initialized) {
            // Initialize history
            for (int i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
                altitude_history[i] = current_altitude;
            }
            history_initialized = true;
            return true;
        }

        altitude_history[altitude_index] = current_altitude;
        altitude_index = (altitude_index + 1) % ALTITUDE_HISTORY_SIZE;

        // Calculate altitude variance
        float mean_altitude = 0.0f;
        for (int i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
            mean_altitude += altitude_history[i];
        }
        mean_altitude /= (float)ALTITUDE_HISTORY_SIZE;

        float altitude_variance = 0.0f;
        for (int i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
            float diff = altitude_history[i] - mean_altitude;
            altitude_variance += diff * diff;
        }
        altitude_variance /= (float)ALTITUDE_HISTORY_SIZE;

        return sqrtf(altitude_variance) < ALTITUDE_VARIANCE_THRESHOLD;
    }

    bool detect_freefall(const vehicle_acceleration_s &acceleration)
    {
        // Detect freefall condition (near-zero acceleration)
        Vector3f accel_body(acceleration.xyz[0], acceleration.xyz[1], acceleration.xyz[2]);
        float accel_magnitude = accel_body.norm();

        // Freefall: acceleration magnitude significantly less than gravity
        return accel_magnitude < FREEFALL_THRESHOLD;
    }

    bool detect_ground_contact(const vehicle_acceleration_s &acceleration,
                              const actuator_controls_s &actuator_controls)
    {
        // Detect ground contact through impact signature
        Vector3f accel_body(acceleration.xyz[0], acceleration.xyz[1], acceleration.xyz[2]);
        float vertical_accel = accel_body(2);  // Z-axis in body frame

        // Ground contact: high vertical acceleration with low thrust
        float thrust_z = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];

        bool high_vertical_accel = vertical_accel > GROUND_CONTACT_ACCEL_THRESHOLD;
        bool low_thrust = thrust_z < GROUND_CONTACT_THRUST_THRESHOLD;

        return high_vertical_accel && low_thrust;
    }
};
```

#### **Land Detection State Machine**
```cpp
// Land detection state transitions
void LandDetector::update_state()
{
    vehicle_land_detected_s land_detected{};
    land_detected.timestamp = hrt_absolute_time();

    // Update from vehicle-specific detector
    if (_land_detector) {
        _land_detector->update();

        // Get detection results
        bool landed = _land_detector->landed();
        bool freefall = _land_detector->freefall();
        bool ground_contact = _land_detector->ground_contact();
        bool maybe_landed = _land_detector->maybe_landed();

        // State transition logic
        if (!_landed && landed) {
            // AIRBORNE → LANDED transition
            _land_detection_start = hrt_absolute_time();
            _landed = true;

            events::send(events::ID("land_detected"), events::Log::Info,
                        "Landing detected");

        } else if (_landed && !landed && !maybe_landed) {
            // LANDED → AIRBORNE transition
            _takeoff_time = hrt_absolute_time();
            _landed = false;

            events::send(events::ID("takeoff_detected"), events::Log::Info,
                        "Takeoff detected");
        }

        // Update all states
        land_detected.landed = landed;
        land_detected.freefall = freefall;
        land_detected.ground_contact = ground_contact;
        land_detected.maybe_landed = maybe_landed;

        // Time since landing/takeoff
        if (landed) {
            land_detected.ground_contact_time = hrt_elapsed_time(&_land_detection_start);
        } else {
            land_detected.flight_time = hrt_elapsed_time(&_takeoff_time);
        }
    }

    _vehicle_land_detected_pub.publish(land_detected);
}
```

---

### **2. Flight Mode Manager Module**

**Location**: `src/modules/flight_mode_manager/`
**Main Class**: `FlightModeManager`
**Work Queue**: `nav_and_controllers`

#### **Purpose and Functionality**
The **Flight Mode Manager** coordinates flight mode transitions, validates mode changes, and manages the overall flight state machine for safe and predictable vehicle behavior.

```cpp
class FlightModeManager : public ModuleBase<FlightModeManager>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Flight mode states
    enum class FlightMode {
        MANUAL = 0,
        STABILIZED,
        ACRO,
        ALTITUDE_CONTROL,
        POSITION_CONTROL,
        AUTO_LOITER,
        AUTO_RTL,
        AUTO_MISSION,
        AUTO_LAND,
        AUTO_TAKEOFF,
        OFFBOARD
    } _current_mode{FlightMode::MANUAL};

    // Mode transition validation
    struct ModeTransition {
        FlightMode from;
        FlightMode to;
        bool (*validator)(void);
        const char *description;
    };

    // Publications
    uORB::Publication<vehicle_control_mode_s> _control_mode_pub{ORB_ID(vehicle_control_mode)};
    uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};

    // State tracking
    bool _armed{false};
    bool _landed{true};
    hrt_abstime _mode_switch_time{0};
};
```

#### **Flight Mode State Machine**
```mermaid
stateDiagram-v2
    [*] --> MANUAL

    MANUAL --> STABILIZED : Pilot switch
    MANUAL --> ACRO : Pilot switch

    STABILIZED --> MANUAL : Pilot switch
    STABILIZED --> ALTITUDE : Pilot switch
    STABILIZED --> ACRO : Pilot switch

    ACRO --> MANUAL : Pilot switch
    ACRO --> STABILIZED : Pilot switch

    ALTITUDE --> STABILIZED : Pilot switch
    ALTITUDE --> POSITION : GPS valid

    POSITION --> ALTITUDE : GPS lost
    POSITION --> AUTO_LOITER : Auto mode

    AUTO_LOITER --> AUTO_MISSION : Mission active
    AUTO_LOITER --> AUTO_RTL : RTL command
    AUTO_LOITER --> POSITION : Manual mode

    AUTO_MISSION --> AUTO_RTL : Mission complete
    AUTO_MISSION --> AUTO_LOITER : Mission pause

    AUTO_RTL --> AUTO_LAND : At home position
    AUTO_RTL --> AUTO_LOITER : RTL abort

    AUTO_LAND --> MANUAL : Landing complete

    note right of POSITION : Requires GPS lock
    note right of AUTO_MISSION : Requires valid mission
    note right of AUTO_RTL : Return to launch
```

#### **Mode Transition Validation System**
```cpp
void FlightModeManager::Run()
{
    // 1. READ INPUTS
    vehicle_status_s vehicle_status;
    manual_control_switches_s manual_switches;
    vehicle_local_position_s local_position;
    mission_s mission;

    _vehicle_status_sub.copy(&vehicle_status);
    _manual_switches_sub.copy(&manual_switches);
    _local_position_sub.copy(&local_position);
    _mission_sub.copy(&mission);

    // 2. DETERMINE REQUESTED MODE
    FlightMode requested_mode = determine_requested_mode(manual_switches, vehicle_status);

    // 3. VALIDATE MODE TRANSITION
    if (requested_mode != _current_mode) {
        if (validate_mode_transition(_current_mode, requested_mode,
                                   vehicle_status, local_position, mission)) {

            // 4. EXECUTE MODE TRANSITION
            execute_mode_transition(requested_mode);

        } else {
            // 5. REJECT INVALID TRANSITION
            reject_mode_transition(requested_mode);
        }
    }

    // 6. UPDATE CONTROL MODE
    update_control_mode();

    // 7. PUBLISH STATE
    publish_vehicle_status();
}

FlightModeManager::FlightMode FlightModeManager::determine_requested_mode(
    const manual_control_switches_s &switches,
    const vehicle_status_s &vehicle_status)
{
    // Handle manual mode selection from RC switches
    if (!vehicle_status.rc_signal_lost) {

        switch (switches.mode_slot) {
            case manual_control_switches_s::MODE_SLOT_1:
                // Manual modes (parameter configurable)
                switch (_param_mode_slot_1.get()) {
                    case MODE_MANUAL: return FlightMode::MANUAL;
                    case MODE_STABILIZED: return FlightMode::STABILIZED;
                    case MODE_ACRO: return FlightMode::ACRO;
                }
                break;

            case manual_control_switches_s::MODE_SLOT_2:
                // Assisted modes
                switch (_param_mode_slot_2.get()) {
                    case MODE_ALTITUDE: return FlightMode::ALTITUDE_CONTROL;
                    case MODE_POSITION: return FlightMode::POSITION_CONTROL;
                }
                break;

            case manual_control_switches_s::MODE_SLOT_3:
                // Auto modes
                if (switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
                    return FlightMode::AUTO_RTL;
                } else {
                    switch (_param_mode_slot_3.get()) {
                        case MODE_AUTO_LOITER: return FlightMode::AUTO_LOITER;
                        case MODE_AUTO_MISSION: return FlightMode::AUTO_MISSION;
                    }
                }
                break;
        }
    }

    // Handle autonomous mode requests
    if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
        return FlightMode::AUTO_MISSION;
    }

    if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
        return FlightMode::AUTO_RTL;
    }

    if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
        return FlightMode::AUTO_LAND;
    }

    if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
        return FlightMode::OFFBOARD;
    }

    // Default fallback
    return FlightMode::MANUAL;
}

bool FlightModeManager::validate_mode_transition(FlightMode from, FlightMode to,
                                                const vehicle_status_s &vehicle_status,
                                                const vehicle_local_position_s &local_pos,
                                                const mission_s &mission)
{
    // 1. BASIC SAFETY CHECKS

    // Cannot switch to certain modes while disarmed
    if (!vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        if (to == FlightMode::AUTO_TAKEOFF || to == FlightMode::AUTO_MISSION ||
            to == FlightMode::AUTO_RTL || to == FlightMode::OFFBOARD) {
            PX4_WARN("Cannot switch to auto/offboard mode while disarmed");
            return false;
        }
    }

    // 2. GPS-DEPENDENT MODES
    if (to == FlightMode::POSITION_CONTROL || to == FlightMode::AUTO_LOITER ||
        to == FlightMode::AUTO_MISSION || to == FlightMode::AUTO_RTL) {

        if (!local_pos.xy_valid || !local_pos.z_valid) {
            PX4_WARN("GPS required for position-based modes");
            return false;
        }

        // Check GPS accuracy
        if (local_pos.eph > GPS_EPH_THRESHOLD || local_pos.epv > GPS_EPV_THRESHOLD) {
            PX4_WARN("GPS accuracy insufficient for position mode");
            return false;
        }
    }

    // 3. MISSION-SPECIFIC CHECKS
    if (to == FlightMode::AUTO_MISSION) {
        if (!mission.valid || mission.count == 0) {
            PX4_WARN("No valid mission loaded");
            return false;
        }

        // Check if mission is executable
        if (mission.current_seq >= mission.count) {
            PX4_WARN("Mission sequence out of bounds");
            return false;
        }
    }

    // 4. OFFBOARD MODE CHECKS
    if (to == FlightMode::OFFBOARD) {
        offboard_control_mode_s offboard_mode;
        if (_offboard_control_mode_sub.copy(&offboard_mode)) {

            hrt_abstime offboard_timeout = hrt_absolute_time() - offboard_mode.timestamp;
            if (offboard_timeout > OFFBOARD_TIMEOUT_US) {
                PX4_WARN("Offboard control signal timeout");
                return false;
            }

            // Check if valid control modes are set
            if (!offboard_mode.position && !offboard_mode.velocity &&
                !offboard_mode.acceleration && !offboard_mode.attitude) {
                PX4_WARN("No valid offboard control mode set");
                return false;
            }

        } else {
            PX4_WARN("No offboard control mode available");
            return false;
        }
    }

    // 5. TRANSITION-SPECIFIC LOGIC
    switch (from) {
        case FlightMode::AUTO_LAND:
            // Cannot exit auto land until landed
            if (!_landed && to != FlightMode::AUTO_LAND) {
                PX4_WARN("Cannot exit auto land mode while airborne");
                return false;
            }
            break;

        case FlightMode::AUTO_RTL:
            // Allow emergency exit from RTL
            if (to == FlightMode::MANUAL || to == FlightMode::STABILIZED) {
                PX4_WARN("Emergency exit from RTL mode");
                return true;
            }
            break;
    }

    return true;  // Transition approved
}

void FlightModeManager::execute_mode_transition(FlightMode new_mode)
{
    FlightMode previous_mode = _current_mode;
    _current_mode = new_mode;
    _mode_switch_time = hrt_absolute_time();

    // Mode-specific initialization
    switch (new_mode) {
        case FlightMode::AUTO_TAKEOFF:
            initialize_auto_takeoff();
            break;

        case FlightMode::AUTO_MISSION:
            initialize_auto_mission();
            break;

        case FlightMode::AUTO_RTL:
            initialize_auto_rtl();
            break;

        case FlightMode::AUTO_LAND:
            initialize_auto_land();
            break;

        case FlightMode::OFFBOARD:
            initialize_offboard_mode();
            break;
    }

    // Log mode transition
    events::send<uint8_t, uint8_t>(events::ID("flight_mode_changed"),
                                  events::Log::Info,
                                  "Flight mode changed: {1} -> {2}",
                                  (uint8_t)previous_mode, (uint8_t)new_mode);

    PX4_INFO("Flight mode transition: %s -> %s",
             mode_name(previous_mode), mode_name(new_mode));
}
```

#### **Control Mode Generation**
```cpp
void FlightModeManager::update_control_mode()
{
    vehicle_control_mode_s control_mode{};
    control_mode.timestamp = hrt_absolute_time();

    // Reset all flags
    control_mode.flag_armed = _armed;
    control_mode.flag_multicopter_position_control_enabled = false;
    control_mode.flag_control_manual_enabled = false;
    control_mode.flag_control_auto_enabled = false;
    control_mode.flag_control_offboard_enabled = false;
    control_mode.flag_control_rates_enabled = false;
    control_mode.flag_control_attitude_enabled = false;
    control_mode.flag_control_velocity_enabled = false;
    control_mode.flag_control_position_enabled = false;
    control_mode.flag_control_altitude_enabled = false;
    control_mode.flag_control_climb_rate_enabled = false;
    control_mode.flag_control_termination_enabled = false;

    // Set mode-specific control flags
    switch (_current_mode) {
        case FlightMode::MANUAL:
            control_mode.flag_control_manual_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            break;

        case FlightMode::STABILIZED:
            control_mode.flag_control_manual_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            control_mode.flag_control_attitude_enabled = true;
            break;

        case FlightMode::ACRO:
            control_mode.flag_control_manual_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            break;

        case FlightMode::ALTITUDE_CONTROL:
            control_mode.flag_control_manual_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            control_mode.flag_control_attitude_enabled = true;
            control_mode.flag_control_altitude_enabled = true;
            control_mode.flag_control_climb_rate_enabled = true;
            break;

        case FlightMode::POSITION_CONTROL:
            control_mode.flag_control_manual_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            control_mode.flag_control_attitude_enabled = true;
            control_mode.flag_control_altitude_enabled = true;
            control_mode.flag_control_climb_rate_enabled = true;
            control_mode.flag_control_velocity_enabled = true;
            control_mode.flag_control_position_enabled = true;
            control_mode.flag_multicopter_position_control_enabled = true;
            break;

        case FlightMode::AUTO_LOITER:
        case FlightMode::AUTO_MISSION:
        case FlightMode::AUTO_RTL:
        case FlightMode::AUTO_LAND:
        case FlightMode::AUTO_TAKEOFF:
            control_mode.flag_control_auto_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            control_mode.flag_control_attitude_enabled = true;
            control_mode.flag_control_altitude_enabled = true;
            control_mode.flag_control_climb_rate_enabled = true;
            control_mode.flag_control_velocity_enabled = true;
            control_mode.flag_control_position_enabled = true;
            control_mode.flag_multicopter_position_control_enabled = true;
            break;

        case FlightMode::OFFBOARD:
            control_mode.flag_control_offboard_enabled = true;
            control_mode.flag_control_rates_enabled = true;
            control_mode.flag_control_attitude_enabled = true;
            control_mode.flag_control_altitude_enabled = true;
            control_mode.flag_control_climb_rate_enabled = true;
            control_mode.flag_control_velocity_enabled = true;
            control_mode.flag_control_position_enabled = true;
            control_mode.flag_multicopter_position_control_enabled = true;
            break;
    }

    _control_mode_pub.publish(control_mode);
}
```

---

### **3. Control Allocator Module**

**Location**: `src/modules/control_allocator/`
**Main Class**: `ControlAllocator`
**Work Queue**: `wq_rate_ctrl`

#### **Purpose and Functionality**
The **Control Allocator** module converts high-level control commands (torques and thrust) into individual actuator outputs (motor speeds, servo positions), handling motor mixing, actuator constraints, and failure compensation.

```cpp
class ControlAllocator : public ModuleBase<ControlAllocator>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Control allocation methods
    ControlAllocationMethod *_control_allocation{nullptr};

    // Actuator configuration
    ActuatorEffectiveness *_actuator_effectiveness{nullptr};

    // Subscriptions
    uORB::Subscription _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};
    uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};

    // Publications
    uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
    uORB::PublicationMulti<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

    // Mixing matrix and constraints
    matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _effectiveness;
    matrix::Vector<float, NUM_ACTUATORS> _actuator_min;
    matrix::Vector<float, NUM_ACTUATORS> _actuator_max;

    // Actuator testing
    bool _actuator_test_active{false};
    ActuatorTest _actuator_test;
};
```

#### **Control Allocation Architecture**
```mermaid
flowchart TB
    subgraph "Control Commands"
        TORQUE[Torque Setpoints]
        THRUST[Thrust Setpoints]
        TEST[Actuator Test]
    end

    subgraph "Control Allocator"
        EFFECT[Effectiveness Matrix]
        SOLVER[Allocation Solver]
        CONSTRAIN[Constraint Handling]
        FAILURE[Failure Compensation]
    end

    subgraph "Allocation Methods"
        PSEUDO[Pseudoinverse]
        SEQUENTIAL[Sequential Least Squares]
        CONSTRAINED[Constrained Optimization]
    end

    subgraph "Actuator Types"
        MOTORS[Motors/ESCs]
        SERVOS[Control Servos]
        TILTS[Tilt Servos]
        SPOILERS[Spoilers/Flaps]
    end

    subgraph "Hardware Outputs"
        PWM[PWM Signals]
        DSHOT[DShot Protocol]
        UAVCAN[UAVCAN/CAN]
        SERIAL[Serial Protocol]
    end

    TORQUE --> EFFECT
    THRUST --> EFFECT
    TEST --> EFFECT

    EFFECT --> SOLVER
    SOLVER --> CONSTRAIN
    CONSTRAIN --> FAILURE

    SOLVER --> PSEUDO
    SOLVER --> SEQUENTIAL
    SOLVER --> CONSTRAINED

    FAILURE --> MOTORS
    FAILURE --> SERVOS
    FAILURE --> TILTS
    FAILURE --> SPOILERS

    MOTORS --> PWM
    SERVOS --> PWM
    TILTS --> DSHOT
    SPOILERS --> UAVCAN
```

#### **Actuator Effectiveness Matrix**
```cpp
class ActuatorEffectiveness
{
public:
    // Effectiveness matrix: [actuators x control_axes]
    // Control axes: roll, pitch, yaw, thrust_x, thrust_y, thrust_z
    void update_effectiveness_matrix()
    {
        // Clear effectiveness matrix
        _effectiveness.setZero();

        // For each actuator, calculate its effectiveness
        for (int i = 0; i < _num_actuators; i++) {

            switch (_actuator_config[i].type) {
                case ActuatorType::MOTOR:
                    calculate_motor_effectiveness(i);
                    break;

                case ActuatorType::SERVO:
                    calculate_servo_effectiveness(i);
                    break;

                case ActuatorType::TILT_SERVO:
                    calculate_tilt_servo_effectiveness(i);
                    break;
            }
        }

        // Validate effectiveness matrix
        if (!validate_effectiveness_matrix()) {
            PX4_ERR("Invalid actuator effectiveness matrix");
        }
    }

private:
    void calculate_motor_effectiveness(int motor_index)
    {
        const ActuatorConfig &config = _actuator_config[motor_index];

        // Motor position relative to center of mass
        Vector3f position(config.position_x, config.position_y, config.position_z);

        // Motor thrust direction (normalized)
        Vector3f thrust_direction(config.thrust_x, config.thrust_y, config.thrust_z);
        thrust_direction.normalize();

        // Motor moment arm (position cross thrust direction)
        Vector3f moment_arm = position.cross(thrust_direction);

        // Effectiveness for torques (roll, pitch, yaw)
        _effectiveness(motor_index, 0) = moment_arm(0);  // Roll moment
        _effectiveness(motor_index, 1) = moment_arm(1);  // Pitch moment
        _effectiveness(motor_index, 2) = moment_arm(2);  // Yaw moment

        // Effectiveness for thrust (thrust_x, thrust_y, thrust_z)
        _effectiveness(motor_index, 3) = thrust_direction(0);  // Thrust X
        _effectiveness(motor_index, 4) = thrust_direction(1);  // Thrust Y
        _effectiveness(motor_index, 5) = thrust_direction(2);  // Thrust Z

        // Add motor-specific torque (propeller reaction torque)
        float motor_torque_coeff = config.motor_torque_coeff;
        _effectiveness(motor_index, 2) += motor_torque_coeff;  // Add to yaw
    }

    void calculate_servo_effectiveness(int servo_index)
    {
        const ActuatorConfig &config = _actuator_config[servo_index];

        // Control surface effectiveness depends on surface type and deflection
        switch (config.servo_function) {
            case ServoFunction::AILERON:
                _effectiveness(servo_index, 0) = config.effectiveness_roll;
                break;

            case ServoFunction::ELEVATOR:
                _effectiveness(servo_index, 1) = config.effectiveness_pitch;
                break;

            case ServoFunction::RUDDER:
                _effectiveness(servo_index, 2) = config.effectiveness_yaw;
                break;

            case ServoFunction::FLAP:
                _effectiveness(servo_index, 5) = config.effectiveness_thrust_z;
                break;
        }
    }
};
```

#### **Control Allocation Solver**
```cpp
class ControlAllocationMethod
{
public:
    virtual void allocate(const matrix::Vector<float, NUM_AXES> &control_sp,
                         matrix::Vector<float, NUM_ACTUATORS> &actuator_sp) = 0;
};

class PseudoinverseMethod : public ControlAllocationMethod
{
public:
    void allocate(const matrix::Vector<float, NUM_AXES> &control_sp,
                 matrix::Vector<float, NUM_ACTUATORS> &actuator_sp) override
    {
        // Simple pseudoinverse allocation
        // actuator_sp = pinv(effectiveness) * control_sp

        // Compute Moore-Penrose pseudoinverse
        matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> effectiveness_T = _effectiveness.transpose();
        matrix::Matrix<float, NUM_AXES, NUM_AXES> gram = _effectiveness * effectiveness_T;

        // Check if Gram matrix is invertible
        if (fabsf(gram.determinant()) < 1e-6f) {
            PX4_WARN("Effectiveness matrix near-singular");
            // Use damped least squares
            gram += matrix::eye<float, NUM_AXES>() * DAMPING_FACTOR;
        }

        matrix::Matrix<float, NUM_AXES, NUM_AXES> gram_inv;
        if (gram.inv(gram_inv)) {
            matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> pinv = effectiveness_T * gram_inv;
            actuator_sp = pinv * control_sp;
        } else {
            PX4_ERR("Failed to compute pseudoinverse");
            actuator_sp.setZero();
        }
    }
};

class SequentialLeastSquares : public ControlAllocationMethod
{
public:
    void allocate(const matrix::Vector<float, NUM_AXES> &control_sp,
                 matrix::Vector<float, NUM_ACTUATORS> &actuator_sp) override
    {
        // Sequential least squares with constraint handling
        actuator_sp.setZero();
        matrix::Vector<float, NUM_AXES> remaining_control = control_sp;

        // Priority-based allocation
        for (int priority = 0; priority < MAX_PRIORITY_LEVELS; priority++) {

            // Find actuators at this priority level
            for (int i = 0; i < _num_actuators; i++) {
                if (_actuator_priority[i] == priority) {

                    // Calculate required actuator output
                    float required_output = calculate_required_output(i, remaining_control);

                    // Apply constraints
                    float constrained_output = math::constrain(required_output,
                                                             _actuator_min(i),
                                                             _actuator_max(i));

                    actuator_sp(i) = constrained_output;

                    // Update remaining control requirements
                    for (int axis = 0; axis < NUM_AXES; axis++) {
                        remaining_control(axis) -= _effectiveness(i, axis) * constrained_output;
                    }
                }
            }
        }
    }

private:
    float calculate_required_output(int actuator_index,
                                   const matrix::Vector<float, NUM_AXES> &control_error)
    {
        // Find the control axis this actuator is most effective for
        int primary_axis = 0;
        float max_effectiveness = 0.0f;

        for (int axis = 0; axis < NUM_AXES; axis++) {
            if (fabsf(_effectiveness(actuator_index, axis)) > max_effectiveness) {
                max_effectiveness = fabsf(_effectiveness(actuator_index, axis));
                primary_axis = axis;
            }
        }

        // Calculate required output for primary axis
        if (max_effectiveness > 1e-6f) {
            return control_error(primary_axis) / _effectiveness(actuator_index, primary_axis);
        }

        return 0.0f;
    }
};
```

#### **Actuator Failure Handling**
```cpp
void ControlAllocator::handle_actuator_failure()
{
    actuator_controls_status_s actuator_status;
    if (_actuator_controls_status_sub.copy(&actuator_status)) {

        // Check for actuator failures
        for (int i = 0; i < _num_actuators; i++) {

            bool actuator_failed = false;

            // Check for various failure modes
            if (actuator_status.control_power[i] < ACTUATOR_POWER_THRESHOLD) {
                actuator_failed = true;
                PX4_WARN("Actuator %d power failure", i);
            }

            if (fabsf(actuator_status.control[i] - _last_actuator_output[i]) > ACTUATOR_STUCK_THRESHOLD) {
                actuator_failed = true;
                PX4_WARN("Actuator %d stuck/unresponsive", i);
            }

            if (actuator_failed && !_actuator_failed[i]) {
                // New failure detected
                _actuator_failed[i] = true;

                // Update effectiveness matrix (remove failed actuator)
                for (int axis = 0; axis < NUM_AXES; axis++) {
                    _effectiveness(i, axis) = 0.0f;
                }

                // Recalculate allocation with reduced actuator set
                _control_allocation->update_effectiveness(_effectiveness);

                // Notify other systems of actuator failure
                events::send<uint8_t>(events::ID("actuator_failure"),
                                     events::Log::Emergency,
                                     "Actuator {1} failure detected", i);

                // Check if vehicle is still controllable
                if (!check_vehicle_controllability()) {
                    // Trigger emergency landing
                    trigger_emergency_action();
                }
            }
        }
    }
}

bool ControlAllocator::check_vehicle_controllability()
{
    // Check if remaining actuators can provide minimum control authority

    // Calculate rank of effectiveness matrix
    matrix::Matrix<float, NUM_AXES, NUM_AXES> controllability_gram =
        _effectiveness * _effectiveness.transpose();

    // Check if we can still control all required axes
    float min_singular_value = compute_minimum_singular_value(controllability_gram);

    if (min_singular_value < CONTROLLABILITY_THRESHOLD) {
        PX4_ERR("Vehicle may not be controllable with remaining actuators");
        return false;
    }

    return true;
}

void ControlAllocator::run_actuator_test()
{
    if (_actuator_test_active) {

        // Get test command
        actuator_test_s test_cmd;
        if (_actuator_test_sub.copy(&test_cmd)) {

            switch (test_cmd.action) {
                case actuator_test_s::ACTION_MOTOR_TEST:
                    test_motor(test_cmd.motor_number, test_cmd.value);
                    break;

                case actuator_test_s::ACTION_SERVO_TEST:
                    test_servo(test_cmd.servo_number, test_cmd.value);
                    break;

                case actuator_test_s::ACTION_DO_MOTOR_TEST_SEQUENCE:
                    run_motor_test_sequence();
                    break;
            }
        }
    }
}

void ControlAllocator::test_motor(int motor_number, float test_value)
{
    if (motor_number < 0 || motor_number >= _num_motors) {
        return;
    }

    // Safety checks for motor testing
    vehicle_land_detected_s land_detected;
    if (_vehicle_land_detected_sub.copy(&land_detected) && !land_detected.landed) {
        PX4_ERR("Motor test not allowed while airborne");
        return;
    }

    // Set test value for specific motor
    actuator_motors_s motor_outputs{};
    motor_outputs.timestamp = hrt_absolute_time();

    for (int i = 0; i < _num_motors; i++) {
        if (i == motor_number) {
            motor_outputs.control[i] = math::constrain(test_value, 0.0f, 1.0f);
        } else {
            motor_outputs.control[i] = 0.0f;  // All other motors off
        }
    }

    _actuator_motors_pub.publish(motor_outputs);

    PX4_INFO("Testing motor %d at %.1f%%", motor_number, test_value * 100.0f);
}
```

---

## **Navigation and Control Support Summary**

These modules provide essential flight assistance and control distribution:

1. **Land Detector** ← Automatic landing/takeoff detection with multi-vehicle algorithms
2. **Flight Mode Manager** ← Safe mode transitions with comprehensive validation
3. **Control Allocator** ← Motor mixing, constraint handling, and failure compensation

---

## **Safety and Monitoring Modules - Detailed**

### **1. Load Monitor Module**

**Location**: `src/modules/load_mon/`
**Main Class**: `LoadMon`
**Work Queue**: `lp_default`

#### **Purpose and Functionality**
The **Load Monitor** module continuously tracks system resource usage including CPU load, memory consumption, stack usage, and task execution times to ensure system stability and detect performance bottlenecks.

```cpp
namespace load_mon
{
class LoadMon : public ModuleBase<LoadMon>, public ModuleParams, public px4::ScheduledWorkItem
{
    // System monitoring data
    struct TaskInfo {
        pid_t pid;
        char name[CONFIG_TASK_NAME_SIZE];
        uint64_t total_runtime;
        uint64_t curr_start_time;
        float curr_cpu_load;
        size_t stack_size;
        size_t stack_free;
        sched_priority_t priority;
        enum sched_policy_e policy;
    };

    // Resource tracking
    static constexpr int MAX_TASKS = 64;
    TaskInfo _tasks[MAX_TASKS];
    int _num_tasks{0};

    // System metrics
    uint64_t _last_idle_time{0};
    uint64_t _last_total_time{0};
    float _system_load{0.0f};
    uint32_t _free_memory{0};
    uint32_t _total_memory{0};

    // Publications
    uORB::Publication<cpuload_s> _cpuload_pub{ORB_ID(cpuload)};
    uORB::Publication<task_stack_info_s> _task_stack_info_pub{ORB_ID(task_stack_info)};

    // Monitoring parameters
    float _cpu_load_threshold{0.8f};
    uint32_t _memory_threshold{1024};  // KB
    uint32_t _stack_threshold{512};    // bytes
};
}
```

#### **System Resource Monitoring Architecture**
```mermaid
flowchart TB
    subgraph "System Resources"
        CPU[CPU Usage]
        MEM[Memory Usage]
        STACK[Stack Usage]
        IO[I/O Operations]
    end

    subgraph "Task Monitoring"
        SCHED[Scheduler Stats]
        RUNTIME[Runtime Tracking]
        PRIORITY[Priority Analysis]
        POLICY[Scheduling Policy]
    end

    subgraph "Load Monitor"
        COLLECT[Data Collection]
        ANALYZE[Performance Analysis]
        THRESHOLD[Threshold Checking]
        ALERT[Alert Generation]
    end

    subgraph "Monitoring Outputs"
        CPULOAD[CPU Load Report]
        STACKINFO[Stack Info Report]
        WARNINGS[Performance Warnings]
        EVENTS[System Events]
    end

    subgraph "Consumer Modules"
        COMMANDER[Commander]
        LOGGER[Logger]
        MAVLINK[MAVLink]
        FAILSAFE[Failsafe System]
    end

    CPU --> COLLECT
    MEM --> COLLECT
    STACK --> COLLECT
    IO --> COLLECT

    SCHED --> ANALYZE
    RUNTIME --> ANALYZE
    PRIORITY --> ANALYZE
    POLICY --> ANALYZE

    COLLECT --> ANALYZE
    ANALYZE --> THRESHOLD
    THRESHOLD --> ALERT

    ALERT --> CPULOAD
    ALERT --> STACKINFO
    ALERT --> WARNINGS
    ALERT --> EVENTS

    CPULOAD --> COMMANDER
    STACKINFO --> LOGGER
    WARNINGS --> MAVLINK
    EVENTS --> FAILSAFE
```

#### **CPU Load Monitoring Implementation**
```cpp
void LoadMon::Run()
{
    // 1. COLLECT SYSTEM STATISTICS
    collect_system_stats();

    // 2. ANALYZE TASK PERFORMANCE
    analyze_task_performance();

    // 3. CHECK MEMORY USAGE
    check_memory_usage();

    // 4. MONITOR STACK USAGE
    monitor_stack_usage();

    // 5. DETECT PERFORMANCE ISSUES
    detect_performance_issues();

    // 6. PUBLISH MONITORING DATA
    publish_monitoring_data();

    // 7. SCHEDULE NEXT MONITORING CYCLE
    ScheduleDelayed(LOAD_MON_INTERVAL_US);
}

void LoadMon::collect_system_stats()
{
    // Get current system time and idle time
    struct system_load_s system_load;

    // Read from /proc/loadavg equivalent or system calls
    uint64_t total_time = hrt_absolute_time();
    uint64_t idle_time = get_idle_time();

    if (_last_total_time > 0) {
        uint64_t total_delta = total_time - _last_total_time;
        uint64_t idle_delta = idle_time - _last_idle_time;

        if (total_delta > 0) {
            // Calculate system CPU load as percentage
            _system_load = 1.0f - ((float)idle_delta / (float)total_delta);
            _system_load = math::constrain(_system_load, 0.0f, 1.0f);
        }
    }

    _last_total_time = total_time;
    _last_idle_time = idle_time;

    // Get memory information
    struct mallinfo mem_info = mallinfo();
    _free_memory = mem_info.fordblks;  // Free memory
    _total_memory = mem_info.arena;    // Total heap memory
}

void LoadMon::analyze_task_performance()
{
    // Clear previous task info
    _num_tasks = 0;

    // Iterate through all running tasks
    DIR *proc_dir = opendir("/proc");
    if (proc_dir == nullptr) {
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(proc_dir)) != nullptr && _num_tasks < MAX_TASKS) {

        // Check if directory name is numeric (PID)
        char *endptr;
        pid_t pid = strtol(entry->d_name, &endptr, 10);

        if (*endptr == '\0' && pid > 0) {
            // Read task information
            if (read_task_info(pid, _tasks[_num_tasks])) {
                _num_tasks++;
            }
        }
    }

    closedir(proc_dir);

    // Sort tasks by CPU usage for analysis
    sort_tasks_by_cpu_load();

    // Calculate per-task CPU percentages
    calculate_task_cpu_percentages();
}

bool LoadMon::read_task_info(pid_t pid, TaskInfo &task_info)
{
    char stat_path[64];
    snprintf(stat_path, sizeof(stat_path), "/proc/%d/stat", pid);

    FILE *stat_file = fopen(stat_path, "r");
    if (stat_file == nullptr) {
        return false;
    }

    // Parse /proc/[pid]/stat
    char comm[256];
    unsigned long utime, stime;
    int priority, nice;

    int ret = fscanf(stat_file, "%d %s %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %lu %lu %*d %*d %d %d",
                     &task_info.pid, comm, &utime, &stime, &priority, &nice);

    fclose(stat_file);

    if (ret != 6) {
        return false;
    }

    // Store task information
    strncpy(task_info.name, comm, CONFIG_TASK_NAME_SIZE - 1);
    task_info.name[CONFIG_TASK_NAME_SIZE - 1] = '\0';

    task_info.total_runtime = (utime + stime) * (1000000 / sysconf(_SC_CLK_TCK)); // Convert to microseconds
    task_info.priority = priority;

    // Get stack information
    read_task_stack_info(pid, task_info);

    return true;
}

void LoadMon::read_task_stack_info(pid_t pid, TaskInfo &task_info)
{
    // Read stack information from task control block
    struct tcb_s *tcb = nxsched_get_tcb(pid);

    if (tcb != nullptr) {
        task_info.stack_size = tcb->adj_stack_size;

        // Calculate stack usage
        if (tcb->stack_alloc_ptr != nullptr) {

            // Find stack pointer bounds
            uintptr_t stack_base = (uintptr_t)tcb->stack_alloc_ptr;
            uintptr_t stack_top = stack_base + tcb->adj_stack_size;
            uintptr_t current_sp = (uintptr_t)tcb->xcp.regs[REG_SP];

            // Calculate used and free stack
            size_t stack_used = stack_top - current_sp;
            task_info.stack_free = tcb->adj_stack_size - stack_used;

            // Check for stack overflow detection pattern
            uint32_t *stack_base_ptr = (uint32_t *)stack_base;
            bool stack_overflow = false;

            for (int i = 0; i < STACK_OVERFLOW_CHECK_SIZE / sizeof(uint32_t); i++) {
                if (stack_base_ptr[i] != STACK_OVERFLOW_MAGIC) {
                    stack_overflow = true;
                    break;
                }
            }

            if (stack_overflow) {
                PX4_ERR("Stack overflow detected in task %s (PID: %d)", task_info.name, pid);

                // Generate critical event
                events::send<pid_t>(events::ID("stack_overflow"),
                                   events::Log::Emergency,
                                   "Stack overflow in task PID {1}", pid);
            }
        }

        // Get scheduling policy
        task_info.policy = tcb->sched_policy;
    }
}

void LoadMon::calculate_task_cpu_percentages()
{
    static uint64_t last_task_runtime[MAX_TASKS] = {0};
    static hrt_abstime last_measurement_time = 0;

    hrt_abstime current_time = hrt_absolute_time();
    hrt_abstime time_delta = current_time - last_measurement_time;

    if (last_measurement_time > 0 && time_delta > 0) {

        for (int i = 0; i < _num_tasks; i++) {

            // Find previous runtime for this task
            uint64_t previous_runtime = 0;
            for (int j = 0; j < MAX_TASKS; j++) {
                if (last_task_runtime[j] > 0) {  // Simple matching by runtime
                    previous_runtime = last_task_runtime[j];
                    break;
                }
            }

            // Calculate CPU percentage for this measurement period
            uint64_t runtime_delta = _tasks[i].total_runtime - previous_runtime;
            _tasks[i].curr_cpu_load = (float)runtime_delta / (float)time_delta;

            // Constrain to reasonable values
            _tasks[i].curr_cpu_load = math::constrain(_tasks[i].curr_cpu_load, 0.0f, 1.0f);
        }
    }

    // Store current runtimes for next calculation
    for (int i = 0; i < _num_tasks; i++) {
        if (i < MAX_TASKS) {
            last_task_runtime[i] = _tasks[i].total_runtime;
        }
    }

    last_measurement_time = current_time;
}

void LoadMon::detect_performance_issues()
{
    // 1. CHECK SYSTEM CPU LOAD
    if (_system_load > _cpu_load_threshold) {
        PX4_WARN("High system CPU load: %.1f%%", _system_load * 100.0f);

        events::send<uint16_t>(events::ID("high_cpu_load"),
                              events::Log::Warning,
                              "High CPU load: {1}%", (uint16_t)(_system_load * 100.0f));
    }

    // 2. CHECK MEMORY USAGE
    if (_free_memory < _memory_threshold * 1024) {
        PX4_WARN("Low free memory: %u KB", _free_memory / 1024);

        events::send<uint32_t>(events::ID("low_memory"),
                              events::Log::Warning,
                              "Low free memory: {1} KB", _free_memory / 1024);
    }

    // 3. CHECK INDIVIDUAL TASK PERFORMANCE
    for (int i = 0; i < _num_tasks; i++) {

        // Check for high CPU usage by individual tasks
        if (_tasks[i].curr_cpu_load > TASK_CPU_THRESHOLD) {
            PX4_WARN("Task %s using %.1f%% CPU", _tasks[i].name,
                     _tasks[i].curr_cpu_load * 100.0f);
        }

        // Check for low stack remaining
        if (_tasks[i].stack_free < _stack_threshold) {
            PX4_WARN("Task %s low stack: %zu bytes free", _tasks[i].name,
                     _tasks[i].stack_free);

            events::send<uint16_t>(events::ID("low_stack"),
                                  events::Log::Warning,
                                  "Task low stack: {1} bytes", (uint16_t)_tasks[i].stack_free);
        }
    }

    // 4. CHECK FOR SCHEDULING ANOMALIES
    detect_scheduling_anomalies();
}

void LoadMon::detect_scheduling_anomalies()
{
    // Look for tasks that should be high priority but are getting low CPU time
    for (int i = 0; i < _num_tasks; i++) {

        if (_tasks[i].priority < SCHED_PRIORITY_MAX / 2) {  // High priority task

            if (_tasks[i].curr_cpu_load < MIN_HIGH_PRIORITY_CPU_TIME) {
                PX4_WARN("High priority task %s getting insufficient CPU time", _tasks[i].name);

                events::send(events::ID("scheduling_anomaly"),
                            events::Log::Warning,
                            "High priority task starved");
            }
        }
    }

    // Check for priority inversion symptoms
    check_priority_inversion();
}

void LoadMon::check_priority_inversion()
{
    // Simple heuristic: if low priority tasks are using more CPU than high priority tasks
    float high_priority_total = 0.0f;
    float low_priority_total = 0.0f;
    int high_priority_count = 0;
    int low_priority_count = 0;

    for (int i = 0; i < _num_tasks; i++) {
        if (_tasks[i].priority < SCHED_PRIORITY_MAX / 2) {
            high_priority_total += _tasks[i].curr_cpu_load;
            high_priority_count++;
        } else {
            low_priority_total += _tasks[i].curr_cpu_load;
            low_priority_count++;
        }
    }

    if (high_priority_count > 0 && low_priority_count > 0) {
        float high_priority_avg = high_priority_total / high_priority_count;
        float low_priority_avg = low_priority_total / low_priority_count;

        if (low_priority_avg > high_priority_avg * PRIORITY_INVERSION_RATIO) {
            PX4_WARN("Possible priority inversion detected");

            events::send(events::ID("priority_inversion"),
                        events::Log::Warning,
                        "Priority inversion detected");
        }
    }
}
```

#### **Performance Data Publication**
```cpp
void LoadMon::publish_monitoring_data()
{
    // 1. PUBLISH CPU LOAD DATA
    cpuload_s cpuload{};
    cpuload.timestamp = hrt_absolute_time();
    cpuload.load = _system_load;
    cpuload.ram_usage = 1.0f - ((float)_free_memory / (float)_total_memory);

    // Add top CPU-consuming tasks
    cpuload.task_count = math::min(_num_tasks, (int)cpuload_s::MAX_TASKS);

    for (int i = 0; i < cpuload.task_count; i++) {
        strncpy(cpuload.tasks[i].name, _tasks[i].name, sizeof(cpuload.tasks[i].name) - 1);
        cpuload.tasks[i].name[sizeof(cpuload.tasks[i].name) - 1] = '\0';
        cpuload.tasks[i].load = _tasks[i].curr_cpu_load;
        cpuload.tasks[i].stack_free = _tasks[i].stack_free;
    }

    _cpuload_pub.publish(cpuload);

    // 2. PUBLISH STACK INFORMATION
    for (int i = 0; i < _num_tasks; i++) {
        task_stack_info_s stack_info{};
        stack_info.timestamp = hrt_absolute_time();
        stack_info.task_id = _tasks[i].pid;
        strncpy(stack_info.task_name, _tasks[i].name, sizeof(stack_info.task_name) - 1);
        stack_info.task_name[sizeof(stack_info.task_name) - 1] = '\0';
        stack_info.stack_size = _tasks[i].stack_size;
        stack_info.stack_free = _tasks[i].stack_free;

        _task_stack_info_pub.publish(stack_info);
    }
}
```

---

### **2. Events Module**

**Location**: `src/modules/events/`
**Main Class**: `Events`
**Work Queue**: `lp_default`

#### **Purpose and Functionality**
The **Events** module provides a centralized event management system for logging, categorizing, and distributing system events, warnings, and errors throughout the PX4 system.

```cpp
namespace events
{
class Events : public ModuleBase<Events>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Event storage and management
    static constexpr int MAX_EVENTS = 100;

    struct EventRecord {
        uint32_t id;
        uint8_t log_level;
        hrt_abstime timestamp;
        uint16_t sequence;
        char message[EVENT_MAX_MESSAGE_LENGTH];
        uint8_t arguments[EVENT_MAX_ARGUMENTS_LENGTH];
        uint8_t num_arguments;
    };

    // Event storage
    EventRecord _events[MAX_EVENTS];
    int _event_count{0};
    int _event_index{0};
    uint16_t _sequence_number{0};

    // Event filtering and routing
    uint8_t _log_level_filter{events::Log::Debug};
    bool _mavlink_enabled{true};
    bool _console_enabled{true};

    // Publications
    uORB::Publication<event_s> _event_pub{ORB_ID(event)};
    uORB::PublicationMulti<mavlink_log_s> _mavlink_log_pub{ORB_ID(mavlink_log)};

    // Subscriptions for event requests
    uORB::Subscription _event_request_sub{ORB_ID(event_request)};
};

// Global event logging functions
namespace events
{
    // Event severity levels
    enum class Log : uint8_t {
        Emergency = 0,  // System unusable
        Alert = 1,      // Action must be taken immediately
        Critical = 2,   // Critical conditions
        Error = 3,      // Error conditions
        Warning = 4,    // Warning conditions
        Notice = 5,     // Normal but significant
        Info = 6,       // Informational
        Debug = 7       // Debug messages
    };

    // Event ID structure
    struct ID {
        uint32_t id;
        constexpr ID(const char* name) : id(hash_32_fnv1a(name)) {}
    };

    // Event sending functions
    void send(ID id, Log log_level, const char* message);

    template<typename... Args>
    void send(ID id, Log log_level, const char* message, Args... args);
}
}
```

#### **Event Management Architecture**
```mermaid
flowchart TB
    subgraph "Event Sources"
        MODULES[System Modules]
        DRIVERS[Device Drivers]
        CONTROLLERS[Controllers]
        SENSORS[Sensor Processing]
        SAFETY[Safety Systems]
    end

    subgraph "Event Processing"
        COLLECTION[Event Collection]
        FILTERING[Level Filtering]
        ROUTING[Message Routing]
        STORAGE[Event Storage]
    end

    subgraph "Event Categories"
        EMERGENCY[Emergency Events]
        CRITICAL[Critical Events]
        WARNING[Warning Events]
        INFO[Info Events]
        DEBUG[Debug Events]
    end

    subgraph "Event Outputs"
        CONSOLE[Console Output]
        MAVLINK[MAVLink Messages]
        LOGGING[Flight Logs]
        TELEMETRY[Ground Station]
    end

    subgraph "Event Consumers"
        COMMANDER[Commander]
        NAVIGATOR[Navigator]
        FAILSAFE[Failsafe Systems]
        GROUND[Ground Control]
    end

    MODULES --> COLLECTION
    DRIVERS --> COLLECTION
    CONTROLLERS --> COLLECTION
    SENSORS --> COLLECTION
    SAFETY --> COLLECTION

    COLLECTION --> FILTERING
    FILTERING --> ROUTING
    ROUTING --> STORAGE

    FILTERING --> EMERGENCY
    FILTERING --> CRITICAL
    FILTERING --> WARNING
    FILTERING --> INFO
    FILTERING --> DEBUG

    ROUTING --> CONSOLE
    ROUTING --> MAVLINK
    ROUTING --> LOGGING
    ROUTING --> TELEMETRY

    STORAGE --> COMMANDER
    STORAGE --> NAVIGATOR
    STORAGE --> FAILSAFE
    STORAGE --> GROUND
```

#### **Event Processing Implementation**
```cpp
void Events::Run()
{
    // 1. PROCESS INCOMING EVENT REQUESTS
    process_event_requests();

    // 2. HANDLE PENDING EVENTS
    process_pending_events();

    // 3. MANAGE EVENT STORAGE
    manage_event_storage();

    // 4. UPDATE EVENT ROUTING
    update_event_routing();

    // 5. SCHEDULE NEXT PROCESSING
    ScheduleDelayed(EVENTS_PROCESSING_INTERVAL_US);
}

void Events::process_event_requests()
{
    event_request_s event_request;

    while (_event_request_sub.update(&event_request)) {

        // Validate event request
        if (event_request.timestamp == 0) {
            continue;
        }

        // Create event record
        EventRecord new_event;
        new_event.id = event_request.id;
        new_event.log_level = event_request.log_level;
        new_event.timestamp = event_request.timestamp;
        new_event.sequence = _sequence_number++;

        // Copy message and arguments
        strncpy(new_event.message, event_request.message, EVENT_MAX_MESSAGE_LENGTH - 1);
        new_event.message[EVENT_MAX_MESSAGE_LENGTH - 1] = '\0';

        new_event.num_arguments = math::min((int)event_request.num_arguments,
                                           EVENT_MAX_ARGUMENTS_LENGTH);
        memcpy(new_event.arguments, event_request.arguments, new_event.num_arguments);

        // Store event
        store_event(new_event);

        // Route event based on level and configuration
        route_event(new_event);
    }
}

void Events::store_event(const EventRecord &event)
{
    // Store in circular buffer
    _events[_event_index] = event;
    _event_index = (_event_index + 1) % MAX_EVENTS;

    if (_event_count < MAX_EVENTS) {
        _event_count++;
    }

    // Publish event for subscribers
    event_s published_event{};
    published_event.timestamp = event.timestamp;
    published_event.id = event.id;
    published_event.log_level = event.log_level;
    published_event.sequence = event.sequence;

    strncpy(published_event.message, event.message, sizeof(published_event.message) - 1);
    published_event.message[sizeof(published_event.message) - 1] = '\0';

    _event_pub.publish(published_event);
}

void Events::route_event(const EventRecord &event)
{
    // 1. FILTER BY LOG LEVEL
    if (event.log_level > _log_level_filter) {
        return;  // Event filtered out
    }

    // 2. CONSOLE OUTPUT
    if (_console_enabled) {
        route_to_console(event);
    }

    // 3. MAVLINK OUTPUT
    if (_mavlink_enabled) {
        route_to_mavlink(event);
    }

    // 4. SPECIAL HANDLING FOR CRITICAL EVENTS
    if (event.log_level <= events::Log::Critical) {
        handle_critical_event(event);
    }
}

void Events::route_to_console(const EventRecord &event)
{
    // Format event for console output
    const char* level_str = get_log_level_string(event.log_level);

    // Print to console with timestamp and formatting
    printf("[%llu] %s: %s\n",
           event.timestamp,
           level_str,
           event.message);

    // For emergency/critical events, also print to stderr
    if (event.log_level <= events::Log::Critical) {
        fprintf(stderr, "[CRITICAL] %s\n", event.message);
    }
}

void Events::route_to_mavlink(const EventRecord &event)
{
    mavlink_log_s mavlink_log{};
    mavlink_log.timestamp = hrt_absolute_time();
    mavlink_log.severity = convert_to_mavlink_severity(event.log_level);

    // Copy message (truncate if necessary for MAVLink)
    strncpy(mavlink_log.text, event.message, sizeof(mavlink_log.text) - 1);
    mavlink_log.text[sizeof(mavlink_log.text) - 1] = '\0';

    _mavlink_log_pub.publish(mavlink_log);
}

void Events::handle_critical_event(const EventRecord &event)
{
    // For critical events, take additional actions

    switch (event.log_level) {
        case events::Log::Emergency:
            // System is unusable - notify failsafe system
            trigger_emergency_response(event);
            break;

        case events::Log::Alert:
            // Action must be taken immediately
            trigger_alert_response(event);
            break;

        case events::Log::Critical:
            // Critical conditions - log immediately
            force_immediate_logging(event);
            break;
    }

    // Flash status LED for critical events
    flash_status_led(event.log_level);
}

const char* Events::get_log_level_string(uint8_t log_level)
{
    switch (log_level) {
        case events::Log::Emergency: return "EMERGENCY";
        case events::Log::Alert:     return "ALERT";
        case events::Log::Critical:  return "CRITICAL";
        case events::Log::Error:     return "ERROR";
        case events::Log::Warning:   return "WARNING";
        case events::Log::Notice:    return "NOTICE";
        case events::Log::Info:      return "INFO";
        case events::Log::Debug:     return "DEBUG";
        default:                     return "UNKNOWN";
    }
}

uint8_t Events::convert_to_mavlink_severity(uint8_t log_level)
{
    // Convert PX4 log levels to MAVLink severity levels
    switch (log_level) {
        case events::Log::Emergency: return MAV_SEVERITY_EMERGENCY;
        case events::Log::Alert:     return MAV_SEVERITY_ALERT;
        case events::Log::Critical:  return MAV_SEVERITY_CRITICAL;
        case events::Log::Error:     return MAV_SEVERITY_ERROR;
        case events::Log::Warning:   return MAV_SEVERITY_WARNING;
        case events::Log::Notice:    return MAV_SEVERITY_NOTICE;
        case events::Log::Info:      return MAV_SEVERITY_INFO;
        case events::Log::Debug:     return MAV_SEVERITY_DEBUG;
        default:                     return MAV_SEVERITY_INFO;
    }
}
```

#### **Event Generation API**
```cpp
// Global event sending implementation
namespace events
{
    // Hash function for event IDs
    constexpr uint32_t hash_32_fnv1a(const char* str)
    {
        uint32_t hash = 2166136261U;
        while (*str) {
            hash ^= static_cast<uint32_t>(*str++);
            hash *= 16777619U;
        }
        return hash;
    }

    // Simple event sending
    void send(ID id, Log log_level, const char* message)
    {
        event_request_s event_request{};
        event_request.timestamp = hrt_absolute_time();
        event_request.id = id.id;
        event_request.log_level = static_cast<uint8_t>(log_level);
        event_request.num_arguments = 0;

        strncpy(event_request.message, message, sizeof(event_request.message) - 1);
        event_request.message[sizeof(event_request.message) - 1] = '\0';

        // Publish event request
        orb_advert_t event_request_pub = orb_advertise(ORB_ID(event_request), &event_request);
        if (event_request_pub != nullptr) {
            orb_publish(ORB_ID(event_request), event_request_pub, &event_request);
            orb_unadvertise(event_request_pub);
        }
    }

    // Template event sending with arguments
    template<typename... Args>
    void send(ID id, Log log_level, const char* message, Args... args)
    {
        event_request_s event_request{};
        event_request.timestamp = hrt_absolute_time();
        event_request.id = id.id;
        event_request.log_level = static_cast<uint8_t>(log_level);

        // Format message with arguments
        char formatted_message[EVENT_MAX_MESSAGE_LENGTH];
        snprintf(formatted_message, sizeof(formatted_message), message, args...);

        strncpy(event_request.message, formatted_message, sizeof(event_request.message) - 1);
        event_request.message[sizeof(event_request.message) - 1] = '\0';

        // Pack arguments for structured logging
        pack_arguments(event_request.arguments, event_request.num_arguments, args...);

        // Publish event request
        orb_advert_t event_request_pub = orb_advertise(ORB_ID(event_request), &event_request);
        if (event_request_pub != nullptr) {
            orb_publish(ORB_ID(event_request), event_request_pub, &event_request);
            orb_unadvertise(event_request_pub);
        }
    }

    // Example usage throughout PX4 system:
    // events::send(events::ID("takeoff_complete"), events::Log::Info, "Takeoff completed at altitude {1}m", altitude);
    // events::send(events::ID("sensor_failure"), events::Log::Error, "IMU sensor {1} failure detected", sensor_id);
    // events::send(events::ID("mission_started"), events::Log::Notice, "Mission started with {1} waypoints", waypoint_count);
}

// Event usage examples in other modules
void commander_example_usage()
{
    // Armed state change
    events::send(events::ID("armed_state_changed"),
                events::Log::Info,
                "Vehicle armed state: {1}", armed ? "ARMED" : "DISARMED");

    // GPS health
    if (gps_failed) {
        events::send(events::ID("gps_failure"),
                    events::Log::Error,
                    "GPS signal lost, accuracy: {1}m", gps_accuracy);
    }

    // Battery warning
    events::send<float>(events::ID("battery_low"),
                       events::Log::Warning,
                       "Battery voltage low: {1}V", battery_voltage);
}
```

---

## **Safety and Monitoring Summary**

These modules provide essential system health monitoring and event management:

1. **Load Monitor** ← System resource monitoring, performance analysis, stack overflow detection
2. **Events** ← Centralized event logging, message routing, critical event handling

---

## **Utility and Management Modules - Detailed**

### **1. Logger Module**

**Location**: `src/modules/logger/`
**Main Class**: `Logger`
**Work Queue**: `lp_default`

#### **Purpose and Functionality**
The **Logger** module provides comprehensive flight data logging capabilities, recording all sensor data, control commands, system states, and events to ULog files for post-flight analysis, debugging, and regulatory compliance.

```cpp
namespace logger
{
class Logger : public ModuleBase<Logger>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Logging configuration
    enum LogMode {
        LOG_MODE_DISABLED = 0,
        LOG_MODE_WHEN_ARMED,
        LOG_MODE_FROM_BOOT,
        LOG_MODE_ALWAYS
    };

    // ULog writer
    ULogWriter *_writer{nullptr};

    // Subscription management
    static constexpr int MAX_LOGGED_TOPICS = 200;

    struct LoggedTopic {
        const orb_metadata *metadata;
        uORB::Subscription *subscription;
        uint8_t instance;
        bool is_mandatory;
        hrt_abstime last_logged_time;
        uint32_t log_interval_us;
    };

    LoggedTopic _logged_topics[MAX_LOGGED_TOPICS];
    int _num_logged_topics{0};

    // File management
    char _log_dir[64];
    char _current_log_file[128];
    int _log_file_fd{-1};
    uint32_t _log_file_size{0};
    uint32_t _max_log_file_size{0};

    // Logging state
    bool _logging_enabled{false};
    bool _log_on_start{false};
    hrt_abstime _logging_start_time{0};
    uint32_t _log_session_id{0};

    // Statistics
    uint32_t _bytes_written{0};
    uint32_t _messages_logged{0};
    uint32_t _dropped_messages{0};
};
}
```

#### **Logging Architecture and Data Flow**
```mermaid
flowchart TB
    subgraph "Data Sources"
        SENSORS[Sensor Data]
        CONTROL[Control Commands]
        NAVIGATION[Navigation Data]
        SYSTEM[System States]
        EVENTS[System Events]
        PARAMS[Parameters]
    end

    subgraph "Logger Module"
        SUBSCRIBE[Topic Subscription]
        BUFFER[Message Buffering]
        FORMAT[ULog Formatting]
        COMPRESS[Data Compression]
        WRITER[File Writer]
    end

    subgraph "Logging Modes"
        DISABLED[Disabled]
        ARMED[When Armed]
        BOOT[From Boot]
        ALWAYS[Always On]
    end

    subgraph "Storage Management"
        FILESYSTEM[File System]
        ROTATION[File Rotation]
        CLEANUP[Cleanup Policy]
        METADATA[Log Metadata]
    end

    subgraph "Log Analysis"
        DOWNLOAD[Log Download]
        REPLAY[Log Replay]
        ANALYSIS[Flight Analysis]
        DEBUGGING[Issue Debugging]
    end

    SENSORS --> SUBSCRIBE
    CONTROL --> SUBSCRIBE
    NAVIGATION --> SUBSCRIBE
    SYSTEM --> SUBSCRIBE
    EVENTS --> SUBSCRIBE
    PARAMS --> SUBSCRIBE

    SUBSCRIBE --> BUFFER
    BUFFER --> FORMAT
    FORMAT --> COMPRESS
    COMPRESS --> WRITER

    WRITER --> DISABLED
    WRITER --> ARMED
    WRITER --> BOOT
    WRITER --> ALWAYS

    WRITER --> FILESYSTEM
    FILESYSTEM --> ROTATION
    ROTATION --> CLEANUP
    CLEANUP --> METADATA

    FILESYSTEM --> DOWNLOAD
    DOWNLOAD --> REPLAY
    REPLAY --> ANALYSIS
    ANALYSIS --> DEBUGGING
```

#### **ULog File Format and Message Logging**
```cpp
void Logger::Run()
{
    // 1. CHECK LOGGING CONDITIONS
    if (!should_start_logging()) {
        if (_logging_enabled) {
            stop_logging();
        }
        ScheduleDelayed(LOGGER_INTERVAL_US);
        return;
    }

    // 2. START LOGGING IF NOT ACTIVE
    if (!_logging_enabled) {
        start_logging();
    }

    // 3. LOG PENDING MESSAGES
    log_pending_messages();

    // 4. MANAGE FILE SIZE AND ROTATION
    manage_log_files();

    // 5. UPDATE STATISTICS
    update_logging_statistics();

    // 6. SCHEDULE NEXT CYCLE
    ScheduleDelayed(LOGGER_INTERVAL_US);
}

bool Logger::should_start_logging()
{
    vehicle_status_s vehicle_status;
    if (!_vehicle_status_sub.copy(&vehicle_status)) {
        return false;
    }

    LogMode log_mode = static_cast<LogMode>(_param_log_mode.get());

    switch (log_mode) {
        case LOG_MODE_DISABLED:
            return false;

        case LOG_MODE_WHEN_ARMED:
            return vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

        case LOG_MODE_FROM_BOOT:
            return true;  // Always log after boot

        case LOG_MODE_ALWAYS:
            return true;  // Always log

        default:
            return false;
    }
}

void Logger::start_logging()
{
    // 1. CREATE LOG DIRECTORY
    create_log_directory();

    // 2. GENERATE LOG FILE NAME
    generate_log_filename();

    // 3. INITIALIZE ULOG WRITER
    initialize_ulog_writer();

    // 4. SUBSCRIBE TO TOPICS
    subscribe_to_topics();

    // 5. WRITE LOG HEADER
    write_log_header();

    _logging_enabled = true;
    _logging_start_time = hrt_absolute_time();
    _log_session_id++;

    PX4_INFO("Logging started: %s", _current_log_file);

    events::send(events::ID("logging_started"),
                events::Log::Info,
                "Flight logging started");
}

void Logger::generate_log_filename()
{
    // Create timestamp-based filename
    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = gmtime(&rawtime);

    // Format: YYYY-MM-DD_HH-MM-SS_session.ulg
    snprintf(_current_log_file, sizeof(_current_log_file),
             "%s/%04d-%02d-%02d_%02d-%02d-%02d_%03u.ulg",
             _log_dir,
             timeinfo->tm_year + 1900,
             timeinfo->tm_mon + 1,
             timeinfo->tm_mday,
             timeinfo->tm_hour,
             timeinfo->tm_min,
             timeinfo->tm_sec,
             _log_session_id);
}

void Logger::subscribe_to_topics()
{
    _num_logged_topics = 0;

    // Core system topics (always logged)
    add_topic_to_log(ORB_ID(vehicle_status), 0, true, 100000);  // 10Hz
    add_topic_to_log(ORB_ID(vehicle_local_position), 0, true, 50000);  // 20Hz
    add_topic_to_log(ORB_ID(vehicle_global_position), 0, true, 100000);  // 10Hz
    add_topic_to_log(ORB_ID(vehicle_attitude), 0, true, 20000);  // 50Hz
    add_topic_to_log(ORB_ID(vehicle_angular_velocity), 0, true, 10000);  // 100Hz
    add_topic_to_log(ORB_ID(vehicle_acceleration), 0, true, 10000);  // 100Hz

    // Sensor topics
    add_topic_to_log(ORB_ID(sensor_gyro), 0, true, 5000);  // 200Hz
    add_topic_to_log(ORB_ID(sensor_accel), 0, true, 5000);  // 200Hz
    add_topic_to_log(ORB_ID(sensor_mag), 0, false, 50000);  // 20Hz
    add_topic_to_log(ORB_ID(sensor_baro), 0, false, 100000);  // 10Hz
    add_topic_to_log(ORB_ID(sensor_gps), 0, false, 100000);  // 10Hz

    // Control topics
    add_topic_to_log(ORB_ID(vehicle_rates_setpoint), 0, true, 20000);  // 50Hz
    add_topic_to_log(ORB_ID(vehicle_attitude_setpoint), 0, true, 50000);  // 20Hz
    add_topic_to_log(ORB_ID(vehicle_local_position_setpoint), 0, true, 50000);  // 20Hz
    add_topic_to_log(ORB_ID(actuator_controls), 0, true, 20000);  // 50Hz
    add_topic_to_log(ORB_ID(actuator_outputs), 0, true, 20000);  // 50Hz

    // Navigation topics
    add_topic_to_log(ORB_ID(mission), 0, false, 1000000);  // 1Hz
    add_topic_to_log(ORB_ID(mission_result), 0, false, 0);  // On change
    add_topic_to_log(ORB_ID(position_setpoint_triplet), 0, false, 100000);  // 10Hz

    // System health topics
    add_topic_to_log(ORB_ID(cpuload), 0, false, 1000000);  // 1Hz
    add_topic_to_log(ORB_ID(battery_status), 0, true, 1000000);  // 1Hz
    add_topic_to_log(ORB_ID(system_power), 0, false, 1000000);  // 1Hz

    // Event topics
    add_topic_to_log(ORB_ID(event), 0, false, 0);  // On change
    add_topic_to_log(ORB_ID(mavlink_log), 0, false, 0);  // On change

    PX4_INFO("Subscribed to %d topics for logging", _num_logged_topics);
}

void Logger::add_topic_to_log(const orb_metadata *meta, uint8_t instance,
                             bool mandatory, uint32_t interval_us)
{
    if (_num_logged_topics >= MAX_LOGGED_TOPICS) {
        PX4_ERR("Too many topics to log");
        return;
    }

    LoggedTopic &topic = _logged_topics[_num_logged_topics];
    topic.metadata = meta;
    topic.instance = instance;
    topic.is_mandatory = mandatory;
    topic.log_interval_us = interval_us;
    topic.last_logged_time = 0;

    // Create subscription
    topic.subscription = new uORB::Subscription(meta, instance);

    if (topic.subscription == nullptr) {
        PX4_ERR("Failed to subscribe to topic %s", meta->o_name);
        return;
    }

    _num_logged_topics++;
}

void Logger::log_pending_messages()
{
    if (!_logging_enabled || _writer == nullptr) {
        return;
    }

    hrt_abstime current_time = hrt_absolute_time();

    // Process each subscribed topic
    for (int i = 0; i < _num_logged_topics; i++) {
        LoggedTopic &topic = _logged_topics[i];

        // Check if it's time to log this topic
        if (topic.log_interval_us > 0) {
            if (current_time - topic.last_logged_time < topic.log_interval_us) {
                continue;  // Skip this cycle
            }
        }

        // Check for new data
        if (topic.subscription->updated()) {

            // Read the data
            uint8_t data_buffer[topic.metadata->o_size];
            if (topic.subscription->copy(data_buffer)) {

                // Write to ULog
                if (_writer->write_message(topic.metadata->o_name,
                                         data_buffer,
                                         topic.metadata->o_size,
                                         current_time)) {

                    topic.last_logged_time = current_time;
                    _messages_logged++;
                    _bytes_written += topic.metadata->o_size;

                } else {
                    _dropped_messages++;

                    if (topic.is_mandatory) {
                        PX4_ERR("Failed to log mandatory topic %s", topic.metadata->o_name);
                    }
                }
            }
        }
    }
}

void Logger::write_log_header()
{
    if (_writer == nullptr) {
        return;
    }

    // Write ULog format header
    _writer->write_format_header();

    // Write system information
    ULogMessageInfo system_info;
    system_info.key_len = strlen("sys_name");
    strcpy(system_info.key, "sys_name");
    system_info.value_len = strlen("PX4");
    strcpy(system_info.value, "PX4");
    _writer->write_info(&system_info);

    // Write version information
    ULogMessageInfo version_info;
    version_info.key_len = strlen("ver_sw");
    strcpy(version_info.key, "ver_sw");
    version_info.value_len = strlen(PX4_GIT_VERSION);
    strcpy(version_info.value, PX4_GIT_VERSION);
    _writer->write_info(&version_info);

    // Write build information
    ULogMessageInfo build_info;
    build_info.key_len = strlen("build_type");
    strcpy(build_info.key, "build_type");
    version_info.value_len = strlen(PX4_BUILD_TYPE);
    strcpy(build_info.value, PX4_BUILD_TYPE);
    _writer->write_info(&build_info);

    // Write vehicle configuration
    write_vehicle_config_to_log();

    // Write parameter values
    write_parameters_to_log();
}

void Logger::write_parameters_to_log()
{
    // Log all current parameter values
    param_t param_handle = PARAM_INVALID;

    do {
        param_handle = param_for_index(param_count_used());

        if (param_handle != PARAM_INVALID) {

            // Get parameter info
            param_info_s param_info;
            if (param_get_info(param_handle, &param_info) == 0) {

                // Get parameter value
                param_value_u param_value;
                if (param_get(param_handle, &param_value) == 0) {

                    // Write parameter to log
                    ULogMessageParameter param_msg;
                    param_msg.key_len = strlen(param_info.name);
                    strcpy(param_msg.key, param_info.name);

                    switch (param_info.type) {
                        case PARAM_TYPE_INT32:
                            param_msg.value = param_value.i;
                            break;
                        case PARAM_TYPE_FLOAT:
                            param_msg.value = param_value.f;
                            break;
                    }

                    _writer->write_parameter(&param_msg);
                }
            }
        }

    } while (param_handle != PARAM_INVALID);
}
```

#### **Log File Management and Rotation**
```cpp
void Logger::manage_log_files()
{
    if (!_logging_enabled || _log_file_fd < 0) {
        return;
    }

    // Check current file size
    struct stat file_stat;
    if (fstat(_log_file_fd, &file_stat) == 0) {
        _log_file_size = file_stat.st_size;

        // Check if file size exceeds limit
        if (_max_log_file_size > 0 && _log_file_size > _max_log_file_size) {

            PX4_INFO("Log file size limit reached, rotating log");

            // Close current file and start new one
            close_current_log_file();
            start_new_log_file();
        }
    }

    // Cleanup old log files if disk space is low
    cleanup_old_logs();
}

void Logger::cleanup_old_logs()
{
    // Check available disk space
    struct statvfs disk_stat;
    if (statvfs(_log_dir, &disk_stat) != 0) {
        return;
    }

    uint64_t free_space = disk_stat.f_bavail * disk_stat.f_frsize;
    uint64_t min_free_space = _param_log_min_free_space.get() * 1024 * 1024;  // MB to bytes

    if (free_space < min_free_space) {

        PX4_WARN("Low disk space, cleaning up old logs");

        // Get list of log files sorted by modification time
        std::vector<std::string> log_files;
        get_log_files_sorted_by_age(log_files);

        // Delete oldest files until we have enough space
        for (const auto &log_file : log_files) {

            std::string full_path = std::string(_log_dir) + "/" + log_file;

            if (unlink(full_path.c_str()) == 0) {
                PX4_INFO("Deleted old log file: %s", log_file.c_str());

                // Check if we have enough space now
                if (statvfs(_log_dir, &disk_stat) == 0) {
                    free_space = disk_stat.f_bavail * disk_stat.f_frsize;
                    if (free_space >= min_free_space) {
                        break;
                    }
                }
            }
        }
    }
}

void Logger::stop_logging()
{
    if (!_logging_enabled) {
        return;
    }

    // Write final log entries
    finalize_log_file();

    // Close file
    close_current_log_file();

    // Clean up subscriptions
    cleanup_subscriptions();

    _logging_enabled = false;

    // Calculate session statistics
    hrt_abstime session_duration = hrt_absolute_time() - _logging_start_time;
    float duration_seconds = session_duration / 1000000.0f;
    float avg_data_rate = _bytes_written / duration_seconds;

    PX4_INFO("Logging stopped. Duration: %.1fs, Data: %u bytes (%.1f KB/s), Messages: %u, Dropped: %u",
             duration_seconds, _bytes_written, avg_data_rate / 1024.0f,
             _messages_logged, _dropped_messages);

    events::send<uint32_t, uint32_t>(events::ID("logging_stopped"),
                                    events::Log::Info,
                                    "Logging stopped: {1} messages, {2} bytes",
                                    _messages_logged, _bytes_written);
}
```

---

### **2. Data Manager Module**

**Location**: `src/modules/dataman/`
**Main Class**: `DataManager`
**Work Queue**: `lp_default`

#### **Purpose and Functionality**
The **Data Manager** module provides persistent storage for mission data, parameters, geofence definitions, and other critical flight data that needs to survive power cycles and system reboots.

```cpp
namespace dataman
{
// Data item types
enum dm_item_t {
    DM_KEY_WAYPOINTS_OFFBOARD_0 = 0,
    DM_KEY_WAYPOINTS_OFFBOARD_1,
    DM_KEY_WAYPOINTS_ONBOARD,
    DM_KEY_SAFE_POINTS,
    DM_KEY_FENCE_POINTS,
    DM_KEY_MISSION_STATE,
    DM_KEY_COMPAT,
    DM_KEY_NUM_KEYS
};

class DataManager : public ModuleBase<DataManager>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Storage backend
    StorageBackend *_storage_backend{nullptr};

    // Data caching
    static constexpr int CACHE_SIZE = 64;

    struct CacheEntry {
        dm_item_t item;
        unsigned index;
        uint8_t *data;
        size_t data_len;
        hrt_abstime timestamp;
        bool dirty;
    };

    CacheEntry _cache[CACHE_SIZE];
    int _cache_count{0};

    // Request handling
    uORB::Subscription _dataman_request_sub{ORB_ID(dataman_request)};
    uORB::Publication<dataman_response_s> _dataman_response_pub{ORB_ID(dataman_response)};

    // Statistics
    uint32_t _read_requests{0};
    uint32_t _write_requests{0};
    uint32_t _cache_hits{0};
    uint32_t _cache_misses{0};
};
}
```

#### **Data Management Architecture**
```mermaid
flowchart TB
    subgraph "Data Types"
        MISSION[Mission Waypoints]
        PARAMS[Parameters]
        FENCE[Geofence Points]
        SAFE[Safe Points]
        STATE[Mission State]
        COMPAT[Compatibility Data]
    end

    subgraph "Data Manager"
        REQUEST[Request Handler]
        CACHE[Data Cache]
        VALIDATE[Data Validation]
        BACKEND[Storage Backend]
    end

    subgraph "Storage Backends"
        FLASH[Flash Storage]
        EEPROM[EEPROM]
        FILE[File System]
        RAMTRON[RAMTRON]
    end

    subgraph "Access Patterns"
        READ[Read Operations]
        WRITE[Write Operations]
        BATCH[Batch Operations]
        ATOMIC[Atomic Updates]
    end

    subgraph "Data Consumers"
        NAVIGATOR[Navigator]
        COMMANDER[Commander]
        GEOFENCE[Geofence]
        MISSION_MGR[Mission Manager]
    end

    MISSION --> REQUEST
    PARAMS --> REQUEST
    FENCE --> REQUEST
    SAFE --> REQUEST
    STATE --> REQUEST
    COMPAT --> REQUEST

    REQUEST --> CACHE
    CACHE --> VALIDATE
    VALIDATE --> BACKEND

    BACKEND --> FLASH
    BACKEND --> EEPROM
    BACKEND --> FILE
    BACKEND --> RAMTRON

    CACHE --> READ
    CACHE --> WRITE
    CACHE --> BATCH
    CACHE --> ATOMIC

    CACHE --> NAVIGATOR
    CACHE --> COMMANDER
    CACHE --> GEOFENCE
    CACHE --> MISSION_MGR
```

#### **Persistent Data Storage Implementation**
```cpp
void DataManager::Run()
{
    // 1. PROCESS DATAMAN REQUESTS
    process_dataman_requests();

    // 2. FLUSH DIRTY CACHE ENTRIES
    flush_dirty_cache_entries();

    // 3. MANAGE CACHE MEMORY
    manage_cache_memory();

    // 4. PERFORM MAINTENANCE
    perform_storage_maintenance();

    // 5. SCHEDULE NEXT CYCLE
    ScheduleDelayed(DATAMAN_INTERVAL_US);
}

void DataManager::process_dataman_requests()
{
    dataman_request_s request;

    while (_dataman_request_sub.update(&request)) {

        dataman_response_s response{};
        response.timestamp = hrt_absolute_time();
        response.client_id = request.client_id;
        response.request_type = request.request_type;
        response.item = request.item;
        response.index = request.index;

        switch (request.request_type) {
            case dataman_request_s::READ:
                response.status = handle_read_request(request, response);
                break;

            case dataman_request_s::WRITE:
                response.status = handle_write_request(request, response);
                break;

            case dataman_request_s::CLEAR:
                response.status = handle_clear_request(request, response);
                break;

            case dataman_request_s::RESTART:
                response.status = handle_restart_request(request, response);
                break;

            default:
                response.status = dataman_response_s::STATUS_INVALID_REQUEST;
                break;
        }

        _dataman_response_pub.publish(response);
    }
}

uint8_t DataManager::handle_read_request(const dataman_request_s &request,
                                        dataman_response_s &response)
{
    _read_requests++;

    // 1. CHECK CACHE FIRST
    CacheEntry *cache_entry = find_cache_entry(request.item, request.index);

    if (cache_entry != nullptr) {
        // Cache hit
        _cache_hits++;

        response.data_len = cache_entry->data_len;
        memcpy(response.data, cache_entry->data, cache_entry->data_len);

        // Update access time
        cache_entry->timestamp = hrt_absolute_time();

        return dataman_response_s::STATUS_SUCCESS;
    }

    // 2. CACHE MISS - READ FROM STORAGE
    _cache_misses++;

    uint8_t storage_data[DM_MAX_DATA_SIZE];
    size_t storage_data_len;

    if (!_storage_backend->read(request.item, request.index,
                               storage_data, sizeof(storage_data),
                               &storage_data_len)) {
        return dataman_response_s::STATUS_READ_ERROR;
    }

    // 3. VALIDATE DATA
    if (!validate_data(request.item, storage_data, storage_data_len)) {
        return dataman_response_s::STATUS_INVALID_DATA;
    }

    // 4. ADD TO CACHE
    add_to_cache(request.item, request.index, storage_data, storage_data_len);

    // 5. RETURN DATA
    response.data_len = storage_data_len;
    memcpy(response.data, storage_data, storage_data_len);

    return dataman_response_s::STATUS_SUCCESS;
}

uint8_t DataManager::handle_write_request(const dataman_request_s &request,
                                         dataman_response_s &response)
{
    _write_requests++;

    // 1. VALIDATE INPUT DATA
    if (!validate_data(request.item, request.data, request.data_len)) {
        return dataman_response_s::STATUS_INVALID_DATA;
    }

    // 2. UPDATE OR ADD TO CACHE
    CacheEntry *cache_entry = find_cache_entry(request.item, request.index);

    if (cache_entry != nullptr) {
        // Update existing cache entry
        free(cache_entry->data);
        cache_entry->data = (uint8_t *)malloc(request.data_len);

        if (cache_entry->data == nullptr) {
            return dataman_response_s::STATUS_OUT_OF_MEMORY;
        }

        memcpy(cache_entry->data, request.data, request.data_len);
        cache_entry->data_len = request.data_len;
        cache_entry->timestamp = hrt_absolute_time();
        cache_entry->dirty = true;

    } else {
        // Add new cache entry
        if (!add_to_cache(request.item, request.index, request.data, request.data_len)) {
            return dataman_response_s::STATUS_OUT_OF_MEMORY;
        }

        // Mark as dirty for later storage write
        cache_entry = find_cache_entry(request.item, request.index);
        if (cache_entry != nullptr) {
            cache_entry->dirty = true;
        }
    }

    // 3. IMMEDIATE WRITE FOR CRITICAL DATA
    if (is_critical_data(request.item)) {
        if (!write_to_storage(request.item, request.index, request.data, request.data_len)) {
            return dataman_response_s::STATUS_WRITE_ERROR;
        }

        if (cache_entry != nullptr) {
            cache_entry->dirty = false;
        }
    }

    return dataman_response_s::STATUS_SUCCESS;
}

bool DataManager::validate_data(dm_item_t item, const uint8_t *data, size_t data_len)
{
    switch (item) {
        case DM_KEY_WAYPOINTS_OFFBOARD_0:
        case DM_KEY_WAYPOINTS_OFFBOARD_1:
        case DM_KEY_WAYPOINTS_ONBOARD:
            return validate_mission_item(data, data_len);

        case DM_KEY_FENCE_POINTS:
            return validate_fence_point(data, data_len);

        case DM_KEY_SAFE_POINTS:
            return validate_safe_point(data, data_len);

        case DM_KEY_MISSION_STATE:
            return validate_mission_state(data, data_len);

        default:
            return true;  // No validation for unknown types
    }
}

bool DataManager::validate_mission_item(const uint8_t *data, size_t data_len)
{
    if (data_len != sizeof(mission_item_s)) {
        return false;
    }

    const mission_item_s *mission_item = reinterpret_cast<const mission_item_s *>(data);

    // Basic sanity checks
    if (mission_item->nav_cmd == 0) {
        return false;  // Invalid command
    }

    // Coordinate validation
    if (mission_item->nav_cmd == vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT ||
        mission_item->nav_cmd == vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM) {

        // Check latitude range
        if (mission_item->lat < -90.0 || mission_item->lat > 90.0) {
            return false;
        }

        // Check longitude range
        if (mission_item->lon < -180.0 || mission_item->lon > 180.0) {
            return false;
        }

        // Check altitude range (reasonable limits)
        if (mission_item->altitude < -1000.0 || mission_item->altitude > 60000.0) {
            return false;
        }
    }

    // Parameter validation
    for (int i = 0; i < 4; i++) {
        if (!isfinite(mission_item->params[i])) {
            return false;
        }
    }

    return true;
}

bool DataManager::validate_fence_point(const uint8_t *data, size_t data_len)
{
    if (data_len != sizeof(mission_fence_point_s)) {
        return false;
    }

    const mission_fence_point_s *fence_point =
        reinterpret_cast<const mission_fence_point_s *>(data);

    // Coordinate validation
    if (fence_point->lat < -90.0 || fence_point->lat > 90.0) {
        return false;
    }

    if (fence_point->lon < -180.0 || fence_point->lon > 180.0) {
        return false;
    }

    // Altitude validation
    if (!isfinite(fence_point->alt)) {
        return false;
    }

    return true;
}

void DataManager::flush_dirty_cache_entries()
{
    for (int i = 0; i < _cache_count; i++) {

        if (_cache[i].dirty) {

            // Write to storage
            if (write_to_storage(_cache[i].item, _cache[i].index,
                               _cache[i].data, _cache[i].data_len)) {

                _cache[i].dirty = false;

            } else {
                PX4_ERR("Failed to write cache entry to storage: item %d, index %d",
                        _cache[i].item, _cache[i].index);
            }
        }
    }
}

bool DataManager::write_to_storage(dm_item_t item, unsigned index,
                                  const uint8_t *data, size_t data_len)
{
    if (_storage_backend == nullptr) {
        return false;
    }

    // Calculate CRC for data integrity
    uint32_t crc = calculate_crc32(data, data_len);

    // Prepare storage data with header
    struct StorageHeader {
        uint32_t magic;
        uint32_t crc;
        uint32_t data_len;
        uint8_t version;
    } __attribute__((packed));

    StorageHeader header;
    header.magic = DATAMAN_MAGIC;
    header.crc = crc;
    header.data_len = data_len;
    header.version = DATAMAN_VERSION;

    // Write header + data atomically
    uint8_t storage_buffer[sizeof(StorageHeader) + DM_MAX_DATA_SIZE];
    memcpy(storage_buffer, &header, sizeof(StorageHeader));
    memcpy(storage_buffer + sizeof(StorageHeader), data, data_len);

    return _storage_backend->write(item, index, storage_buffer,
                                  sizeof(StorageHeader) + data_len);
}
```

---

### **3. Time Persistor Module**

**Location**: `src/modules/time_persistor/`
**Main Class**: `TimePersistor`
**Work Queue**: `lp_default`

#### **Purpose and Functionality**
The **Time Persistor** module maintains accurate system time across power cycles by saving timestamps to persistent storage and estimating time drift during power-off periods.

```cpp
namespace time_persistor
{
class TimePersistor : public ModuleBase<TimePersistor>, public ModuleParams, public px4::ScheduledWorkItem
{
    // Time persistence data
    struct TimeData {
        uint64_t rtc_time_us;      // RTC time when saved
        uint64_t boot_time_us;     // Boot time estimate
        uint64_t monotonic_time_us; // Monotonic time when saved
        uint32_t boot_count;       // Number of boots
        uint32_t crc;              // Data integrity check
    };

    // Storage management
    static constexpr int TIME_STORAGE_KEY = 42;
    TimeData _time_data{};
    bool _time_data_valid{false};

    // Time synchronization
    hrt_abstime _last_save_time{0};
    hrt_abstime _boot_timestamp{0};
    bool _time_synchronized{false};

    // GPS time tracking
    uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
    uint64_t _last_gps_time_us{0};
    bool _gps_time_valid{false};
};
}
```

#### **Time Synchronization Implementation**
```cpp
void TimePersistor::Run()
{
    // 1. LOAD SAVED TIME ON FIRST RUN
    if (!_time_data_valid) {
        load_saved_time();
    }

    // 2. UPDATE TIME FROM GPS IF AVAILABLE
    update_time_from_gps();

    // 3. SAVE TIME PERIODICALLY
    if (should_save_time()) {
        save_current_time();
    }

    // 4. SCHEDULE NEXT UPDATE
    ScheduleDelayed(TIME_PERSISTOR_INTERVAL_US);
}

void TimePersistor::load_saved_time()
{
    // Read time data from persistent storage
    uint8_t storage_data[sizeof(TimeData)];
    size_t data_len;

    if (dataman_read(DM_KEY_TIME_PERSISTOR, TIME_STORAGE_KEY,
                    storage_data, sizeof(storage_data), &data_len)) {

        if (data_len == sizeof(TimeData)) {
            memcpy(&_time_data, storage_data, sizeof(TimeData));

            // Verify CRC
            uint32_t calculated_crc = calculate_crc32((uint8_t*)&_time_data,
                                                     sizeof(TimeData) - sizeof(uint32_t));

            if (calculated_crc == _time_data.crc) {
                _time_data_valid = true;

                // Estimate current time based on saved data
                estimate_current_time();

                PX4_INFO("Loaded saved time data, boot count: %u", _time_data.boot_count);

            } else {
                PX4_WARN("Time data CRC mismatch, ignoring saved time");
            }
        }
    }

    if (!_time_data_valid) {
        // Initialize with default values
        initialize_time_data();
    }

    _time_data.boot_count++;
}

void TimePersistor::estimate_current_time()
{
    if (!_time_data_valid) {
        return;
    }

    // Get current monotonic time
    uint64_t current_monotonic = hrt_absolute_time();

    // Calculate time elapsed since last save (estimated)
    uint64_t monotonic_elapsed = current_monotonic - _time_data.monotonic_time_us;

    // Estimate current RTC time
    uint64_t estimated_rtc_time = _time_data.rtc_time_us + monotonic_elapsed;

    // Set system time if we have a reasonable estimate
    if (monotonic_elapsed < MAX_TIME_DRIFT_US) {

        struct timespec ts;
        ts.tv_sec = estimated_rtc_time / 1000000;
        ts.tv_nsec = (estimated_rtc_time % 1000000) * 1000;

        if (clock_settime(CLOCK_REALTIME, &ts) == 0) {
            _time_synchronized = true;

            PX4_INFO("System time synchronized from persistent storage");

            events::send(events::ID("time_synchronized"),
                        events::Log::Info,
                        "System time restored from storage");
        }
    }
}

void TimePersistor::update_time_from_gps()
{
    vehicle_gps_position_s gps_position;

    if (_vehicle_gps_position_sub.update(&gps_position)) {

        // Check GPS time validity
        if (gps_position.time_utc_usec > 0 &&
            gps_position.fix_type >= 3) {  // 3D fix or better

            uint64_t gps_time_us = gps_position.time_utc_usec;

            // Validate GPS time (reasonable range check)
            uint64_t min_valid_time = 1577836800000000ULL;  // 2020-01-01 00:00:00 UTC
            uint64_t max_valid_time = 4102444800000000ULL;  // 2100-01-01 00:00:00 UTC

            if (gps_time_us >= min_valid_time && gps_time_us <= max_valid_time) {

                // Check if GPS time has changed significantly
                if (!_gps_time_valid ||
                    abs((int64_t)(gps_time_us - _last_gps_time_us)) > GPS_TIME_CHANGE_THRESHOLD_US) {

                    // Set system time from GPS
                    struct timespec ts;
                    ts.tv_sec = gps_time_us / 1000000;
                    ts.tv_nsec = (gps_time_us % 1000000) * 1000;

                    if (clock_settime(CLOCK_REALTIME, &ts) == 0) {
                        _time_synchronized = true;
                        _gps_time_valid = true;
                        _last_gps_time_us = gps_time_us;

                        PX4_INFO("System time synchronized from GPS");

                        events::send<uint64_t>(events::ID("time_gps_sync"),
                                              events::Log::Info,
                                              "Time synchronized from GPS: {1}",
                                              gps_time_us);

                        // Update time data for next save
                        update_time_data();
                    }
                }
            }
        }
    }
}

bool TimePersistor::should_save_time()
{
    // Don't save too frequently
    if (hrt_elapsed_time(&_last_save_time) < MIN_SAVE_INTERVAL_US) {
        return false;
    }

    // Only save if time is synchronized
    if (!_time_synchronized) {
        return false;
    }

    return true;
}

void TimePersistor::save_current_time()
{
    update_time_data();

    // Calculate CRC
    _time_data.crc = calculate_crc32((uint8_t*)&_time_data,
                                    sizeof(TimeData) - sizeof(uint32_t));

    // Save to persistent storage
    if (dataman_write(DM_KEY_TIME_PERSISTOR, TIME_STORAGE_KEY,
                     (uint8_t*)&_time_data, sizeof(TimeData))) {

        _last_save_time = hrt_absolute_time();

        PX4_DEBUG("Time data saved to persistent storage");

    } else {
        PX4_ERR("Failed to save time data");
    }
}

void TimePersistor::update_time_data()
{
    struct timespec ts;

    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        _time_data.rtc_time_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    }

    _time_data.monotonic_time_us = hrt_absolute_time();

    if (_boot_timestamp == 0) {
        _boot_timestamp = _time_data.monotonic_time_us;
    }

    _time_data.boot_time_us = _boot_timestamp;
}
```

---

### **4. Replay Module**

**Location**: `src/modules/replay/`
**Main Class**: `Replay`
**Work Queue**: `lp_default`

#### **Purpose and Functionality**
The **Replay** module enables replaying of recorded ULog files for testing, debugging, and analysis, allowing developers to reproduce flight scenarios and test algorithm changes offline.

```cpp
namespace replay
{
class Replay : public ModuleBase<Replay>, public ModuleParams, public px4::ScheduledWorkItem
{
    // ULog reader
    ULogReader *_reader{nullptr};

    // Replay state
    enum ReplayState {
        REPLAY_IDLE,
        REPLAY_LOADING,
        REPLAY_RUNNING,
        REPLAY_PAUSED,
        REPLAY_FINISHED
    } _state{REPLAY_IDLE};

    // Timing control
    uint64_t _replay_start_time{0};
    uint64_t _log_start_time{0};
    float _time_factor{1.0f};
    bool _time_sync_enabled{true};

    // Message publishers (dynamic)
    struct ReplayPublisher {
        const orb_metadata *metadata;
        orb_advert_t advertiser;
        uint8_t instance;
    };

    static constexpr int MAX_REPLAY_PUBLISHERS = 100;
    ReplayPublisher _publishers[MAX_REPLAY_PUBLISHERS];
    int _num_publishers{0};

    // Replay configuration
    char _log_file_path[256];
    bool _filter_enabled{false};
    std::vector<std::string> _topic_filter;
};
}
```

#### **Log Replay Implementation**
```cpp
void Replay::Run()
{
    switch (_state) {
        case REPLAY_IDLE:
            // Wait for replay command
            break;

        case REPLAY_LOADING:
            continue_loading();
            break;

        case REPLAY_RUNNING:
            continue_replay();
            break;

        case REPLAY_PAUSED:
            // Wait for resume command
            break;

        case REPLAY_FINISHED:
            cleanup_replay();
            break;
    }

    ScheduleDelayed(REPLAY_INTERVAL_US);
}

bool Replay::start_replay(const char *log_file_path, float time_factor)
{
    if (_state != REPLAY_IDLE) {
        PX4_ERR("Replay already active");
        return false;
    }

    // Copy file path
    strncpy(_log_file_path, log_file_path, sizeof(_log_file_path) - 1);
    _log_file_path[sizeof(_log_file_path) - 1] = '\0';

    _time_factor = time_factor;

    // Initialize ULog reader
    _reader = new ULogReader();
    if (_reader == nullptr) {
        PX4_ERR("Failed to create ULog reader");
        return false;
    }

    if (!_reader->open(_log_file_path)) {
        PX4_ERR("Failed to open log file: %s", _log_file_path);
        delete _reader;
        _reader = nullptr;
        return false;
    }

    _state = REPLAY_LOADING;

    PX4_INFO("Starting replay of: %s (time factor: %.2f)", _log_file_path, _time_factor);

    return true;
}

void Replay::continue_loading()
{
    // Read log header and prepare publishers
    if (!prepare_publishers()) {
        PX4_ERR("Failed to prepare publishers for replay");
        _state = REPLAY_FINISHED;
        return;
    }

    // Get log start time
    _log_start_time = _reader->get_start_timestamp();
    _replay_start_time = hrt_absolute_time();

    _state = REPLAY_RUNNING;

    PX4_INFO("Replay loaded, starting playback");

    events::send(events::ID("replay_started"),
                events::Log::Info,
                "Log replay started");
}

bool Replay::prepare_publishers()
{
    _num_publishers = 0;

    // Get list of message formats from log
    auto message_formats = _reader->get_message_formats();

    for (const auto &format : message_formats) {

        // Skip filtered topics
        if (_filter_enabled && !is_topic_allowed(format.name)) {
            continue;
        }

        // Find matching uORB metadata
        const orb_metadata *metadata = get_orb_metadata(format.name);
        if (metadata == nullptr) {
            PX4_WARN("No uORB metadata found for topic: %s", format.name);
            continue;
        }

        // Create publisher
        if (_num_publishers >= MAX_REPLAY_PUBLISHERS) {
            PX4_ERR("Too many replay publishers");
            return false;
        }

        ReplayPublisher &pub = _publishers[_num_publishers];
        pub.metadata = metadata;
        pub.instance = 0;  // Default instance

        // Advertise topic
        pub.advertiser = orb_advertise_multi(metadata, nullptr, &pub.instance);
        if (pub.advertiser == nullptr) {
            PX4_ERR("Failed to advertise topic: %s", format.name);
            continue;
        }

        _num_publishers++;

        PX4_DEBUG("Prepared publisher for topic: %s", format.name);
    }

    PX4_INFO("Prepared %d publishers for replay", _num_publishers);
    return true;
}

void Replay::continue_replay()
{
    // Calculate current replay time
    uint64_t elapsed_real_time = hrt_absolute_time() - _replay_start_time;
    uint64_t target_log_time = _log_start_time + (uint64_t)(elapsed_real_time * _time_factor);

    // Read and publish messages up to target time
    ULogMessage message;

    while (_reader->read_message(message)) {

        if (message.timestamp > target_log_time) {
            // Future message, wait for next cycle
            break;
        }

        // Find publisher for this message
        ReplayPublisher *publisher = find_publisher(message.topic_name);
        if (publisher == nullptr) {
            continue;  // Skip unknown topics
        }

        // Publish message
        if (orb_publish(publisher->metadata, publisher->advertiser, message.data) != PX4_OK) {
            PX4_WARN("Failed to publish replay message: %s", message.topic_name);
        }
    }

    // Check if we've reached end of log
    if (_reader->is_eof()) {
        _state = REPLAY_FINISHED;

        PX4_INFO("Replay finished");

        events::send(events::ID("replay_finished"),
                    events::Log::Info,
                    "Log replay completed");
    }
}

void Replay::cleanup_replay()
{
    // Clean up publishers
    for (int i = 0; i < _num_publishers; i++) {
        if (_publishers[i].advertiser != nullptr) {
            orb_unadvertise(_publishers[i].advertiser);
            _publishers[i].advertiser = nullptr;
        }
    }
    _num_publishers = 0;

    // Clean up reader
    if (_reader != nullptr) {
        _reader->close();
        delete _reader;
        _reader = nullptr;
    }

    _state = REPLAY_IDLE;

    PX4_INFO("Replay cleanup completed");
}

ReplayPublisher* Replay::find_publisher(const char *topic_name)
{
    for (int i = 0; i < _num_publishers; i++) {
        if (strcmp(_publishers[i].metadata->o_name, topic_name) == 0) {
            return &_publishers[i];
        }
    }
    return nullptr;
}

bool Replay::is_topic_allowed(const char *topic_name)
{
    if (!_filter_enabled || _topic_filter.empty()) {
        return true;  // No filter, allow all
    }

    for (const auto &filter : _topic_filter) {
        if (strstr(topic_name, filter.c_str()) != nullptr) {
            return true;  // Topic matches filter
        }
    }

    return false;  // Topic not in filter
}
```

---

## **Complete Guide Summary**

🎉 **Congratulations!** We have successfully created a comprehensive guide covering **all common multicopter modules in PX4**:

### **✅ Completed Sections:**

1. **Core System Modules** (5 modules)
   - Commander, Sensors, EKF2, Navigator, MAVLink

2. **Sensor and Data Processing Modules** (4 modules)
   - Battery Status, Airspeed Selector, Temperature Compensation, Gyro Calibration

3. **Communication and Interface Modules** (3 modules)
   - Manual Control, RC Update, Camera Feedback

4. **Navigation and Control Support Modules** (3 modules)
   - Land Detector, Flight Mode Manager, Control Allocator

5. **Safety and Monitoring Modules** (2 modules)
   - Load Monitor, Events

6. **Utility and Management Modules** (4 modules)
   - Logger, Data Manager, Time Persistor, Replay

### **📋 Total Coverage:**
- **21 Essential Modules** detailed with full implementations
- **C++ class structures** and real-world code examples
- **uORB communication patterns** and message flow
- **Mermaid diagrams** for architecture visualization
- **State machines** and control logic
- **Error handling** and failsafe mechanisms
- **Parameter management** and configuration
- **Performance optimization** and resource monitoring

This guide provides developers with deep technical insights into PX4's modular architecture and serves as a comprehensive reference for multicopter development! 🚁✨
```mermaid
flowchart TB
    subgraph "Control Pipeline"
        CTRL[Control Loops]
        CA[Control Allocator]
        MIX[Mixer Module]
    end

    subgraph "Output Drivers"
        PWM_DRV[PWM Driver]
        DSHOT_DRV[DShot Driver]
        UAVCAN_DRV[UAVCAN Driver]
        PX4IO_DRV[PX4IO Driver]
    end

    subgraph "Hardware Layer"
        TIMER[Hardware Timers]
        GPIO[GPIO Pins]
        CAN_BUS[CAN Bus]
        SERIAL[Serial Interface]
    end

    subgraph "Physical Actuators"
        ESC[ESCs]
        SERVO[Servos]
        MOTOR[Motors]
    end

    CTRL --> CA
    CA --> MIX

    MIX --> PWM_DRV
    MIX --> DSHOT_DRV
    MIX --> UAVCAN_DRV
    MIX --> PX4IO_DRV

    PWM_DRV --> TIMER
    DSHOT_DRV --> TIMER
    UAVCAN_DRV --> CAN_BUS
    PX4IO_DRV --> SERIAL

    TIMER --> GPIO
    GPIO --> ESC
    ESC --> MOTOR
    GPIO --> SERVO
    CAN_BUS --> ESC
    SERIAL --> ESC
```

#### **Output Driver Modules (The Missing Link)**

**1. PWM Driver (`src/drivers/pwm_out/`)**
```cpp
class PWMOut : public ModuleBase<PWMOut>, public OutputModuleInterface
{
    // Hardware timer management
    MixingOutput _mixing_output{PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this,
                               MixingOutput::SchedulingPolicy::Auto, false, false};

    // PWM generation
    bool updateOutputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
                      unsigned num_control_groups_updated) override
    {
        for (unsigned i = 0; i < num_outputs; i++) {
            // Convert normalized output (0.0-1.0) to PWM microseconds (1000-2000μs)
            uint16_t pwm_value = _mixing_output.outputFunction(i) == OutputFunction::Motor1 ?
                               (1100 + (outputs[i] * 800 / PWM_HIGHEST_MAX)) :  // Motor: 1100-1900μs
                               (1000 + (outputs[i] * 1000 / PWM_HIGHEST_MAX));  // Servo: 1000-2000μs

            // Set hardware timer compare value
            up_pwm_servo_set(i, pwm_value);
        }
        return true;
    }
};
```

**2. DShot Driver (`src/drivers/dshot/`)**
```cpp
class DShot : public ModuleBase<DShot>, public OutputModuleInterface
{
    bool updateOutputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
                      unsigned num_control_groups_updated) override
    {
        for (int i = 0; i < (int)num_outputs; i++) {
            uint16_t output = outputs[i];

            if (output == DSHOT_DISARM_VALUE) {
                // Send motor stop command
                up_dshot_motor_command(i, DShot_cmd_motor_stop, false);
            } else {
                // Convert to DShot throttle value (48-2047)
                uint16_t dshot_value = DSHOT_MIN_THROTTLE +
                    ((output * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)) / UINT16_MAX);

                // Generate DShot bitstream and transmit via DMA
                up_dshot_motor_data_set(i, dshot_value, false);
            }
        }
        return true;
    }
};
```

**3. UAVCAN Driver (`src/drivers/uavcan/`)**
```cpp
class UavcanNode : public ModuleBase<UavcanNode>, public OutputModuleInterface
{
    bool updateOutputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
                      unsigned num_control_groups_updated) override
    {
        // Broadcast ESC commands over CAN bus
        uavcan::equipment::esc::RawCommand raw_command;

        for (unsigned i = 0; i < num_outputs; i++) {
            // Convert to UAVCAN ESC command (-8192 to +8191)
            raw_command.cmd.push_back(static_cast<int16_t>(
                (outputs[i] - PWM_DEFAULT_MID) * 8191 / (PWM_HIGHEST_MAX - PWM_DEFAULT_MID)
            ));
        }

        // Transmit via CAN
        _esc_raw_cmd_pub.broadcast(raw_command);
        return true;
    }
};
```

**4. Mixer Module (`src/lib/mixer_module/`)**
```cpp
class MixingOutput
{
    void updateSubscriptions(bool allow_wq_switch)
    {
        // Subscribe to control allocator outputs
        if (_control_allocator_sub.updated()) {
            actuator_motors_s actuator_motors;
            actuator_servos_s actuator_servos;

            _control_allocator_sub.copy(&actuator_motors);

            // Route to appropriate outputs based on function assignment
            for (int i = 0; i < _max_num_outputs; i++) {
                OutputFunction function = (OutputFunction)_parameters.function[i].get();

                switch (function) {
                    case OutputFunction::Motor1:
                    case OutputFunction::Motor2:
                    case OutputFunction::Motor3:
                    case OutputFunction::Motor4:
                        _outputs[i] = actuator_motors.control[function - OutputFunction::Motor1];
                        break;

                    case OutputFunction::Servo1:
                    case OutputFunction::Servo2:
                        _outputs[i] = actuator_servos.control[function - OutputFunction::Servo1];
                        break;
                }
            }
        }
    }
};
```

---

## **Appendix B: Command Interface - How to Send Setpoints to PX4**

### **Overview: Multiple Command Interfaces**

After compiling and uploading PX4 firmware, you can send commands through several interfaces:

```mermaid
flowchart TB
    subgraph "Command Sources"
        GCS[Ground Control Station]
        COMPANION[Companion Computer]
        RC[RC Transmitter]
        SCRIPT[Python/C++ Scripts]
        ROS[ROS2 Nodes]
    end

    subgraph "Communication Protocols"
        MAVLINK[MAVLink Protocol]
        MICRORTPS[Micro RTPS/DDS]
        SERIAL[Serial Commands]
        UORB[Direct uORB]
    end

    subgraph "PX4 Command Processing"
        MAVLINK_RX[MAVLink Receiver]
        OFFBOARD[Offboard Mode]
        MANUAL_CTL[Manual Control]
        MISSION[Mission Commands]
    end

    subgraph "Internal Setpoints"
        POS_SP[Position Setpoint]
        ATT_SP[Attitude Setpoint]
        RATE_SP[Rate Setpoint]
        THR_SP[Thrust Setpoint]
    end

    GCS --> MAVLINK
    COMPANION --> MAVLINK
    COMPANION --> MICRORTPS
    RC --> SERIAL
    SCRIPT --> MAVLINK
    ROS --> MICRORTPS

    MAVLINK --> MAVLINK_RX
    MICRORTPS --> OFFBOARD
    SERIAL --> MANUAL_CTL

    MAVLINK_RX --> OFFBOARD
    MAVLINK_RX --> MISSION

    OFFBOARD --> POS_SP
    OFFBOARD --> ATT_SP
    OFFBOARD --> RATE_SP
    OFFBOARD --> THR_SP

    MANUAL_CTL --> ATT_SP
    MISSION --> POS_SP
```

### **1. MAVLink Commands (Most Common)**

#### **A. Position Setpoints**
```python
# Python example using pymavlink
from pymavlink import mavutil

# Connect to PX4
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Send position setpoint (NED coordinates)
master.mav.set_position_target_local_ned_send(
    0,                          # time_boot_ms
    master.target_system,       # target_system
    master.target_component,    # target_component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate_frame
    0b0000111111111000,         # type_mask (position only)
    10.0,                       # x (North) [m]
    5.0,                        # y (East) [m]
    -3.0,                       # z (Down) [m]
    0, 0, 0,                    # vx, vy, vz [m/s] (ignored)
    0, 0, 0,                    # afx, afy, afz [m/s²] (ignored)
    0, 0                        # yaw, yaw_rate (ignored)
)
```

#### **B. Attitude Setpoints**
```python
import math

# Send attitude setpoint (quaternion + thrust)
# Roll=10°, Pitch=5°, Yaw=45°, Thrust=0.6
roll = math.radians(10)
pitch = math.radians(5)
yaw = math.radians(45)

# Convert to quaternion
q = [
    math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2),
    math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2),
    math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2),
    math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
]

master.mav.set_attitude_target_send(
    0,                          # time_boot_ms
    master.target_system,       # target_system
    master.target_component,    # target_component
    0b00000111,                 # type_mask (attitude + thrust)
    q,                          # quaternion [w, x, y, z]
    0, 0, 0,                    # body_roll_rate, body_pitch_rate, body_yaw_rate [rad/s]
    0.6                         # thrust [0.0-1.0]
)
```

#### **C. Rate Setpoints**
```python
# Send angular rate setpoint
master.mav.set_attitude_target_send(
    0,                          # time_boot_ms
    master.target_system,       # target_system
    master.target_component,    # target_component
    0b00000000,                 # type_mask (body rates + thrust)
    [1, 0, 0, 0],              # quaternion (ignored)
    0.1,                        # body_roll_rate [rad/s]
    0.05,                       # body_pitch_rate [rad/s]
    0.2,                        # body_yaw_rate [rad/s]
    0.7                         # thrust [0.0-1.0]
)
```

### **2. MAVSDK (High-Level C++ API)**

```cpp
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>

using namespace mavsdk;

int main()
{
    Mavsdk mavsdk;
    auto result = mavsdk.add_any_connection("udp://:14540");

    auto system = mavsdk.systems().at(0);
    auto action = Action{system};
    auto offboard = Offboard{system};

    // 1. ARM THE VEHICLE
    action.arm();

    // 2. START OFFBOARD MODE
    offboard.start();

    // 3. SEND POSITION SETPOINT
    Offboard::PositionNedYaw position_ned_yaw{};
    position_ned_yaw.north_m = 10.0f;
    position_ned_yaw.east_m = 5.0f;
    position_ned_yaw.down_m = -3.0f;
    position_ned_yaw.yaw_deg = 45.0f;

    offboard.set_position_ned(position_ned_yaw);

    // 4. SEND ATTITUDE SETPOINT
    Offboard::Attitude attitude{};
    attitude.roll_deg = 10.0f;
    attitude.pitch_deg = 5.0f;
    attitude.yaw_deg = 45.0f;
    attitude.thrust_value = 0.6f;

    offboard.set_attitude(attitude);

    // 5. SEND RATE SETPOINT
    Offboard::AttitudeRate attitude_rate{};
    attitude_rate.roll_deg_s = 10.0f;
    attitude_rate.pitch_deg_s = 5.0f;
    attitude_rate.yaw_deg_s = 20.0f;
    attitude_rate.thrust_value = 0.7f;

    offboard.set_attitude_rate(attitude_rate);

    return 0;
}
```

### **3. ROS 2 Interface**

```python
# ROS 2 Python node
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

    def control_loop(self):
        # 1. Enable offboard mode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)

        # 2. Send position setpoint
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [10.0, 5.0, -3.0]  # NED coordinates
        traj_msg.yaw = 0.785  # 45 degrees in radians
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(traj_msg)

def main():
    rclpy.init()
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()
```

### **4. QGroundControl (Manual)**

1. **Connect QGC to vehicle** via MAVLink (USB/telemetry)
2. **Plan Mission**:
   - Plan View → Add waypoints with coordinates
   - Upload mission to vehicle
3. **Manual Control**:
   - Flight View → Virtual joystick for attitude control
   - Set flight modes (Position, Attitude, Stabilized)
4. **Parameter Tuning**:
   - Vehicle Setup → Parameters
   - Modify control gains, setpoints, limits

### **5. PX4 Console Commands**

```bash
# Connect to PX4 shell via MAVLink
px4-shell

# Direct parameter commands
param set MC_ROLL_P 6.5
param set MC_PITCH_P 6.5

# Direct module commands
commander arm
commander takeoff
navigator rtl
```

### **Command Flow Summary**

1. **Offboard Mode**: External computer sends continuous setpoints via MAVLink/ROS2
2. **Mission Mode**: Pre-planned waypoints executed autonomously
3. **Manual Mode**: RC transmitter provides direct pilot input
4. **API Control**: High-level APIs (MAVSDK, DroneKit) simplify command generation
5. **Ground Station**: QGroundControl provides GUI-based mission planning and control

**Key Requirements:**
- Vehicle must be in appropriate flight mode (OFFBOARD for external control)
- Commands must be sent at minimum 2Hz for safety
- Proper coordinate frames (NED, body, global) must be used
- Vehicle must be armed before accepting motion commands

This comprehensive command interface allows full control over PX4 vehicles from takeoff to landing! 🚁
