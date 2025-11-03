# PX4 Commander.cpp Hierarchy – Pictorial Representation

## 1. High-Level Commander Flow

```mermaid
flowchart TD
    START[PX4 Startup] --> INIT[Commander Init]
    INIT --> PREARM[Pre-Arm Safety Checks]
    PREARM -->|All Pass| ARMREQ[Arm Request]
    ARMREQ --> ARMCHECKS[Repeat All Checks]
    ARMCHECKS -->|All Pass| ARMED[Set ARMED State]
    ARMED --> FLIGHTLOOP[Main Flight Loop]
    FLIGHTLOOP -->|MAVLink/RC| SETPOINTS[Receive Setpoints]
    SETPOINTS --> CONTROLLERS[Position/Attitude/Rate Controllers]
    CONTROLLERS --> MOTORS[Motor Outputs]
```

---

## 2. Commander Pre-Arm Safety Checks (Expanded)

```mermaid
graph TD
    PREARM[Pre-Arm Checks]
    PREARM --> SENSORS[Sensor Health]
    PREARM --> CALIB[Sensor Calibration]
    PREARM --> BATTERY[Battery Health]
    PREARM --> RC[RC Input]
    PREARM --> ESTIMATOR[Estimator Health]
    PREARM --> GEOFENCE[Geofence]
    PREARM --> MISSION[Mission Validity]
    PREARM --> CIRCUIT[No Circuit Breakers]
    PREARM --> FAILSAFE[Failsafe Flags]
    PREARM --> MANUAL[Manual Control Sticks]
    PREARM --> ARMSW[Arm Switch]
    PREARM --> OTHER[Other Checks]
```

---

## 3. Commander State Machine (Simplified)

```mermaid
stateDiagram-v2
    [*] --> STANDBY: Power On
    STANDBY --> ARMING: Arm Switch
    ARMING --> ARMED: All Checks Pass
    ARMING --> STANDBY: Any Check Fails
    ARMED --> FLIGHT: Flight Modes
    FLIGHT --> FAILSAFE: Failsafe Triggered
    FAILSAFE --> ARMED: Recovery
    ARMED --> DISARMED: Land Detected
    DISARMED --> STANDBY
```

---

## 4. Commander to Controller Data Flow

```mermaid
flowchart LR
    CMD[Commander] --> STATUS[vehicle_status]
    CMD --> MODE[vehicle_control_mode]
    CMD --> FAILSAFE[vehicle_status_flags]
    CMD -->|Setpoints| CONTROLLERS[Controllers]
    CONTROLLERS --> RATE[mc_rate_control]
    CONTROLLERS --> ATT[mc_att_control]
    CONTROLLERS --> POS[mc_pos_control]
    CONTROLLERS --> ALLOC[control_allocator]
    ALLOC --> MOTORS[actuator_motors]
```

---

## 5. Commander Arming Sequence (Detailed)

```mermaid
flowchart TD
    ARMREQ[Arm Request] --> CHECKS[Run All Pre-Arm Checks]
    CHECKS -->|Pass| SETARM[Set ARMED State]
    CHECKS -->|Fail| REJECT[Reject Arming]
    SETARM --> PUB[Publish vehicle_status]
    SETARM --> PUBMODE[Publish vehicle_control_mode]
    PUB --> CONTROLLERS
    REJECT --> []
```

---

## 6. Commander Failsafe Handling

```mermaid
flowchart TD
    FLIGHT[In Flight] --> FAILSAFE[Monitor Failsafes]
    FAILSAFE -->|Battery Low| RTL[Return to Launch]
    FAILSAFE -->|RC Loss| RTL
    FAILSAFE -->|Datalink Loss| RTL
    FAILSAFE -->|Geofence| RTL
    RTL --> LAND[Land]
    LAND --> DISARM[Disarm]
```

---

## 7. Commander Mode Switching

```mermaid
flowchart TD
    ARMED[Armed] --> MANUAL[Manual Mode]
    ARMED --> OFFBOARD[Offboard Mode]
    ARMED --> AUTO[Auto Mode]
    AUTO --> MISSION[Mission]
    AUTO --> LOITER[Loiter]
    AUTO --> RTL[Return to Launch]
    AUTO --> LAND[Land]
    MANUAL --> ALTCTL[Altitude Control]
    ALTCTL --> POSCTL[Position Control]
    POSCTL --> AUTO
```
