# Minimal Commander Hierarchy – Pictorial Representation


```mermaid
flowchart TD
    START[PX4 Startup] --> INIT[Minimal Commander Init]
    INIT --> PREARM[Pre-Arm Safety Checks 5]
    PREARM -->|All Pass| ARMREQ[Arm Request]
    ARMREQ --> ARMCHECKS[Repeat 5 Checks]
    ARMCHECKS -->|All Pass| ARMED[Set ARMED State]
    ARMED --> FLIGHTLOOP[Main Flight Loop]
    FLIGHTLOOP -->|MAVLink| SETPOINTS[Receive Rate Setpoints]
    SETPOINTS --> RATECTRL[Rate Controller Only]
    RATECTRL --> MOTORS[Motor Outputs]
```

---

## 2. Minimal Commander Pre-Arm Safety Checks (Expanded)

```mermaid
graph TD
    PREARM[Pre-Arm Checks]
    PREARM --> BATTERY[Battery Voltage > 10.5V]
    PREARM --> SENSORS[IMU/Gyro/Accel Healthy]
    PREARM --> EKF[EKF2 Attitude Valid]
    PREARM --> MODE[Offboard Mode Active]
    PREARM --> RC[RC Input Valid Optional]
```

---

## 3. Minimal Commander State Machine (Simplified)

```mermaid
stateDiagram-v2
    [*] --> STANDBY: Power On
    STANDBY --> ARMING: Arm Command (MAVLink)
    ARMING --> ARMED: All 5 Checks Pass
    ARMING --> STANDBY: Any Check Fails
    ARMED --> FLIGHT: Offboard/Autonomous
    FLIGHT --> DISARMED: Offboard Lost or Battery Low
    DISARMED --> STANDBY
```

---

## 4. Minimal Commander to Controller Data Flow

```mermaid
flowchart LR
    MINCMD[Minimal Commander] --> STATUS[vehicle_status]
    MINCMD --> MODE[vehicle_control_mode]
    MINCMD -->|Rate Setpoints| RATECTRL[mc_rate_control]
    RATECTRL --> ALLOC[control_allocator]
    ALLOC --> MOTORS[actuator_motors]
```

---

## 5. Minimal Commander Arming Sequence (Detailed)

```mermaid
flowchart TD
    ARMREQ[Arm Request] --> CHECKS[Run 5 Pre-Arm Checks]
    CHECKS -->|Pass| SETARM[Set ARMED State]
    CHECKS -->|Fail| REJECT[Reject Arming]
    SETARM --> PUB[Publish vehicle_status]
    SETARM --> PUBMODE[Publish vehicle_control_mode]
    PUB --> RATECTRL
    REJECT --> [*]
```

---

## 6. Minimal Commander Offboard/Autonomous Handling

```mermaid
flowchart TD
    ARMED[Armed] --> OFFBOARD[Offboard Mode]
    ARMED --> AUTONOMOUS[Autonomous Mode (if implemented)]
    OFFBOARD --> RATECTRL[mc_rate_control]
    AUTONOMOUS --> RATECTRL
    RATECTRL --> MOTORS[Motor Outputs]
```
