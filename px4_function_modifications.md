# PX4 Function Modifications for Arming Without Position and Attitude Controllers

This document provides a detailed analysis of each function in the PX4 codebase that requires modification to enable arming the multicopter even after disabling the position and attitude controllers.

---

## **1. Commander Module**

### File: `src/modules/commander/Commander.cpp`

#### **Function: `arm()`**
- **Purpose**: Handles the arming process.
- **Required Modifications**:
  - Remove dependencies on position and attitude controllers.
  - Ensure the function does not fail due to missing modules.

#### **Function: `run()`**
- **Purpose**: Main loop of the commander module.
- **Required Modifications**:
  - Bypass calls to `check_valid_state()` and `check_failsafe()`.

#### **Function: `check_valid_state()`**
- **Purpose**: Verifies if the system is in a valid state to arm.
- **Required Modifications**:
  - Remove checks for position and attitude controllers.

#### **Function: `check_failsafe()`**
- **Purpose**: Checks for failsafe conditions.
- **Required Modifications**:
  - Remove failsafe conditions related to position and attitude controllers.

---

## **2. Preflight Checks**

### File: `src/modules/commander/PreflightCheck.cpp`

#### **Function: `preflightCheck()`**
- **Purpose**: Performs all preflight checks.
- **Required Modifications**:
  - Remove or bypass checks for position and attitude controllers.

#### **Function: `checkPositionControl()`**
- **Purpose**: Verifies the position control module.
- **Required Modifications**:
  - Ensure the function returns success even if the module is disabled.

#### **Function: `checkAttitudeControl()`**
- **Purpose**: Verifies the attitude control module.
- **Required Modifications**:
  - Ensure the function returns success even if the module is disabled.

---

## **3. EKF2 (Estimator)**

### File: `src/modules/ekf2/EKF2.cpp`

#### **Function: `update()`**
- **Purpose**: Main loop of the estimator.
- **Required Modifications**:
  - Ensure the function does not block arming due to missing position or attitude data.

#### **Function: `is_healthy()`**
- **Purpose**: Checks if the estimator is providing valid data.
- **Required Modifications**:
  - Replace with a minimal implementation that always returns true.

---

## **4. Failsafe Mechanisms**

### File: `src/modules/commander/Commander.cpp`

#### **Function: `check_failsafe()`**
- **Purpose**: Checks for failsafe conditions.
- **Required Modifications**:
  - Remove failsafe conditions related to position and attitude controllers.

#### **Function: `handle_failsafe()`**
- **Purpose**: Handles failsafe events.
- **Required Modifications**:
  - Ensure the system does not enter failsafe mode due to missing modules.

---

## **5. Actuator Outputs**

### File: `src/modules/actuator_outputs/ActuatorOutputs.cpp`

#### **Function: `update()`**
- **Purpose**: Sends commands to motors/servos.
- **Required Modifications**:
  - Ensure the function does not depend on position or attitude controllers.

#### **Function: `check_actuators()`**
- **Purpose**: Verifies the functionality of actuators.
- **Required Modifications**:
  - Ensure the function does not fail due to missing modules.

---

## **6. uORB Messaging**

### File: `src/modules/uORB/uORB.cpp`

#### **Function: `publish()`**
- **Purpose**: Publishes messages to topics.
- **Required Modifications**:
  - Remove or bypass subscriptions to topics published by position and attitude controllers.

#### **Function: `subscribe()`**
- **Purpose**: Subscribes to topics.
- **Required Modifications**:
  - Remove or bypass subscriptions to topics published by position and attitude controllers.

---

## **7. Arm Authorization**

### File: `src/modules/commander/Arming/ArmAuthorization/ArmAuthorization.cpp`

#### **Function: `_auth_method_arm_req_check()`**
- **Purpose**: Checks arming requirements.
- **Required Modifications**:
  - Bypass dependencies on EKF validation.

#### **Function: `_auth_method_two_arm_check()`**
- **Purpose**: Additional arming checks.
- **Required Modifications**:
  - Ensure arming proceeds without position or attitude data.

---

## **8. Necessary Modifications Summary**

### **Commander Module**
1. **File**: `src/modules/commander/Commander.cpp`
   - Modify `arm()` to bypass dependencies on position and attitude controllers.
   - Update `check_valid_state()` to remove checks for position and attitude controllers.
   - Adjust `check_failsafe()` to ignore failsafe conditions related to position and attitude controllers.
   - Ensure `VEHICLE_CMD_RUN_PREARM_CHECKS` does not fail due to EKF validation.

### **Preflight Checks**
2. **File**: `src/modules/commander/PreflightCheck.cpp`
   - Remove or bypass checks for position control (`checkPositionControl()`).
   - Remove or bypass checks for attitude control (`checkAttitudeControl()`).
   - Ensure `preflightCheck()` returns success even if position and attitude controllers are disabled.

### **EKF2 (Estimator)**
3. **File**: `src/modules/ekf2/EKF2.cpp`
   - Modify `is_healthy()` to always return true or implement a minimal state validation.
   - Ensure `update()` does not block arming due to missing position or attitude data.

### **Failsafe Mechanisms**
4. **File**: `src/modules/commander/Commander.cpp`
   - Remove failsafe conditions in `check_failsafe()` related to position and attitude controllers.
   - Ensure `handle_failsafe()` does not trigger failsafe mode due to missing modules.

### **Actuator Outputs**
5. **File**: `src/modules/actuator_outputs/ActuatorOutputs.cpp`
   - Ensure `update()` does not depend on position or attitude controllers.
   - Modify `check_actuators()` to ignore dependencies on these modules.

### **uORB Messaging**
6. **File**: `src/modules/uORB/uORB.cpp`
   - Remove or bypass subscriptions to topics published by position and attitude controllers.
   - Ensure `publish()` and `subscribe()` do not fail due to missing topics.

### **Arm Authorization**
7. **File**: `src/modules/commander/Arming/ArmAuthorization/ArmAuthorization.cpp`
   - Modify `_auth_method_arm_req_check()` to bypass dependencies on EKF validation.
   - Adjust `_auth_method_two_arm_check()` to ensure arming proceeds without position or attitude data.

---

### **Testing and Validation**
- Use SITL (Software-In-The-Loop) simulation to test the modified system.
- Verify that the vehicle can arm and respond to commands without position and attitude controllers.
- Ensure stability and safety in all flight modes.

---
