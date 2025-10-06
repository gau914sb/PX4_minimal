# ArduPilot vs PX4: Communication Architecture Comparison

## Executive Summary

ArduPilot and PX4 represent two fundamentally different approaches to autopilot software architecture, particularly in how modules communicate and share data. This analysis compares ArduPilot's **singleton-based direct access pattern** with PX4's **uORB publish-subscribe messaging system**.

## 1. ArduPilot Communication Architecture

### 1.1 Singleton Pattern with Global Access

ArduPilot uses a **singleton pattern** where major subsystems are accessible globally through the `AP::` namespace:

```cpp
// Global access functions in namespace AP
AP::ahrs()      // Access AHRS singleton
AP::logger()    // Access logging singleton
AP::arming()    // Access arming singleton
AP::gps()       // Access GPS singleton
```

### 1.2 Direct Object Access Pattern

Modules directly call methods on other modules:

```cpp
// Direct method calls on AHRS singleton
float roll = ahrs.get_roll_rad();
Vector3f velocity;
bool success = ahrs.get_velocity_NED(velocity);
ahrs.get_relative_position_D_home(altitude_above_home_m);

// Direct logging calls
AP::logger().WriteStreaming("FHLD", "TimeUS,SFx,SFy,Ax,Ay", data);
AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
```

### 1.3 Header-Based Coupling

Modules include specific headers and directly instantiate objects:

```cpp
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors.h>

// Direct access to member variables and methods
const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
AP::logger().set_vehicle_armed(true);
```

### 1.4 AP_HAL Hardware Abstraction Layer

ArduPilot uses AP_HAL for hardware abstraction:

```cpp
// Hardware abstraction through HAL
const HAL& hal = AP_HAL::get_HAL();
hal.scheduler->delay(100);
hal.gpio->pinMode(pin, OUTPUT);
```

## 2. PX4 Communication Architecture

### 2.1 uORB Publish-Subscribe System

PX4 uses **uORB (micro Object Request Broker)** for all inter-module communication:

```cpp
// Publishers create topics
orb_advert_t attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);

// Subscribers listen to topics
int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude_data);
```

### 2.2 Message-Based Decoupling

Modules communicate only through standardized message types:

```cpp
// Attitude data structure
struct vehicle_attitude_s {
    uint64_t timestamp;
    float q[4];        // Quaternion
    float rollspeed;   // Roll angular velocity
    float pitchspeed;  // Pitch angular velocity
    float yawspeed;    // Yaw angular velocity
};
```

### 2.3 Work Queue Scheduling

PX4 integrates communication with its work queue system:

```cpp
class ModuleBase : public px4::ScheduledWorkItem {
    void Run() override {
        // Check for new messages
        if (_param_update_sub.updated()) {
            parameter_update_s param_update;
            _param_update_sub.copy(&param_update);
            // Handle parameter update
        }

        // Publish results
        _attitude_pub.publish(attitude_data);
    }
};
```

## 3. Architecture Comparison

| Aspect | ArduPilot | PX4 |
|--------|-----------|-----|
| **Communication Pattern** | Direct object access | Publish-subscribe messaging |
| **Coupling** | Tight (header dependencies) | Loose (message contracts) |
| **Global State** | Singletons with global access | No global state, only messages |
| **Thread Safety** | Semaphores + careful design | Lock-free atomic operations |
| **Performance** | Direct calls (faster) | Message passing (overhead) |
| **Testing** | Harder (global dependencies) | Easier (mock message queues) |
| **Modularity** | Lower (tight coupling) | Higher (loose coupling) |
| **Real-time Guarantees** | Depends on careful coding | Built-in (work queue priority) |

## 4. Detailed Technical Analysis

### 4.1 ArduPilot Singleton Implementation

```cpp
// AP_AHRS singleton pattern
class AP_AHRS {
private:
    static AP_AHRS *_singleton;
    HAL_Semaphore _rsem;  // Thread safety through semaphores

public:
    static AP_AHRS *get_singleton() {
        return _singleton;
    }

    // Direct access with semaphore protection
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }

    // Direct data access
    const Vector3f &get_gyro(void) const { return state.gyro_estimate; }
    float get_roll_rad() const { return roll; }
};

// Global namespace access
namespace AP {
    AP_AHRS &ahrs() { return *AP_AHRS::get_singleton(); }
    AP_Logger &logger() { return *AP_Logger::get_singleton(); }
}
```

### 4.2 PX4 uORB Implementation

```cpp
// uORB topic definition
ORB_DEFINE(vehicle_attitude, struct vehicle_attitude_s);

// Publisher
class AttitudeEstimator : public ModuleBase {
private:
    uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};

public:
    void Run() override {
        vehicle_attitude_s attitude{};
        attitude.timestamp = hrt_absolute_time();
        // ... fill attitude data
        _attitude_pub.publish(attitude);
    }
};

// Subscriber
class FlightController : public ModuleBase {
private:
    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};

public:
    void Run() override {
        vehicle_attitude_s attitude;
        if (_attitude_sub.update(&attitude)) {
            // Use attitude data for control
            float roll = attitude.q[0]; // Access quaternion
        }
    }
};
```

### 4.3 Thread Safety Comparison

**ArduPilot:** Uses semaphores and careful design:
```cpp
// Thread-safe access in ArduPilot
void get_attitude_data() {
    ahrs.get_semaphore().take_blocking();
    float roll = ahrs.get_roll_rad();
    float pitch = ahrs.get_pitch_rad();
    ahrs.get_semaphore().give();
}
```

**PX4:** Lock-free atomic operations:
```cpp
// Atomic operations in uORB
bool uORB::Subscription::update(void *dst) {
    if (!copy(dst)) {
        return false;
    }
    return true;  // Atomic copy operation
}
```

### 4.4 Performance Characteristics

**ArduPilot Direct Access:**
- **Pros:** Zero overhead function calls, immediate data access
- **Cons:** Potential cache misses, semaphore overhead

**PX4 Message Passing:**
- **Pros:** Predictable memory patterns, lock-free operations
- **Cons:** Memory allocation/copy overhead, indirection

## 5. Real-World Usage Examples

### 5.1 ArduPilot Flight Mode Implementation

```cpp
// ArduCopter mode implementation
void ModeRTL::run() {
    // Direct access to multiple singletons
    if (!ahrs.get_location(current_loc)) {
        return;
    }

    // Direct method calls for navigation
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned)) {
        float speed = vel_ned.length();
    }

    // Direct logging
    AP::logger().WriteStreaming("RTL", "TimeUS,State,Lat,Lng",
                               AP_HAL::micros64(), state,
                               current_loc.lat, current_loc.lng);

    // Direct motor control
    attitude_control->input_euler_angle_roll_pitch_yaw(
        target_roll, target_pitch, target_yaw, true);
}
```

### 5.2 PX4 Flight Mode Implementation

```cpp
// PX4 flight mode implementation
class MulticopterLandDetector : public ModuleBase {
private:
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
    uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};

public:
    void Run() override {
        // Check for new data
        vehicle_status_s vehicle_status;
        if (_vehicle_status_sub.update(&vehicle_status)) {
            _armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
        }

        // Publish land detection result
        vehicle_land_detected_s land_detected{};
        land_detected.timestamp = hrt_absolute_time();
        land_detected.landed = _is_landed();
        _land_detected_pub.publish(land_detected);
    }
};
```

## 6. System Integration Patterns

### 6.1 ArduPilot Vehicle Startup

```cpp
void Copter::setup() {
    // Initialize singletons in order
    hal.console->printf("Initializing ArduCopter\n");

    // All modules access shared singletons
    ahrs.init();
    attitude_control = new AC_AttitudeControl_Multi(ahrs);
    pos_control = new AC_PosControl(ahrs, inertial_nav);

    // Direct parameter access
    g.throttle_min.load();
    g.throttle_max.load();
}
```

### 6.2 PX4 System Startup

```cpp
// PX4 module registration and message routing
int px4_main(int argc, char *argv[]) {
    // Start modules independently
    attitude_estimator_q_main();
    mc_pos_control_main();
    navigator_main();

    // uORB handles all communication automatically
    px4_daemon_main();
}
```

## 7. Error Handling and Debugging

### 7.1 ArduPilot Error Propagation

```cpp
// Errors propagated through return values
bool success = ahrs.get_location(loc);
if (!success) {
    gcs().send_text(MAV_SEVERITY_ERROR, "AHRS: Location invalid");
    return;
}

// Global error state through singletons
if (!ahrs.healthy()) {
    AP::logger().Write_Error(LogErrorSubsystem::AHRS,
                           LogErrorCode::ERROR_RESOLVED);
}
```

### 7.2 PX4 Distributed Error Handling

```cpp
// Errors handled per-message basis
void handle_sensor_data() {
    sensor_combined_s sensor_data;
    if (_sensor_sub.update(&sensor_data)) {
        if (sensor_data.timestamp == 0) {
            // Handle invalid data locally
            _sensor_error_count++;
            return;
        }
        // Process valid data
    }
}
```

## 8. Testing and Simulation Implications

### 8.1 ArduPilot Testing Challenges

```cpp
// Difficult to mock global singletons
class TestFlightMode : public Test {
    void SetUp() override {
        // Must initialize entire system
        ahrs.init();
        AP::logger().init();
        // Cannot easily inject mock dependencies
    }
};
```

### 8.2 PX4 Testing Advantages

```cpp
// Easy to inject mock message sources
class MockAttitudeProvider : public ModuleBase {
public:
    void inject_test_attitude(const vehicle_attitude_s &attitude) {
        _attitude_pub.publish(attitude);
    }
private:
    uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
};
```

## 9. Performance and Resource Analysis

### 9.1 Memory Usage Patterns

| Metric | ArduPilot | PX4 |
|--------|-----------|-----|
| **RAM Overhead** | Lower (shared state) | Higher (message buffers) |
| **Code Size** | Larger (linked libraries) | Smaller (modular linking) |
| **Stack Usage** | Variable (deep call chains) | Predictable (message-driven) |
| **Heap Fragmentation** | Higher (dynamic objects) | Lower (fixed message pools) |

### 9.2 CPU Performance

| Operation | ArduPilot | PX4 |
|-----------|-----------|-----|
| **Data Access** | Direct (0 overhead) | Copy + validate |
| **Function Calls** | Direct vtable | Message dispatch |
| **Context Switches** | Fewer (shared state) | More (message queues) |
| **Cache Performance** | Variable | Better (locality) |

## 10. Evolution and Maintenance

### 10.1 ArduPilot Evolution Challenges

- **Breaking Changes:** Singleton interface changes affect all modules
- **Dependencies:** Adding new data requires modifying multiple singletons
- **Versioning:** Difficult to maintain API compatibility

### 10.2 PX4 Evolution Advantages

- **Interface Stability:** Message contracts provide clear versioning
- **Module Independence:** New modules can be added without changing existing ones
- **Backward Compatibility:** Old message versions can coexist with new ones

## 11. Conclusions and Recommendations

### 11.1 ArduPilot Strengths
- **Performance:** Direct access provides minimal overhead
- **Simplicity:** Straightforward calling patterns
- **Maturity:** Proven in thousands of deployments
- **Integration:** Easy to add quick features

### 11.2 PX4 Strengths
- **Modularity:** Clean separation of concerns
- **Testing:** Excellent testability through message mocking
- **Real-time:** Built-in priority and timing guarantees
- **Safety:** Isolated failure domains

### 11.3 Use Case Recommendations

**Choose ArduPilot when:**
- Performance is critical
- Simple integration needed
- Working with legacy systems
- Rapid prototyping required

**Choose PX4 when:**
- System safety is paramount
- Extensive testing needed
- Modular development preferred
- Real-time guarantees required

### 11.4 Hybrid Approach Possibilities

Future autopilot architectures might combine both approaches:
- High-frequency control loops use direct access (ArduPilot style)
- System-level coordination uses message passing (PX4 style)
- Critical functions isolated through message queues
- Non-critical functions use direct access for performance

## 12. Technical Implementation Details

### 12.1 Message vs Direct Access Performance Benchmark

```cpp
// ArduPilot: ~5 CPU cycles
float roll = ahrs.get_roll_rad();

// PX4: ~50-100 CPU cycles
vehicle_attitude_s attitude;
_attitude_sub.update(&attitude);
float roll = atan2f(2.0f*(attitude.q[0]*attitude.q[1] + attitude.q[2]*attitude.q[3]),
                    1.0f - 2.0f*(attitude.q[1]*attitude.q[1] + attitude.q[2]*attitude.q[2]));
```

### 12.2 Memory Layout Comparison

**ArduPilot Singleton Memory:**
```
[AHRS Instance] -> [Shared State] <- [All Modules]
                   - attitude data
                   - position data
                   - sensor data
```

**PX4 Message Buffer Memory:**
```
[Module A] -> [Message Queue] -> [Module B]
[Module B] -> [Message Queue] -> [Module C]
[Module C] -> [Message Queue] -> [Module A]
```

This comprehensive analysis demonstrates that both ArduPilot and PX4 have made fundamental architectural trade-offs that reflect different priorities: ArduPilot optimizes for performance and simplicity, while PX4 optimizes for modularity and safety. Understanding these differences is crucial for developers working with either platform or considering which approach to adopt for new autopilot projects.
