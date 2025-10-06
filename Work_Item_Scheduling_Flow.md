# PX4 Work Item Scheduling Flow Documentation

## Overview

This document demonstrates how a new work item gets scheduled in the PX4 work queue system when there are already items in the queue. The analysis is based on the actual PX4 source code functions and provides detailed flow diagrams showing the complete scheduling process.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Work Item Types](#work-item-types)
3. [Scheduling Scenarios](#scheduling-scenarios)
4. [Detailed Flow Analysis](#detailed-flow-analysis)
5. [Code References](#code-references)
6. [Performance Considerations](#performance-considerations)

## System Architecture

The PX4 work queue system consists of several key components:

```mermaid
graph TB
    subgraph "PX4 Work Queue System"
        WQM[WorkQueueManager]
        WQ1[WorkQueue: hp_default]
        WQ2[WorkQueue: lp_default]
        WQ3[WorkQueue: rate_ctrl]

        WI1[WorkItem: Sensor Driver]
        WI2[WorkItem: Controller]
        WI3[WorkItem: Estimator]
        SWI1[ScheduledWorkItem: Periodic Task]

        HRT[High Resolution Timer]
    end

    WQM --> WQ1
    WQM --> WQ2
    WQM --> WQ3

    WI1 --> WQ1
    WI2 --> WQ3
    WI3 --> WQ2
    SWI1 --> WQ1

    HRT --> SWI1

    style WQM fill:#e1f5fe
    style WQ1 fill:#f3e5f5
    style WQ2 fill:#f3e5f5
    style WQ3 fill:#f3e5f5
    style HRT fill:#fff3e0
```

## Work Item Types

### 1. Regular WorkItem
- **Purpose**: One-time or manual scheduling
- **Key Function**: `ScheduleNow()`
- **File**: `WorkItem.hpp` (Line 65-68)

### 2. ScheduledWorkItem
- **Purpose**: Periodic or timed execution
- **Key Functions**: `ScheduleDelayed()`, `ScheduleOnInterval()`, `ScheduleAt()`
- **File**: `ScheduledWorkItem.hpp` (Lines 50-70)

## Scheduling Scenarios

### Scenario: Adding a New Work Item to an Active Queue

**Initial State:**
- WorkQueue "hp_default" is running with 2 items already queued
- Worker thread is currently executing an item
- New item needs to be scheduled immediately

## Detailed Flow Analysis

### Phase 1: Work Item Scheduling Request

```mermaid
sequenceDiagram
    participant Module as Module/Driver
    participant WI as New WorkItem
    participant WQ as WorkQueue
    participant WT as Worker Thread
    participant Queue as Execution Queue

    Note over Module, Queue: Phase 1: Scheduling Request

    Module->>WI: Call ScheduleNow()
    Note right of WI: WorkItem.hpp Line 65-68

    WI->>WI: Check if _wq != nullptr

    alt WorkQueue is valid
        WI->>WQ: Call _wq->Add(this)
        Note right of WQ: WorkQueue.cpp Line 125
    else WorkQueue is null
        WI-->>Module: Silent failure (no queue attached)
    end
```

### Phase 2: Work Queue Addition Process

```mermaid
sequenceDiagram
    participant WI as New WorkItem
    participant WQ as WorkQueue
    participant Lock as work_lock()
    participant Queue as _q (IntrusiveQueue)
    participant LS as Lockstep Scheduler
    participant WT as Worker Thread

    Note over WI, WT: Phase 2: Queue Addition Process

    WQ->>Lock: Acquire work_lock()
    Note right of Lock: WorkQueue.cpp Line 126

    opt Lockstep Scheduler Enabled
        WQ->>LS: Register component if not registered
        Note right of LS: WorkQueue.cpp Lines 130-131
    end

    WQ->>Queue: _q.push(new_item)
    Note right of Queue: WorkQueue.cpp Line 136

    WQ->>Lock: Release work_unlock()
    Note right of Lock: WorkQueue.cpp Line 137

    WQ->>WT: SignalWorkerThread()
    Note right of WT: WorkQueue.cpp Line 139
```

### Phase 3: Worker Thread Signaling

```mermaid
sequenceDiagram
    participant WQ as WorkQueue
    participant Sem as _process_lock
    participant WT as Worker Thread

    Note over WQ, WT: Phase 3: Worker Thread Signaling

    WQ->>Sem: px4_sem_getvalue(&_process_lock, &sem_val)
    Note right of Sem: WorkQueue.cpp Line 147

    alt Semaphore value <= 0 (Thread waiting)
        WQ->>Sem: px4_sem_post(&_process_lock)
        Note right of Sem: WorkQueue.cpp Line 148
        Sem-->>WT: Wake up worker thread
    else Semaphore value > 0 (Thread busy)
        Note right of WQ: No signal needed, thread will process queue
    end
```

### Phase 4: Worker Thread Processing

```mermaid
sequenceDiagram
    participant WT as Worker Thread
    participant Sem as _process_lock
    participant Lock as work_lock()
    participant Queue as _q (IntrusiveQueue)
    participant WI1 as Existing Item 1
    participant WI2 as Existing Item 2
    participant WI3 as New Item

    Note over WT, WI3: Phase 4: Worker Thread Processing

    WT->>Sem: px4_sem_wait(&_process_lock)
    Note right of Sem: WorkQueue.cpp Line 173

    WT->>Lock: Acquire work_lock()
    Note right of Lock: WorkQueue.cpp Line 175

    loop Process all queued items
        WT->>Queue: Check if !_q.empty()
        Note right of Queue: WorkQueue.cpp Line 178

        alt Queue has items
            WT->>Queue: work = _q.pop()
            Note right of Queue: WorkQueue.cpp Line 179 (FIFO order)

            WT->>Lock: Release work_unlock()
            Note right of Lock: WorkQueue.cpp Line 181

            par Execute Current Item
                WT->>WI1: work->RunPreamble()
                Note right of WI1: WorkQueue.cpp Line 182
                WT->>WI1: work->Run()
                Note right of WI1: WorkQueue.cpp Line 183
            end

            WT->>Lock: Acquire work_lock()
            Note right of Lock: WorkQueue.cpp Line 185
        else Queue is empty
            Note right of WT: Exit processing loop
        end
    end

    WT->>Lock: Release work_unlock()
    Note right of Lock: WorkQueue.cpp Line 197
```

### Complete System State Flow

```mermaid
stateDiagram-v2
    [*] --> Idle: Worker Thread Waiting

    state "Queue Processing" as Processing {
        [*] --> CheckQueue: Wake Up
        CheckQueue --> ExecuteItem: Items Available
        CheckQueue --> WaitForWork: Queue Empty
        ExecuteItem --> RunPreamble: Pop Item (FIFO)
        RunPreamble --> RunWorkItem: Update Statistics
        RunWorkItem --> CheckQueue: Item Complete
        WaitForWork --> [*]: Sleep on Semaphore
    }

    state "New Item Addition" as Addition {
        [*] --> AcquireLock: ScheduleNow() Called
        AcquireLock --> AddToQueue: Lock Acquired
        AddToQueue --> ReleaseLock: Item Pushed (FIFO)
        ReleaseLock --> SignalWorker: Lock Released
        SignalWorker --> [*]: Semaphore Posted
    }

    Idle --> Processing: Semaphore Posted
    Processing --> Idle: No More Items
    Addition --> Processing: Worker Signaled

    note right of Addition
        New items are added to the tail
        of the FIFO queue and processed
        in order of arrival
    end note

    note right of Processing
        Worker thread processes items
        one by one in FIFO order.
        Each item can requeue itself
        during execution.
    end note
```

## Code References

### Key Functions and Their Locations

| Function | File | Lines | Purpose |
|----------|------|-------|---------|
| `WorkItem::ScheduleNow()` | `WorkItem.hpp` | 65-68 | Entry point for immediate scheduling |
| `WorkQueue::Add()` | `WorkQueue.cpp` | 125-142 | Add item to execution queue |
| `WorkQueue::SignalWorkerThread()` | `WorkQueue.cpp` | 144-149 | Wake worker thread |
| `WorkQueue::Run()` | `WorkQueue.cpp` | 170-201 | Main worker thread loop |
| `ScheduledWorkItem::ScheduleDelayed()` | `ScheduledWorkItem.cpp` | 51-54 | Schedule with timer |

### Data Structures

1. **IntrusiveQueue<WorkItem *> _q**
   - **Type**: FIFO queue
   - **Location**: `WorkQueue.hpp` Line 90
   - **Purpose**: Stores work items waiting for execution

2. **px4_sem_t _process_lock**
   - **Type**: Semaphore (counting)
   - **Location**: `WorkQueue.hpp` Line 91
   - **Purpose**: Worker thread synchronization

3. **List<WorkItem *> _work_items**
   - **Type**: List of attached items
   - **Purpose**: Track all items attached to this queue

## Thread Safety Analysis

### Critical Sections

```mermaid
graph LR
    subgraph "Thread Safety Mechanisms"
        A[work_lock/work_unlock] --> B[Critical Section Entry/Exit]
        C[_process_lock Semaphore] --> D[Worker Thread Synchronization]
        E[_exit_lock Semaphore] --> F[Shutdown Coordination]
    end

    subgraph "Protected Operations"
        G[Queue Addition/Removal]
        H[Item Attachment/Detachment]
        I[Statistics Updates]
    end

    A --> G
    A --> H
    C --> I
```

### Lock Hierarchy
1. **work_lock()** - Protects queue operations
2. **_process_lock** - Coordinates worker thread
3. **_exit_lock** - Manages shutdown synchronization

## Performance Considerations

### Timing Analysis

| Operation | Complexity | Time Cost |
|-----------|------------|-----------|
| Add to Queue | O(1) | ~1-2 μs |
| Signal Worker | O(1) | ~0.5-1 μs |
| Pop from Queue | O(1) | ~1-2 μs |
| Context Switch | O(1) | ~5-10 μs |

### Real-Time Characteristics

1. **Deterministic Scheduling**: FIFO order ensures predictable execution
2. **Priority Inheritance**: Work queues have different priorities
3. **Lockstep Support**: Optional deterministic simulation support
4. **Minimal Latency**: Direct semaphore signaling reduces delays

## Scheduling Examples

### Example 1: Sensor Driver Scheduling

```cpp
// From a sensor driver module
class SensorDriver : public WorkItem {
public:
    SensorDriver() : WorkItem("sensor_driver", wq_configurations::hp_default) {}

    void trigger_read() {
        ScheduleNow();  // Adds to hp_default queue immediately
    }

    void Run() override {
        // Read sensor data
        // Process data
        // Publish to uORB
    }
};
```

### Example 2: Periodic Controller

```cpp
// From a control module
class AttitudeController : public ScheduledWorkItem {
public:
    AttitudeController() :
        ScheduledWorkItem("mc_att_control", wq_configurations::rate_ctrl) {}

    void start() {
        ScheduleOnInterval(4000);  // 250 Hz = 4000 μs interval
    }

    void Run() override {
        // Read attitude estimates
        // Compute control outputs
        // Send actuator commands
    }
};
```

## Queue State Visualization

### Before New Item Addition
```
WorkQueue "hp_default" State:
┌─────────────────────────────────────┐
│ Worker Thread: [EXECUTING Item_A]  │
│                                     │
│ Execution Queue: [Item_B] [Item_C]  │
│                  ↑ head   ↑ tail    │
│                                     │
│ Attached Items: [Item_A, Item_B,   │
│                  Item_C, Item_D]    │
└─────────────────────────────────────┘
```

### After New Item Addition
```
WorkQueue "hp_default" State:
┌─────────────────────────────────────┐
│ Worker Thread: [EXECUTING Item_A]  │
│                                     │
│ Execution Queue: [Item_B] [Item_C]  │
│                           [Item_D]  │ ← New item added
│                  ↑ head   ↑ tail    │
│                                     │
│ Attached Items: [Item_A, Item_B,   │
│                  Item_C, Item_D]    │
└─────────────────────────────────────┘
```

## Conclusion

The PX4 work queue scheduling system provides:

1. **Thread-Safe Operations**: All queue modifications are protected by locks
2. **FIFO Ordering**: Items execute in the order they were scheduled
3. **Efficient Signaling**: Minimal overhead worker thread coordination
4. **Real-Time Guarantees**: Deterministic behavior for time-critical operations
5. **Scalable Design**: Multiple work queues with different priorities

The scheduling flow ensures that new work items are seamlessly integrated into the execution pipeline without disrupting ongoing operations, maintaining the real-time characteristics essential for flight control systems.
