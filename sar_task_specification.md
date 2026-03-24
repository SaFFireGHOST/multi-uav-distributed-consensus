
This document defines the formal input schema for tasks in the multi-UAV distributed consensus system, specifically tailored for Search and Rescue (SAR) operations.

## Overview
The task model represents the physical and causal reality of a disaster response scenario. It strips away artificial constraints and reduces a task down to two fundamental concepts:
1. **Intrinsic Properties:** What the task is and where it is.
2. **Execution Constraints:** When it becomes a failure (time) and what must precede it (causality).

By keeping the schema minimal, we allow the optimal scheduling algorithm to organically compute the shortest possible global execution time.

---

## Task Schema Definition

Each task is defined by the following fields:

### Required Fields

These intrinsic properties must always be provided by the mission planner.

| Field | Type | Description |
| :--- | :--- | :--- |
| **`id`** | `int` | A unique identifier for the task. |
| **`location`** | `tuple(float, float)` | The (x, y) coordinates of the task in the physical environment. |
| **`duration`** | `float` | The time required for a UAV to execute the task once it arrives at the location. |

### Optional Fields (Constraints)

These fields define the structural and temporal limits of the task. Both are **optional** and should only be specified when the physical reality of the environment or the mission strictly requires them.

| Field | Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| **`predecessors`** | `list[int]` | `[]` (Empty) | A list of task IDs that **must be fully completed** before this task can begin. This represents biological/physical causality (e.g., you cannot drop a medical kit until the survivor is located). |
| **`deadline`** | `float` | `infinity` | The absolute time limit by which the task **must** finish. If the task finishes after this time, the mission fails (e.g., the island completely floods, or the sun goes down). |

---

## Code Implementation Example

When parsed by the Python backend natively, the default values are injected to ensure the optimal scheduler can perform fast, crash-free mathematical comparisons:

```python
class Task:
    def __init__(self, task_id, location, duration, deadline=float('inf'), predecessors=None):
        self.id = task_id
        self.location = location
        self.duration = duration
        
        # Constraints
        self.deadline = deadline
        self.predecessors = predecessors if predecessors is not None else []
```

## Scenario Examples

**Example A: Independent Task (No Constraints)**
A simple reconnaissance task that can be done at any point during the mission.
*   Input provided: `id=1, location=(10,10), duration=5`
*   Inferred state: Can start immediately, no deadline.

**Example B: Causality-Bound Task**
Delivering a life raft to the location scouted in Example A.
*   Input provided: `id=2, location=(10,10), duration=2, predecessors=[1]`
*   Inferred state: Must wait for Task 1 to finish, but no hard deadline on completion.

**Example C: Time-Bound Task with Causality**
Retrieving a stranded survivor from the raft before the incoming storm surge.
*   Input provided: `id=3, location=(10,10), duration=10, predecessors=[2], deadline=120`
*   Inferred state: Must wait for Task 2 to deliver the raft, and the entire retrieval must complete before `T=120`.
