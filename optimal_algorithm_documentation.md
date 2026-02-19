# Optimal Centralized Algorithm - Documentation

## 1. Overview

The **Optimal Centralized Algorithm** serves as a baseline for evaluating the performance of decentralized heuristic algorithms like DATW. Unlike DATW, which runs on individual UAVs with limited information, the centralized optimal algorithm has **perfect global knowledge** and runs on a single central computer.

It is designed to find the **True Optimal Solution** (the global minimum cost) for the assignment problem.

*   **Goal**: Minimize total mission time (Assignment Cost).
*   **Constraints**:
    *   Each task must be assigned to exactly one UAV.
    *   Time windows `[Early, Late]` must be respected.
    *   UAV capacity constraints (if applicable, though often just time/fuel).
*   **Method**: **Branch-and-Bound (Recursive Backtracking)**.

---

## 2. Algorithm Logic

The problem is a variation of the **Multiple Traveling Salesperson Problem with Time Windows (mTSP-TW)**, which is NP-Hard. To solve it exactly, we must explore the solution space.

### 2.1 The Search Tree

The algorithm builds a decision tree where each level represents the assignment of **one task**.

*   **Level 0 (Root)**: No tasks assigned.
*   **Level K**: Task K is assigned.

**Nodes and Branches**:
Yes, at each step, the algorithm creates a **separate branch (node)** for every possible valid placement of the current task.

**Example**:
Suppose we have 2 UAVs with some existing tasks:
*   **UAV 1**: Has 2 tasks `[A, B]`
*   **UAV 2**: Has 1 task `[C]`

Now we need to assign **Task D** (The next level). The algorithm will create **5 branches** (child nodes) from this state:

1.  **UAV 1, Position 0**: (Result: `[D, A, B]`, `[C]`)
2.  **UAV 1, Position 1**: (Result: `[A, D, B]`, `[C]`)
3.  **UAV 1, Position 2**: (Result: `[A, B, D]`, `[C]`)
4.  **UAV 2, Position 0**: (Result: `[A, B]`, `[D, C]`)
5.  **UAV 2, Position 1**: (Result: `[A, B]`, `[C, D]`)

### 2.2 Branch-and-Bound Strategy

The algorithm uses Depth-First Search (DFS) on this tree:

1.  **Ordering**: Tasks are assigned in a fixed order.
2.  **Pruning (The "Bound")**:
    *   **Feasibility**: If an insertion violates a time window, that branch is **pruned** (abandoned immediately).
    *   **Cost Bound**: We track the `best_cost` found so far. If the cost of the *partial* solution we are building already exceeds `best_cost`, we **prune** that branch.
3.  **Backtracking**: When a branch is explored (or pruned), the algorithm "backtracks" (undoes the last insertion) and tries the next option.

### 2.3 Complexity

*   **Worst Case**: Factorial complexity `O((N+M)! / M!)`.
*   **Practical Performance**: Thanks to **pruning**, it is much faster than naive brute force, but it still scales poorly. It is generally feasible for small scenarios (N=10-15 tasks), which is sufficient for benchmarking.

---

## 3. Code Implementation

The implementation is split into the runner (logic) and the UAV model (data/helper).

### 3.1 The Solver (`algs/runners.py`)

The core logic resides in `CentralizedOptimalRunner`.

```python
# algs/runners.py

class CentralizedOptimalRunner:
    """
    Centralized Optimal - True Global Upper Bound.
    Implement's a Recursive Branch-and-Bound search to find the
    globally minimum cost assignment.
    """
    def __init__(self, verbose=True):
        self.verbose = verbose
    
    def run(self, uavs, tasks):
        # Setup
        sorted_tasks = sorted(tasks, key=lambda t: t.id)
        num_tasks = len(sorted_tasks)
        
        self.best_cost = float('inf')
        self.best_routes = None 
        initial_routes = [[] for _ in uavs]
        
        # --- Recursive Solver ---
        def backtrack(task_idx, current_routes, current_total_cost):
            # 1. Pruning: Cost Bound
            if current_total_cost >= self.best_cost:
                return

            # 2. Base Case: All tasks assigned
            if task_idx == num_tasks:
                if current_total_cost < self.best_cost:
                    self.best_cost = current_total_cost
                    self.best_routes = [r[:] for r in current_routes] # Save best
                return

            task_to_assign = sorted_tasks[task_idx]
            
            # 3. Branching: Try every UAV, every position
            for u_idx, uav in enumerate(uavs):
                route = current_routes[u_idx]
                
                # Check every position k in the route
                for k in range(len(route) + 1):
                    # Create trial route
                    new_route = route[:k] + [task_to_assign] + route[k:]
                    
                    # 4. Check Feasibility (Time Windows)
                    times = uav._calculate_times_for_sequence(new_route)
                    feasible = True
                    for i, t in enumerate(new_route):
                        if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                            feasible = False
                            break
                    
                    if not feasible:
                        continue
                        
                    # 5. Calculate New Cost
                    # Cost = Sum(Completion Times)
                    new_uav_cost = sum([times[i] + t.duration for i, t in enumerate(new_route)])
                    
                    # Subtract old cost of this UAV to get delta
                    old_uav_cost = 0
                    if route:
                         old_times = uav._calculate_times_for_sequence(route)
                         old_uav_cost = sum([old_times[i] + t.duration for i, t in enumerate(route)])
                    
                    new_total_cost = current_total_cost - old_uav_cost + new_uav_cost
                    
                    # 6. Recurse & Backtrack
                    current_routes[u_idx].insert(k, task_to_assign) # Apply
                    backtrack(task_idx + 1, current_routes, new_total_cost) # Recurse
                    current_routes[u_idx].pop(k) # Undo (Backtrack)
        
        # Start Search
        backtrack(0, initial_routes, 0)
        
        # Apply Best Solution found to UAVs
        if self.best_routes:
            for i, uav in enumerate(uavs):
                uav.tasks = self.best_routes[i]
        
        return evaluate_metrics(uavs, tasks, exec_time)
```

### 3.2 The Data Model (`algs/optimal/uav.py`)

A minimal class used to calculate times and costs for a given sequence.

```python
# algs/optimal/uav.py

class OptimalUAV:
    def _calculate_times_for_sequence(self, sequence):
        """
        Calculates the arrival and start times for a sequence of tasks.
        Respects:
          - Travel time (Distance / Velocity)
          - Early Start Time (Time Windows)
        """
        times = []
        curr_loc = self.location
        curr_time = 0
        
        for task in sequence:
            dist = calculate_distance(curr_loc, task.location)
            arrival = curr_time + (dist / self.velocity)
            # Wait if we arrive too early
            start_time = max(arrival, task.time_window[0])
            times.append(start_time)
            
            # Next task starts after current one finishes
            curr_time = start_time + task.duration
            curr_loc = task.location
            
        return times
```

---

## 4. How it Works: Step-by-Step Example

Imagine **2 UAVs** and **3 Tasks**.

1.  **Start**: `best_cost = infinity`. Routes: `UAV1: [], UAV2: []`.
2.  **Task 1**:
    *   Try putting Task 1 in UAV1.
        *   Feasible? Yes. Cost = 10.
        *   Recurse to Task 2.
    *   Try putting Task 1 in UAV2.
        *   Feasible? Yes. Cost = 12 (maybe UAV2 is further away).
        *   Recurse to Task 2.

3.  **Deep in the Tree (Hypothetical)**:
    *   We have a partial solution: `UAV1: [T1, T3], UAV2: []`. Total Cost = 25.
    *   We try to add **Task 2**.
    *   **Option A**: Insert T2 into UAV1.
        *   Route becomes `[T1, T2, T3]`.
        *   Check time windows. If T2 pushes T3 too late -> **Time Window Violation!** -> **PRUNE** (Stop, go back).
    *   **Option B**: Insert T2 into UAV2.
        *   Route becomes `UAV2: [T2]`.
        *   Cost becomes 25 + 15 = 40.
        *   Suppose we already found a full solution earlier with cost **38**.
        *   Since 40 > 38, we **PRUNE** (Stop, go back).

4.  **Completion**:
    *   The algorithm finishes checking all valid combinations.
    *   The configuration stored in `best_routes` is guaranteed to be the global minimum.
