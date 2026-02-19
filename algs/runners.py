
import time
from shared.simulation_utils import evaluate_metrics

class ConsensusRunner:
    """
    Runs the distributed consensus algorithm (CBBA/DATW).
    Phases:
    1. Task Inclusion (Bundle Building)
    2. Conflict Resolution (Consensus)
    3. Reallocation (optional, for DATW/Advanced CBBA)
    """
    def __init__(self, max_iterations=100, verbose=True):
        self.max_iterations = max_iterations
        self.verbose = verbose

    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting Consensus Simulation with {len(uavs)} UAVs...")
        
        start_time = time.time()
        converged = False
        
        # --- Main Loop ---
        for it in range(self.max_iterations):
            if self.verbose:
                print(f"--- Iteration {it} ---")
            
            # Phase 1: Task Inclusion
            any_inclusion = False
            for uav in uavs:
                if uav.task_inclusion():
                    any_inclusion = True
            
            # Phase 2: Conflict Resolution (Consensus & Removal)
            # We simulate synchronous communication rounds
            phase2_converged = False
            p2_it = 0
            while not phase2_converged and p2_it < 20: # Inner convergence loop
                changes = False
                
                # Communicate & Update
                # All-to-All communication
                for u in uavs:
                    for neighbor in uavs:
                        if u.id == neighbor.id: continue
                        info = neighbor.communicate(u)
                        if u.update_state(info, time.time()):
                            changes = True
                
                # Task Removal
                for u in uavs:
                    if u.task_removal():
                        changes = True
                
                if not changes:
                    phase2_converged = True
                p2_it += 1
            
            if self.verbose:
                print(f"Phase 2 converged in {p2_it} sub-iterations.")

            # Check Global Convergence
            # If no inclusions and no changes in consensus, we are stable.
            # (Strictly speaking, we should check if Z is consistent across fleet, but stable behavior is a good proxy)
            # Just to be safe, we check for consistency in evaluate_metrics later.
                
            if not any_inclusion and phase2_converged:
                if self.verbose:
                    print("Global Convergence Reached (Phase 1 & 2 stable).")
                converged = True
                break
        
        # --- Phase 3: Reallocation ---
        # Some algorithms (like raw CBBA) might not do this, but the provided simulation.py did.
        # We can make this optional or flag-controlled? 
        # For now, we include it as it was in the original DATW implementation.
        if self.verbose:
            print("\n--- Starting Phase 3 (Reallocation) ---")
            
        p3_converged = False
        for it3 in range(20): # Max 20 iterations for reallocation
            if self.verbose:
                 print(f"Phase 3 Iteration {it3}")
            
            # Secondary Inclusion
            any_inclusion_p3 = False
            for uav in uavs:
                if hasattr(uav, 'secondary_inclusion'):
                    if uav.secondary_inclusion():
                        any_inclusion_p3 = True
            
            # Secondary Conflict Resolution
            phase3_cr_converged = False
            p3_cr_it = 0
            while not phase3_cr_converged and p3_cr_it < 20:
                changes = False
                for u in uavs:
                    for neighbor in uavs:
                        if u.id == neighbor.id: continue
                        info = neighbor.communicate(u)
                        if u.update_state(info, time.time()):
                            changes = True
                
                 # Removal
                for u in uavs:
                    if u.task_removal():
                        changes = True
                        
                if not changes:
                    phase3_cr_converged = True
                p3_cr_it += 1
            
            if not any_inclusion_p3 and phase3_cr_converged:
                if self.verbose:
                    print("Phase 3 Converged.")
                break

        end_time = time.time()
        exec_time = end_time - start_time
        
        # Additional Convergence Info
        results = evaluate_metrics(uavs, tasks, exec_time)
        results['converged'] = converged
        
        return results


class BaselineRunner:
    """
    Runs a simplified baseline algorithm.
    Currently, the Baseline UAV is implemented as a Distributed Agent (shares Z/Q).
    So it technically works with ConsensusRunner too.
    
    However, if we want a 'Centralized Greedy' baseline (no communication overhead, perfect info):
    """
    def __init__(self, verbose=True):
        self.verbose = verbose
        
    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting Baseline (Sequential Greedy) Simulation...")
            
        start_time = time.time()
        
        # Simple Sequential Greedy Assignment
        # Each UAV picks its best tasks until full, considering what others ALREADY took.
        # This requires shared knowledge of assignments (Centralized).
        
        assigned_task_ids = set()
        
        # Iterate round-robin until no one can take more tasks
        updated = True
        while updated:
            updated = False
            for uav in uavs:
                if len(uav.tasks) >= uav.capacity:
                    continue
                
                # Find best task that is NOT in assigned_task_ids
                best_task = None
                min_cost = float('inf')
                
                # We need to peek at UAV's logic. 
                # UAV.calculate_marginal_significance returns cost.
                # BaselineUAV overrides this to be distance.
                
                for task in tasks:
                    if task.id in assigned_task_ids:
                        continue
                    if task in uav.tasks: 
                        continue
                        
                    # Calculate cost (Significance)
                    # BaselineUAV methods expect 'sequence'.
                    cost, pos = uav.calculate_marginal_significance(task, uav.tasks)
                    
                    if cost < min_cost:
                        min_cost = cost
                        best_task = task
                        best_pos = pos
                        
                if best_task:
                    # Assign
                    uav.tasks.insert(best_pos, best_task)
                    assigned_task_ids.add(best_task.id)
                    updated = True
        
        end_time = time.time()
        exec_time = end_time - start_time
        
        return evaluate_metrics(uavs, tasks, exec_time)


class CentralizedOptimalRunner:
    """
    Centralized Optimal - True Global Upper Bound.
    
    Implement's a Recursive Branch-and-Bound search to find the
    globally minimum cost assignment.
    
    Strategy:
    - Iterates through tasks in a fixed order.
    - For each task, tries inserting it into EVERY possible position
      in EVERY UAV's route.
    - Prunes branches that violate time windows or exceed current best cost.
    - Complexity: O((N+M)!/M!) worst case, but efficient with pruning.
    
    This guarantees finding the global minimum (Optimal Solution).
    """
    def __init__(self, verbose=True):
        self.verbose = verbose
    
    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting Centralized Optimal (Exact Branch-and-Bound)...")
            print(f"Problem Size: {len(uavs)} UAVs, {len(tasks)} Tasks")
        
        start_time = time.time()
        
        # Sort tasks by ID to have a fixed processing order
        sorted_tasks = sorted(tasks, key=lambda t: t.id)
        num_tasks = len(sorted_tasks)
        
        # Global Best Tracking
        self.best_cost = float('inf')
        self.best_routes = None # List of lists [uav_idx] -> [task_objects]
        
        # Pre-assign empty routes
        # current_routes[uav_id] = list of tasks
        initial_routes = [[] for _ in uavs]
        
        # Performance Counters
        self.nodes_visited = 0
        self.pruned = 0
        
        # Recursive Solver
        def backtrack(task_idx, current_routes, current_total_cost):
            # Performance update
            self.nodes_visited += 1
            if self.nodes_visited % 50000 == 0 and self.verbose:
                 print(f"  Visited {self.nodes_visited} states... (Best: {self.best_cost:.2f})")
            
            # Pruning 1: Cost Bound
            if current_total_cost >= self.best_cost:
                self.pruned += 1
                return

            # Base Case: All tasks assigned
            if task_idx == num_tasks:
                if current_total_cost < self.best_cost:
                    self.best_cost = current_total_cost
                    self.best_routes = [r[:] for r in current_routes] # Deep Copy
                return

            task_to_assign = sorted_tasks[task_idx]
            
            # Branching: Try inserting into every UAV, at every valid position
            for u_idx, uav in enumerate(uavs):
                route = current_routes[u_idx]
                
                # Optimization: Try positions
                # Inserting at k=0..len(route)
                for k in range(len(route) + 1):
                    # Construct new route trial
                    new_route = route[:k] + [task_to_assign] + route[k:]
                    
                    # Check Feasibility & Cost
                    # We need to recalculate cost for THIS UAV
                    # and remove the OLD cost for THIS UAV from total
                    
                    # 1. Check Feasibility (Time Windows)
                    times = uav._calculate_times_for_sequence(new_route)
                    feasible = True
                    for i, t in enumerate(new_route):
                        if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                            feasible = False
                            break
                    
                    if not feasible:
                        continue
                        
                    # 2. Calculate New Cost
                    # New UAV Cost
                    new_uav_cost = 0
                    for i, t in enumerate(new_route):
                        new_uav_cost += (times[i] + t.duration)
                        
                    # Old UAV Cost (need to calculate or track? simpler to recalc for route)
                    # Optimization: Track per-UAV cost in state?
                    # For simplicity, let's calc old cost
                    old_uav_cost = 0
                    if route:
                         old_times = uav._calculate_times_for_sequence(route)
                         for i, t in enumerate(route):
                             old_uav_cost += (old_times[i] + t.duration)
                    
                    new_total_cost = current_total_cost - old_uav_cost + new_uav_cost
                    
                    # Recurse
                    # Pass updated routes (copy)
                    # Optimization: Mutate and backtrack to save memory
                    current_routes[u_idx].insert(k, task_to_assign)
                    backtrack(task_idx + 1, current_routes, new_total_cost)
                    current_routes[u_idx].pop(k) # Backtrack
        
        # Start Search
        backtrack(0, initial_routes, 0)
        
        end_time = time.time()
        exec_time = end_time - start_time
        
        if self.verbose:
            print(f"  Exact Optimal Found in {exec_time:.3f}s")
            print(f"  Nodes: {self.nodes_visited}, Pruned: {self.pruned}")
            if self.best_routes:
                print(f"  Best Cost: {self.best_cost:.2f}")
            else:
                print("  No feasible solution found.")

        # Apply results to UAVs
        if self.best_routes:
            for i, uav in enumerate(uavs):
                uav.tasks = self.best_routes[i]
        
        return evaluate_metrics(uavs, tasks, exec_time)

