import time
from .models import calculate_distance

class CentralizedOptimalRunner:
    """
    Globally Optimal Branch-and-Bound solver for Search & Rescue Operations.
    Evaluates execution using a topological clock to precisely enforce 
    deadlines and causality (predecessors) across the multi-UAV fleet.
    """
    def __init__(self, verbose=True):
        self.verbose = verbose
        
    def evaluate_schedule(self, routes, uavs):
        """
        Simulates the global execution of all tasks in the current routes.
        Resolves the forward timeline based on dependencies and travel distance.
        Returns: (finish_times_dict, sum_of_completion_times)
                 or (None, infinity) if deadlock or deadline missed
        """
        finish_times = {}
        uav_ready = {i: 0 for i in range(len(uavs))}
        uav_loc = {i: uavs[i].location for i in range(len(uavs))}
        route_idx = {i: 0 for i in range(len(uavs))}
        
        while True:
            progress = False
            for i in range(len(uavs)):
                if route_idx[i] < len(routes[i]):
                    task = routes[i][route_idx[i]]
                    
                    # 1. Dependency Check
                    deps_met = True
                    max_dep_finish = 0
                    for pred in task.predecessors:
                        if pred not in finish_times:
                            deps_met = False
                            break
                        max_dep_finish = max(max_dep_finish, finish_times[pred])
                    
                    if deps_met:
                        # 2. Compute Start Time
                        dist = calculate_distance(uav_loc[i], task.location)
                        arrival = uav_ready[i] + (dist / uavs[i].velocity)
                        
                        # The UAV can't start until it arrives AND all predecessors are done
                        start_time = max(arrival, max_dep_finish)
                        finish_time = start_time + task.duration
                        
                        # 3. Hard Deadline Check
                        if finish_time > task.deadline:
                            return None, float('inf') # Infeasible branch
                            
                        # Apply to state
                        finish_times[task.id] = finish_time
                        uav_ready[i] = finish_time
                        uav_loc[i] = task.location
                        route_idx[i] += 1
                        progress = True
            
            if not progress:
                break
                
        # 4. Deadlock Detection
        total_assigned = sum(len(r) for r in routes)
        if len(finish_times) < total_assigned:
            # Not all tasks were completed (e.g., Circular dependency due to route ordering)
            return None, float('inf')
            
        cost = sum(finish_times.values())
        return finish_times, cost

    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting SAR Optimal Solver...")
            print(f"Problem Size: {len(uavs)} UAVs, {len(tasks)} Tasks")
            
        start_time = time.time()
        
        # We sort tasks by ID so they are evaluated strictly in topological order.
        # This guarantees when we insert Task J, its dependencies (Task I) are already
        # present somewhere in the partial routes.
        sorted_tasks = sorted(tasks, key=lambda t: t.id)
        num_tasks = len(sorted_tasks)
        
        self.best_cost = float('inf')
        self.best_routes = None
        self.nodes_visited = 0
        self.pruned = 0
        
        initial_routes = [[] for _ in uavs]
        
        def backtrack(task_idx, current_routes):
            self.nodes_visited += 1
            if self.nodes_visited % 50000 == 0 and self.verbose:
                 print(f"  Visited {self.nodes_visited} states... (Current Best Score: {self.best_cost:.2f})")
                 
            # Evaluate partial branch cost (Sum of completions increases monotonically)
            _, partial_cost = self.evaluate_schedule(current_routes, uavs)
            
            # Prune Cost Bound: If partial branch already costs more than best, stop.
            if partial_cost >= self.best_cost:
                self.pruned += 1
                return
                
            # Base Case: All tasks successfully assigned in this branch
            if task_idx == num_tasks:
                if partial_cost < self.best_cost:
                    self.best_cost = partial_cost
                    self.best_routes = [r[:] for r in current_routes] # Deep copy
                return
                
            task_to_assign = sorted_tasks[task_idx]
            
            # Try exploring every valid insertion point in every UAV's route
            for u_idx in range(len(uavs)):
                route = current_routes[u_idx]
                for k in range(len(route) + 1):
                    # Insert the task temporarily
                    current_routes[u_idx].insert(k, task_to_assign)
                    
                    # Quickly check if the resulting topology is valid before recursing
                    _, test_cost = self.evaluate_schedule(current_routes, uavs)
                    if test_cost < float('inf'):
                        backtrack(task_idx + 1, current_routes)
                    else:
                        self.pruned += 1 # Pruned due to deadline or deadlock
                        
                    # Backtrack (Remove it so we can try the next slot)
                    current_routes[u_idx].pop(k)
                    
        # Kick off recursive search
        backtrack(0, initial_routes)
        
        exec_time = time.time() - start_time
        
        if self.verbose:
            print(f"--- SAR OPTIMAL SEARCH FINISHED ---")
            print(f"  Time taken       : {exec_time:.3f} seconds")
            print(f"  Nodes Navigated  : {self.nodes_visited}")
            print(f"  Branches Pruned  : {self.pruned}")
            
            if self.best_routes:
                print(f"  GLOBAL OPTIMAL COST : {self.best_cost:.2f}")
                print(f"  Optimal Schedule Routes:")
                for i, r in enumerate(self.best_routes):
                    print(f"    UAV {uavs[i].id} Route -> {[t.id for t in r]}")
            else:
                print("\n  [!] INFEASIBLE OPERATION [!]")
                print("  No combination of routes could satisfy all dependencies and deadlines.")
                
        # Commit the optimal schedule directly to the UAV objects
        if self.best_routes:
             for i, uav in enumerate(uavs):
                 uav.tasks = self.best_routes[i]
                 
        return {
            "exec_time": exec_time,
            "cost": self.best_cost,
            "solved": self.best_routes is not None,
            "routes": self.best_routes
        }
