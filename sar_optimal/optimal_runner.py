import time
import multiprocessing
import ctypes
from .models import calculate_distance

# Global variables for worker processes
global_best = None
global_lock = None

def init_worker(shared_val, shared_lck):
    global global_best
    global global_lock
    global_best = shared_val
    global_lock = shared_lck

def evaluate_schedule(routes, uavs):
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

def worker_backtrack(task_idx, current_routes, uavs, sorted_tasks):
    num_tasks = len(sorted_tasks)
    local_best_cost = float('inf')
    local_best_routes = None
    nodes_visited = 0
    pruned = 0
    
    def backtrack(t_idx, c_routes):
        nonlocal local_best_cost, local_best_routes, nodes_visited, pruned
        nodes_visited += 1
        
        # Check global shared best cost
        gb_cost = global_best.value
        
        _, partial_cost = evaluate_schedule(c_routes, uavs)
        
        if partial_cost >= gb_cost or partial_cost >= local_best_cost:
            pruned += 1
            return
            
        if t_idx == num_tasks:
            if partial_cost < local_best_cost:
                local_best_cost = partial_cost
                local_best_routes = [r[:] for r in c_routes]
                with global_lock:
                    if partial_cost < global_best.value:
                        global_best.value = partial_cost
            return
            
        task_to_assign = sorted_tasks[t_idx]
        
        for u_idx in range(len(uavs)):
            route = c_routes[u_idx]
            for k in range(len(route) + 1):
                c_routes[u_idx].insert(k, task_to_assign)
                _, test_cost = evaluate_schedule(c_routes, uavs)
                
                if test_cost < float('inf'):
                    backtrack(t_idx + 1, c_routes)
                else:
                    pruned += 1
                    
                c_routes[u_idx].pop(k)
                
    backtrack(task_idx, current_routes)
    return local_best_cost, local_best_routes, nodes_visited, pruned

class CentralizedOptimalRunner:
    """
    Globally Optimal Branch-and-Bound solver for Search & Rescue Operations.
    Evaluates execution using a topological clock to precisely enforce 
    deadlines and causality (predecessors) across the multi-UAV fleet.
    """
    def __init__(self, verbose=True):
        self.verbose = verbose
        
    def evaluate_schedule(self, routes, uavs):
        return evaluate_schedule(routes, uavs)

    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting SAR Optimal Solver...")
            print(f"Problem Size: {len(uavs)} UAVs, {len(tasks)} Tasks")
            
        start_time = time.time()
        
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
                 
            _, partial_cost = self.evaluate_schedule(current_routes, uavs)
            
            if partial_cost >= self.best_cost:
                self.pruned += 1
                return
                
            if task_idx == num_tasks:
                if partial_cost < self.best_cost:
                    self.best_cost = partial_cost
                    self.best_routes = [r[:] for r in current_routes]
                return
                
            task_to_assign = sorted_tasks[task_idx]
            
            for u_idx in range(len(uavs)):
                route = current_routes[u_idx]
                for k in range(len(route) + 1):
                    current_routes[u_idx].insert(k, task_to_assign)
                    
                    _, test_cost = self.evaluate_schedule(current_routes, uavs)
                    if test_cost < float('inf'):
                        backtrack(task_idx + 1, current_routes)
                    else:
                        self.pruned += 1
                        
                    current_routes[u_idx].pop(k)
                    
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
                
        if self.best_routes:
             for i, uav in enumerate(uavs):
                 uav.tasks = self.best_routes[i]
                 
        return {
            "exec_time": exec_time,
            "cost": self.best_cost,
            "solved": self.best_routes is not None,
            "routes": self.best_routes
        }

class ParallelCentralizedOptimalRunner:
    """
    Parallel Centralized Optimal - True Global Upper Bound.
    
    Implement's a Recursive Branch-and-Bound search across multiple processes using multiprocessing.
    """
    _pool = None
    _shared_val = None
    _shared_lock = None

    @classmethod
    def get_pool(cls, num_workers):
        if cls._pool is None:
            cls._shared_val = multiprocessing.Value(ctypes.c_double, float('inf'))
            cls._shared_lock = multiprocessing.Lock()
            # We'll need to specify spawn or fork safely depending on OS, but Python handles it.
            cls._pool = multiprocessing.Pool(
                processes=num_workers,
                initializer=init_worker,
                initargs=(cls._shared_val, cls._shared_lock)
            )
        return cls._pool, cls._shared_val, cls._shared_lock

    @classmethod
    def close_pool(cls):
        if cls._pool:
            cls._pool.close()
            cls._pool.join()
            cls._pool = None
            cls._shared_val = None
            cls._shared_lock = None

    def __init__(self, verbose=True, num_workers=2):
        self.verbose = verbose
        self.num_workers = num_workers
        
    def evaluate_schedule(self, routes, uavs):
        return evaluate_schedule(routes, uavs)
    
    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting Parallel SAR Optimal Solver...")
            print(f"Problem Size: {len(uavs)} UAVs, {len(tasks)} Tasks")
            print(f"Workers: {self.num_workers}")
            
        pool, shared_val, shared_lock = self.get_pool(self.num_workers)
        shared_val.value = float('inf')
        
        start_time = time.time()
        
        sorted_tasks = sorted(tasks, key=lambda t: t.id)
        num_tasks = len(sorted_tasks)
        
        initial_states = []
        
        def generate_initial_states(t_idx, c_routes):
            depth_to_branch = min(3, num_tasks)
            if t_idx == depth_to_branch: 
                initial_states.append((t_idx, [r[:] for r in c_routes]))
                return
            
            task_to_assign = sorted_tasks[t_idx]
            for u_idx in range(len(uavs)):
                route = c_routes[u_idx]
                for k in range(len(route) + 1):
                    c_routes[u_idx].insert(k, task_to_assign)
                    
                    _, test_cost = self.evaluate_schedule(c_routes, uavs)
                    if test_cost < float('inf'):
                        generate_initial_states(t_idx + 1, c_routes)
                        
                    c_routes[u_idx].pop(k)

        generate_initial_states(0, [[] for _ in uavs])
        
        if self.verbose:
            print(f"Generated {len(initial_states)} initial states for parallel workers.")
        
        worker_args = []
        for state in initial_states:
            worker_args.append((state[0], state[1], uavs, sorted_tasks))
            
        global_best_cost = float('inf')
        global_best_routes = None
        total_nodes = 0
        total_pruned = 0
        
        if len(worker_args) > 0:
            results = pool.starmap(worker_backtrack, worker_args)
            
            for res_cost, res_routes, res_nodes, res_pruned in results:
                total_nodes += res_nodes
                total_pruned += res_pruned
                if res_cost < global_best_cost:
                    global_best_cost = res_cost
                    global_best_routes = res_routes
                        
        exec_time = time.time() - start_time
        
        if self.verbose:
            print(f"--- PARALLEL SAR OPTIMAL SEARCH FINISHED ---")
            print(f"  Time taken       : {exec_time:.3f} seconds")
            print(f"  Nodes Navigated  : {total_nodes}")
            print(f"  Branches Pruned  : {total_pruned}")
            
            if global_best_routes:
                print(f"  GLOBAL OPTIMAL COST : {global_best_cost:.2f}")
                print(f"  Optimal Schedule Routes:")
                for i, r in enumerate(global_best_routes):
                    print(f"    UAV {uavs[i].id} Route -> {[t.id for t in r]}")
            else:
                print("\n  [!] INFEASIBLE OPERATION [!]")
                print("  No combination of routes could satisfy all dependencies and deadlines.")
                
        if global_best_routes:
            for i, uav in enumerate(uavs):
                uav.tasks = global_best_routes[i]
                
        return {
            "exec_time": exec_time,
            "cost": global_best_cost,
            "solved": global_best_routes is not None,
            "routes": global_best_routes
        }
