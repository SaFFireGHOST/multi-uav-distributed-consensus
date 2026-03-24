import time
import multiprocessing
import ctypes
from shared.simulation_utils import evaluate_metrics

# Global variables for worker processes
global_best = None
global_lock = None

def init_worker(shared_val, shared_lck):
    global global_best
    global global_lock
    global_best = shared_val
    global_lock = shared_lck

def worker_backtrack(task_idx, current_routes, current_total_cost, uavs, sorted_tasks):
    num_tasks = len(sorted_tasks)
    local_best_cost = float('inf')
    local_best_routes = None
    nodes_visited = 0
    pruned = 0
    
    def backtrack(t_idx, c_routes, c_cost):
        nonlocal local_best_cost, local_best_routes, nodes_visited, pruned
        nodes_visited += 1
        
        # Check global shared best cost (fast read)
        # Periodically reading gb_cost instead of every iteration can save a tiny bit,
        # but with multiprocessing.Value (shared memory) it's virtually as fast as a local var.
        gb_cost = global_best.value
        
        # Pruning 1: Cost Bound
        if c_cost >= gb_cost or c_cost >= local_best_cost:
            pruned += 1
            return
            
        # Base Case: All tasks assigned
        if t_idx == num_tasks:
            if c_cost < local_best_cost:
                local_best_cost = c_cost
                local_best_routes = [r[:] for r in c_routes]
                # Try to update global best
                with global_lock:
                    if c_cost < global_best.value:
                        global_best.value = c_cost
            return

        task_to_assign = sorted_tasks[t_idx]
        
        # Branching: Try inserting into every UAV, at every valid position
        for u_idx, uav in enumerate(uavs):
            route = c_routes[u_idx]
            
            for k in range(len(route) + 1):
                new_route = route[:k] + [task_to_assign] + route[k:]
                
                # Check Feasibility
                times = uav._calculate_times_for_sequence(new_route)
                feasible = True
                for i, t in enumerate(new_route):
                    if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                        feasible = False
                        break
                
                if not feasible:
                    continue
                    
                # Calculate New Cost
                new_uav_cost = 0
                for i, t in enumerate(new_route):
                    new_uav_cost += (times[i] + t.duration)
                    
                old_uav_cost = 0
                if route:
                    old_times = uav._calculate_times_for_sequence(route)
                    for i, t in enumerate(route):
                        old_uav_cost += (old_times[i] + t.duration)
                
                new_total_cost = c_cost - old_uav_cost + new_uav_cost
                
                # Further pruning check
                if new_total_cost >= global_best.value:
                     continue

                c_routes[u_idx].insert(k, task_to_assign)
                backtrack(t_idx + 1, c_routes, new_total_cost)
                c_routes[u_idx].pop(k) # Backtrack

    backtrack(task_idx, current_routes, current_total_cost)
    return local_best_cost, local_best_routes, nodes_visited, pruned


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
    
    def run(self, uavs, tasks):
        if self.verbose:
            print(f"Starting Parallel Centralized Optimal (Exact Branch-and-Bound)...")
            print(f"Problem Size: {len(uavs)} UAVs, {len(tasks)} Tasks")
            print(f"Workers: {self.num_workers}")
            
        # Initialize pool and reset shared best cost
        pool, shared_val, shared_lock = self.get_pool(self.num_workers)
        shared_val.value = float('inf')
        
        start_time = time.time()
        
        sorted_tasks = sorted(tasks, key=lambda t: t.id)
        num_tasks = len(sorted_tasks)
        
        initial_states = []
        
        def generate_initial_states(t_idx, c_routes, c_cost):
            # Target generating states after branching on the first 3 tasks minimum
            # Depth 2 yields ~12 states. Depth 3 yields ~60 states, which is better for 8+ workers.
            depth_to_branch = min(3, num_tasks)
            if t_idx == depth_to_branch: 
                initial_states.append((t_idx, [r[:] for r in c_routes], c_cost))
                return
            
            task_to_assign = sorted_tasks[t_idx]
            for u_idx, uav in enumerate(uavs):
                route = c_routes[u_idx]
                for k in range(len(route) + 1):
                    new_route = route[:k] + [task_to_assign] + route[k:]
                    
                    times = uav._calculate_times_for_sequence(new_route)
                    feasible = True
                    for i, t in enumerate(new_route):
                        if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                            feasible = False
                            break
                    if not feasible:
                        continue
                        
                    new_uav_cost = sum(times[i] + t.duration for i, t in enumerate(new_route))
                    old_uav_cost = 0
                    if route:
                        old_times = uav._calculate_times_for_sequence(route)
                        old_uav_cost = sum(old_times[i] + t.duration for i, t in enumerate(route))
                    
                    new_total_cost = c_cost - old_uav_cost + new_uav_cost
                    
                    c_routes[u_idx].insert(k, task_to_assign)
                    generate_initial_states(t_idx + 1, c_routes, new_total_cost)
                    c_routes[u_idx].pop(k)

        generate_initial_states(0, [[] for _ in uavs], 0)
        
        if self.verbose:
            print(f"Generated {len(initial_states)} initial states for parallel workers.")
        
        worker_args = []
        for state in initial_states:
            worker_args.append((state[0], state[1], state[2], uavs, sorted_tasks))
            
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
                        
        end_time = time.time()
        exec_time = end_time - start_time
        
        if self.verbose:
            print(f"  Exact Optimal Found in {exec_time:.3f}s")
            print(f"  Nodes: {total_nodes}, Pruned: {total_pruned}")
            if global_best_routes:
                print(f"  Best Cost: {global_best_cost:.2f}")
            else:
                print("  No feasible solution found.")
                
        if global_best_routes:
            for i, uav in enumerate(uavs):
                uav.tasks = global_best_routes[i]
                
        return evaluate_metrics(uavs, tasks, exec_time)
