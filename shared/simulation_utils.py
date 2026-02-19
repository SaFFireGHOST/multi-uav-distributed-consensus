
import random
from shared.models import Task

def setup_scenario(num_uavs=3, num_tasks=10, seed=None):
    """
    Sets up the simulation scenario with tasks and UAV configs.
    Returns: (tasks, uav_configs)
    """
    if seed is not None:
        random.seed(seed)

    # 1. Setup Tasks
    # 100x100x100 space
    tasks = []
    for i in range(num_tasks):
        loc = (random.randint(0, 100), random.randint(0, 100), 0)
        duration = random.randint(5, 15)
        # Random time windows
        start = random.randint(0, 50)
        end = start + random.randint(20, 50) 
        tasks.append(Task(i, loc, duration, (start, end)))
        
    # 2. Setup UAV Configurations (not the objects, just the params)
    uav_configs = []
    for i in range(num_uavs):
        loc = (random.randint(0, 100), random.randint(0, 100), 0)
        velocity = 5 # m/s
        capacity = 5 # max tasks
        fuel = 1000
        rate = 0.5
        uav_configs.append({
            'id': i,
            'loc': loc,
            'velocity': velocity,
            'capacity': capacity,
            'fuel': fuel,
            'rate': rate
        })
        
    return tasks, uav_configs

def evaluate_metrics(uavs, tasks, execution_time, additional_info=None):
    """
    Calculates global metrics for the simulation result.
    """
    total_cost = 0
    assigned_count = 0
    assignment_map = {}
    conflict = False
    
    # 1. Calculate Costs and Check Conflicts
    for u in uavs:
        # Calculate local cost (Time based)
        # Note: UAV class should have this method, or we use a shared utility
        # Assuming UAV class has `calculate_local_time_cost` and `tasks` attribute
        cost = u.calculate_local_time_cost(u.tasks)
        total_cost += cost
        assigned_count += len(u.tasks)
        
        for t in u.tasks:
            if t.id in assignment_map:
                conflict = True
            assignment_map[t.id] = u.id

    # 2. Success Check
    # Check Feasibility (Time Windows)
    valid_assignments = True
    for u in uavs:
        times = u._calculate_times_for_sequence(u.tasks)
        for i, t in enumerate(u.tasks):
            if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                valid_assignments = False
                break
        if not valid_assignments:
            break
            
    success = (assigned_count == len(tasks)) and (not conflict) and valid_assignments
    
    results = {
        "num_uavs": len(uavs),
        "num_tasks": len(tasks),
        "assigned_tasks": assigned_count,
        "total_cost": total_cost,
        "execution_time": execution_time,
        "success": success,
        "valid": valid_assignments,
        "conflict": conflict,
        "uavs": uavs,
        "tasks": tasks
    }
    
    if additional_info:
        results.update(additional_info)
        
    return results
