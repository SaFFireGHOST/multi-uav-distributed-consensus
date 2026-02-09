
import random
import time
from shared.models import Task
from algs.datw.uav import UAV

def run_simulation(num_uavs=3, num_tasks=10, max_iterations=100, verbose=True, uav_class=UAV):
    if verbose:
        print(f"Starting simulation with {num_uavs} UAVs and {num_tasks} Tasks...")
    
    # 1. Setup Scenario
    # 100x100x100 space
    tasks = []
    for i in range(num_tasks):
        loc = (random.randint(0, 100), random.randint(0, 100), 0)
        duration = random.randint(5, 15)
        # Random time windows
        start = random.randint(0, 50)
        end = start + random.randint(20, 50) 
        tasks.append(Task(i, loc, duration, (start, end)))
        
    uavs = []
    for i in range(num_uavs):
        loc = (random.randint(0, 100), random.randint(0, 100), 0)
        velocity = 5 # m/s
        capacity = 5 # max tasks
        fuel = 1000
        rate = 0.5
        uavs.append(uav_class(i, loc, velocity, capacity, fuel, rate, tasks))

    # 2. Main Loop
    # Repeat Phase 1 & 2 until convergence
    converged = False
    start_time = time.time()
    
    for it in range(max_iterations):
        if verbose:
            print(f"--- Iteration {it} ---")
        
        # Phase 1: Task Inclusion (All UAVs run independently)
        any_inclusion = False
        for uav in uavs:
            if uav.task_inclusion():
                any_inclusion = True
                
        # Phase 2: Conflict Resolution
        # All communicate with all (Mesh)
        # We simulate this by having everyone read everyone's state
        # In reality, this is async. Here we do synchronous rounds.
        
        # Communication Round
        # Everyone sends to everyone
        # We repeatedly exchange until consensus in this phase?
        # Paper says: "Phase 2 is conducted to obtain a conflict-free allocation... repeated alternately... until global consensus"
        # So we have inner loop for Phase 2.
        
        phase2_converged = False
        p2_it = 0
        while not phase2_converged and p2_it < 20:
            changes = False
            
            # Consensus Stage
            for u in uavs:
                for neighbor in uavs:
                    if u.id == neighbor.id: continue
                    
                    info = neighbor.communicate(u)
                    if u.update_state(info, time.time()):
                        changes = True
                        
            # Task Removal Stage
            for u in uavs:
                if u.task_removal():
                    changes = True
            
            if not changes:
                phase2_converged = True
            p2_it += 1
            
        if verbose:
            print(f"Phase 2 converged in {p2_it} sub-iterations.")
        
        # Check global convergence (No changes in allocation Z across all UAVs)
        # Verify conflict-free:
        assignment_map = {}
        conflict = False
        for u in uavs:
            for t in u.tasks:
                if t.id in assignment_map:
                    if verbose:
                        print(f"Conflict detected! Task {t.id} assigned to {assignment_map[t.id]} and {u.id}")
                    conflict = True
                assignment_map[t.id] = u.id
                
        if not any_inclusion and not changes and not conflict:
            if verbose:
                print("Global Convergence Reached!")
            converged = True
            break
            
            
    # Phase 3: Task Reallocation
    if verbose:
        print("\n--- Starting Phase 3 (Reallocation) ---")
    
    # Repeat Phase 3 until convergence
    p3_converged = False
    p3_max_it = 20
    
    for it3 in range(p3_max_it):
        if verbose:
            print(f"Phase 3 Iteration {it3}")
        
        # Secondary Inclusion
        any_inclusion_p3 = False
        for uav in uavs:
            if uav.secondary_inclusion():
                any_inclusion_p3 = True
                
        # Secondary Conflict Resolution (Reuse Phase 2 logic)
        phase3_cr_converged = False
        p3_cr_it = 0
        while not phase3_cr_converged and p3_cr_it < 20:
            changes = False
            # Consensus
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
            if verbose:
                print("Phase 3 Converged.")
            break
            
    if verbose:
        print(f"Phase 3 finished in {it3+1} iterations.")
    
    end_time = time.time()
    if verbose:
        print(f"Simulation finished in {end_time - start_time:.2f}s")
    
    # 3. Validation & Output
    total_cost = 0
    assigned_count = 0
    for u in uavs:
        if verbose:
            print(f"UAV {u.id} tasks: {[t.id for t in u.tasks]}")
        cost = u.calculate_local_time_cost(u.tasks)
        total_cost += cost
        assigned_count += len(u.tasks)
        
        # Verify Time Windows
        times = u._calculate_times_for_sequence(u.tasks)
        for i, t in enumerate(u.tasks):
            start_t = times[i]
            valid = t.time_window[0] <= start_t <= t.time_window[1]
            if verbose:
                print(f"  Task {t.id}: Start {start_t:.2f} Window {t.time_window} Valid: {valid}")

    if verbose:
        print(f"Total Global Cost (Objective): {total_cost:.2f}")
        print(f"Total Assigned Tasks: {assigned_count}/{num_tasks}")
    
    return {
        "num_uavs": num_uavs,
        "num_tasks": num_tasks,
        "assigned_tasks": assigned_count,
        "total_cost": total_cost,
        "execution_time": end_time - start_time,
        "converged": converged or (not any_inclusion_p3 and phase3_cr_converged), # Approximate convergence check
        "success": assigned_count == num_tasks,
        "uavs": uavs, # Return objects for visualization
        "tasks": tasks
    }

if __name__ == "__main__":
    import sys
    from algs.datw.uav import UAV
    try:
        from algs.baseline.uav import BaselineUAV
    except ImportError:
        BaselineUAV = None
    try:
        from algs.cbba.uav import CBBAUAV
    except ImportError:
        CBBAUAV = None

    # Simple CLI argument to enable visualization and choose algorithm
    visualize = "--visualize" in sys.argv
    use_baseline = "--baseline" in sys.argv
    use_cbba = "--cbba" in sys.argv
    
    # Check for seed argument
    seed = None
    for arg in sys.argv:
        if arg.startswith("--seed="):
            seed = int(arg.split("=")[1])
            break
            
    if seed is not None:
        print(f"Running with seed {seed}")
        random.seed(seed)
    
    uav_cls = UAV
    if use_baseline:
        if BaselineUAV:
             print("Running with Baseline Algorithm...")
             uav_cls = BaselineUAV
        else:
             print("BaselineUAV not found, defaulting to DATW")
    elif use_cbba:
        if CBBAUAV:
            print("Running with CBBA Algorithm...")
            uav_cls = CBBAUAV
        else:
            print("CBBAUAV not found, defaulting to DATW")
    else:
        print("Running with DATW Algorithm...")
    
    res = run_simulation(verbose=True, uav_class=uav_cls)
    
    if visualize:
        from visualizer import visualize_simulation
        visualize_simulation(res['uavs'], res['tasks'])
