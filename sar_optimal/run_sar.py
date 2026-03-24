import sys
import os
import random

# Ensure absolute imports work from root dir
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sar_optimal.models import Task, UAV
from sar_optimal.optimal_runner import CentralizedOptimalRunner

def generate_random_sar_tasks(num_tasks, seed=None):
    """
    Safely generates random tasks with valid temporal limits and dependencies.
    """
    if seed is not None:
        random.seed(seed)
        
    tasks = []
    # Track the absolute fastest a task could finish (assuming 0 travel time)
    # This prevents us from generating mathematically impossible deadlines.
    min_finish_times = {} 
    
    for i in range(1, num_tasks + 1):
        loc = (random.randint(0, 100), random.randint(0, 100))
        dur = random.randint(2, 15)
        
        # 1. Assign causal dependencies (Only depend on older tasks to prevent circular paradoxes)
        preds = []
        if i > 1 and random.random() < 0.5: # 50% chance to depend on a prior task
            pred_id = random.randint(1, i - 1)
            preds.append(pred_id)
            
        # 2. Calculate the absolute fastest this specific task can finish
        max_pred_finish = 0
        for p in preds:
            max_pred_finish = max(max_pred_finish, min_finish_times[p])
        theoretical_min_finish = max_pred_finish + dur
        min_finish_times[i] = theoretical_min_finish
        
        # 3. Apply optional random deadlines (Only ~30% of tasks get a strict deadline constraint)
        deadline = float('inf')
        if random.random() < 0.3:
            # We add 'slack' to the minimum finish time to ensure it is actually solvable
            slack = random.randint(15, 60)
            deadline = round(theoretical_min_finish + slack, 1)
            
        tasks.append(Task(task_id=i, location=loc, duration=dur, deadline=deadline, predecessors=preds))
        
    return tasks


def main():
    import argparse
    parser = argparse.ArgumentParser(description="SAR Optimal Simulation")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for task generation")
    args = parser.parse_args()

    print("=== INITIALIZING SAR SIMULATION ===")
    
    # 1. Initialize Drone Fleet
    uavs = [
        UAV(uav_id=1, location=(0, 0), velocity=10),
        UAV(uav_id=2, location=(50, 50), velocity=15)
    ]
    
    # 2. Define Mission Tasks (SAR Constraints Example)
    num_tasks = 5
    seed_text = f" (Seed: {args.seed})" if args.seed is not None else ""
    print(f"Generating {num_tasks} Random Tasks with Dependencies{seed_text}...")
    tasks = generate_random_sar_tasks(num_tasks, seed=args.seed)
    
    # Print constraints 
    for t in tasks:
        print(f"  > {t}")
        
    print("\n")
    
    # 3. Execute Optimal Solver
    runner = CentralizedOptimalRunner(verbose=True)
    results = runner.run(uavs, tasks)
    
    # 4. Display the resulting Global Timeline
    if results['solved']:
        print("\n=== FINAL EXECUTION TIMELINE ===")
        # Re-evaluate the best routes to get exact timestamps
        finish_times, total_cost = runner.evaluate_schedule([u.tasks for u in uavs], uavs)
        
        # Sort chronologically by start time
        global_timeline = []
        for task in tasks:
            start_time = finish_times[task.id] - task.duration
            end_time = finish_times[task.id]
            global_timeline.append((task.id, start_time, end_time))
            
        global_timeline.sort(key=lambda x: x[1]) # Sort by start time
        
        for t_id, start, finish in global_timeline:
            # Find which UAV did it
            uav_assigned = None
            for u in uavs:
                if any(t.id == t_id for t in u.tasks):
                    uav_assigned = u.id
                    break
                    
            task_ref = next(t for t in tasks if t.id == t_id)
            print(f" [T={start:05.1f} -> {finish:05.1f}] UAV {uav_assigned} | Task {t_id:<2} (deps: {task_ref.predecessors})")

if __name__ == "__main__":
    main()
