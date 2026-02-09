
import random
import statistics
from simulation import run_simulation


from simulation import run_simulation
from algs.datw.uav import UAV
from algs.baseline.uav import BaselineUAV
from algs.cbba.uav import CBBAUAV

def run_experiments(num_trials=30):
    print(f"Running {num_trials} experiments for Comparison...")
    
    datw_results = []
    baseline_results = []
    cbba_results = []
    
    for i in range(num_trials):
        # random seed for reproducibility across algorithms
        seed = i
        
        # Run DATW
        random.seed(seed)
        res_datw = run_simulation(num_uavs=3, num_tasks=10, max_iterations=50, verbose=False, uav_class=UAV)
        datw_results.append(res_datw)
        
        # Run Baseline
        random.seed(seed)
        res_base = run_simulation(num_uavs=3, num_tasks=10, max_iterations=50, verbose=False, uav_class=BaselineUAV)
        baseline_results.append(res_base)

        # Run CBBA
        random.seed(seed)
        res_cbba = run_simulation(num_uavs=3, num_tasks=10, max_iterations=50, verbose=False, uav_class=CBBAUAV)
        cbba_results.append(res_cbba)
        
        print(f"Trial {i}: DATW Cost {res_datw['total_cost']:.2f} ({res_datw['assigned_tasks']}/{res_datw['num_tasks']}) | CBBA Cost {res_cbba['total_cost']:.2f} ({res_cbba['assigned_tasks']}/{res_cbba['num_tasks']}) | Base Cost {res_base['total_cost']:.2f} ({res_base['assigned_tasks']}/{res_base['num_tasks']})")

    # Calculate metrics
    def get_metrics(results):
        success_count = sum(1 for r in results if r['success'])
        avg_cost = statistics.mean([r['total_cost'] for r in results])
        avg_assigned = statistics.mean([r['assigned_tasks'] for r in results])
        return success_count, avg_cost, avg_assigned

    s_datw, c_datw, a_datw = get_metrics(datw_results)
    s_base, c_base, a_base = get_metrics(baseline_results)
    s_cbba, c_cbba, a_cbba = get_metrics(cbba_results)
    
    print("\n--- Comparative Results ---")
    print(f"Trials: {num_trials}")
    print(f"Metrics             | DATW        | CBBA        | Baseline")
    print(f"--------------------|-------------|-------------|-------------")
    print(f"Success Rate        | {s_datw/num_trials*100:.2f}%      | {s_cbba/num_trials*100:.2f}%      | {s_base/num_trials*100:.2f}%")
    print(f"Avg Assigned Tasks  | {a_datw:.2f}       | {a_cbba:.2f}       | {a_base:.2f}")
    print(f"Avg Global Cost     | {c_datw:.2f}      | {c_cbba:.2f}      | {c_base:.2f}")
    
    # Calculate Optimization Percentage (DATW vs CBBA)
    if c_cbba > 0:
        improvement = (c_cbba - c_datw) / c_cbba * 100
        print(f"\nDATW Time Optimization: {improvement:.2f}% better than CBBA")
    
    if c_base > 0:
        base_improvement = (c_base - c_datw) / c_base * 100
        print(f"DATW Time Optimization: {base_improvement:.2f}% better than Baseline")

if __name__ == "__main__":
    run_experiments()
