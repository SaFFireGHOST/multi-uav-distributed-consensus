
import random
import statistics
from shared.simulation_utils import setup_scenario
from algs.runners import ConsensusRunner, CentralizedOptimalRunner
from algs.datw.uav import DATWUAV
try:
    from algs.baseline.uav import BaselineUAV
except ImportError:
    BaselineUAV = None
try:
    from algs.cbba.uav import CBBAUAV
except ImportError:
    CBBAUAV = None
try:
    from algs.optimal.uav import OptimalUAV
except ImportError:
    OptimalUAV = None

def run_experiments(num_trials=30):
    print(f"Running {num_trials} experiments for Comparison...")
    
    datw_results = []
    baseline_results = []
    cbba_results = []
    optimal_results = []
    
    for i in range(num_trials):
        seed = i
        
        # 1. Setup Shared Scenario (Same for all algs in this trial)
        tasks, uav_configs = setup_scenario(num_uavs=3, num_tasks=10, seed=seed)
        
        # Helper to create fresh UAVs
        def create_uavs(cls, configs, task_list):
            return [cls(c['id'], c['loc'], c['velocity'], c['capacity'], c['fuel'], c['rate'], task_list) for c in configs]
            
        # --- Run DATW ---
        # Re-seed for internal randomness if any (Runner is deterministic usually but good practice)
        random.seed(seed)
        uavs_datw = create_uavs(DATWUAV, uav_configs, tasks)
        runner_datw = ConsensusRunner(max_iterations=50, verbose=False)
        res_datw = runner_datw.run(uavs_datw, tasks)
        datw_results.append(res_datw)
        
        # --- Run Baseline ---
        # Distributed greedy using distance-based cost (vs DATW's time optimization)
        random.seed(seed)
        base_cls = BaselineUAV if BaselineUAV else DATWUAV
        uavs_base = create_uavs(base_cls, uav_configs, tasks)
        runner_base = ConsensusRunner(max_iterations=50, verbose=False)
        res_base = runner_base.run(uavs_base, tasks)
        baseline_results.append(res_base)

        # --- Run CBBA ---
        if CBBAUAV:
             random.seed(seed)
             uavs_cbba = create_uavs(CBBAUAV, uav_configs, tasks)
             runner_cbba = ConsensusRunner(max_iterations=50, verbose=False)
             res_cbba = runner_cbba.run(uavs_cbba, tasks)
             cbba_results.append(res_cbba)
        else:
             res_cbba = {'total_cost': 0, 'assigned_tasks': 0, 'success': False}
             cbba_results.append(res_cbba)
        
        # --- Run Optimal ---
        if OptimalUAV:
            random.seed(seed)
            uavs_optimal = create_uavs(OptimalUAV, uav_configs, tasks)
            runner_optimal = CentralizedOptimalRunner(verbose=False)
            res_optimal = runner_optimal.run(uavs_optimal, tasks)
            optimal_results.append(res_optimal)
        else:
            res_optimal = {'total_cost': 0, 'assigned_tasks': 0, 'success': False}
            optimal_results.append(res_optimal)
        
        print(f"Trial {i}: DATW {res_datw['total_cost']:.2f} ({res_datw['assigned_tasks']}/{res_datw['num_tasks']}) | CBBA {res_cbba['total_cost']:.2f} ({res_cbba['assigned_tasks']}/{res_cbba['num_tasks']}) | Base {res_base['total_cost']:.2f} ({res_base['assigned_tasks']}/{res_base['num_tasks']}) | Optimal {res_optimal['total_cost']:.2f} ({res_optimal['assigned_tasks']}/{res_optimal['num_tasks']})")

    # Calculate metrics
    def get_metrics(results):
        if not results: return 0, 0, 0
        success_count = sum(1 for r in results if r['success'])
        avg_cost = statistics.mean([r['total_cost'] for r in results])
        avg_assigned = statistics.mean([r['assigned_tasks'] for r in results])
        return success_count, avg_cost, avg_assigned

    s_datw, c_datw, a_datw = get_metrics(datw_results)
    s_base, c_base, a_base = get_metrics(baseline_results)
    s_cbba, c_cbba, a_cbba = get_metrics(cbba_results)
    s_optimal, c_optimal, a_optimal = get_metrics(optimal_results)
    
    print("\n--- Comparative Results ---")
    print(f"Trials: {num_trials}")
    print(f"Metrics             | DATW        | CBBA        | Baseline    | Optimal")
    print(f"--------------------|-------------|-------------|-------------|-------------")
    print(f"Success Rate        | {s_datw/num_trials*100:.2f}%      | {s_cbba/num_trials*100:.2f}%      | {s_base/num_trials*100:.2f}%      | {s_optimal/num_trials*100:.2f}%")
    print(f"Avg Assigned Tasks  | {a_datw:.2f}       | {a_cbba:.2f}       | {a_base:.2f}       | {a_optimal:.2f}")
    print(f"Avg Global Cost     | {c_datw:.2f}      | {c_cbba:.2f}      | {c_base:.2f}      | {c_optimal:.2f}")
    
    # Calculate Optimization Percentage
    if c_optimal > 0:
        gap_datw = (c_datw - c_optimal) / c_optimal * 100
        gap_cbba = (c_cbba - c_optimal) / c_optimal * 100
        gap_base = (c_base - c_optimal) / c_optimal * 100
        print(f"\nOptimality Gap (vs Centralized Optimal):")
        print(f"  DATW: {gap_datw:.2f}% above optimal")
        print(f"  CBBA: {gap_cbba:.2f}% above optimal")
        print(f"  Baseline: {gap_base:.2f}% above optimal")

if __name__ == "__main__":
    run_experiments()
