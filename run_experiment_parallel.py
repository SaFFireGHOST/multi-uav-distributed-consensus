import random
import statistics
import time
from shared.simulation_utils import setup_scenario
from algs.runners import ConsensusRunner, CentralizedOptimalRunner
from algs.datw.uav import DATWUAV
from algs.optimal_parallel.runner import ParallelCentralizedOptimalRunner
try:
    from algs.baseline.uav import BaselineUAV
except ImportError:
    BaselineUAV = None
try:
    from algs.optimal.uav import OptimalUAV
except ImportError:
    OptimalUAV = None

def run_experiments(num_trials=30):
    print(f"Running {num_trials} experiments for Comparison...")
    
    datw_results = []
    baseline_results = []
    optimal_results = []
    optimal_parallel_results = []
    
    num_tasks = 10  # fixed scenario size

    for i in range(num_trials):
        seed = i
        
        # 1. Setup Shared Scenario (Same for all algs in this trial)
        tasks, uav_configs = setup_scenario(num_uavs=3, num_tasks=num_tasks, seed=seed)
        
        # Helper to create fresh UAVs
        def create_uavs(cls, configs, task_list):
            return [cls(c['id'], c['loc'], c['velocity'], c['capacity'], c['fuel'], c['rate'], task_list) for c in configs]
            
        # --- Run DATW ---
        random.seed(seed)
        uavs_datw = create_uavs(DATWUAV, uav_configs, tasks)
        runner_datw = ConsensusRunner(max_iterations=50, verbose=False)
        res_datw = runner_datw.run(uavs_datw, tasks)
        datw_results.append(res_datw)
        
        # --- Run Baseline ---
        random.seed(seed)
        base_cls = BaselineUAV if BaselineUAV else DATWUAV
        uavs_base = create_uavs(base_cls, uav_configs, tasks)
        runner_base = ConsensusRunner(max_iterations=50, verbose=False)
        res_base = runner_base.run(uavs_base, tasks)
        baseline_results.append(res_base)

        # --- Run Optimal (Original) ---
        if OptimalUAV:
            random.seed(seed)
            uavs_optimal = create_uavs(OptimalUAV, uav_configs, tasks)
            runner_optimal = CentralizedOptimalRunner(verbose=False)
            res_optimal = runner_optimal.run(uavs_optimal, tasks)
            optimal_results.append(res_optimal)
        else:
            res_optimal = {'total_cost': 0, 'assigned_tasks': 0, 'success': False}
            optimal_results.append(res_optimal)
            
        # --- Run Optimal (Parallel) ---
        if OptimalUAV:
            random.seed(seed)
            uavs_optimal_par = create_uavs(OptimalUAV, uav_configs, tasks)
            runner_optimal_par = ParallelCentralizedOptimalRunner(verbose=False, num_workers=8)
            res_optimal_par = runner_optimal_par.run(uavs_optimal_par, tasks)
            optimal_parallel_results.append(res_optimal_par)
        else:
            res_optimal_par = {'total_cost': 0, 'assigned_tasks': 0, 'success': False}
            optimal_parallel_results.append(res_optimal_par)
        
        # Get execution times and mission times
        t_datw = res_datw.get('execution_time', 0)
        m_datw = res_datw.get('mission_time', 0)
        t_base = res_base.get('execution_time', 0)
        m_base = res_base.get('mission_time', 0)
        t_opt = res_optimal.get('execution_time', 0)
        m_opt = res_optimal.get('mission_time', 0)
        t_opt_par = res_optimal_par.get('execution_time', 0)
        m_opt_par = res_optimal_par.get('mission_time', 0)

        print(f"Trial {i}: DATW {res_datw['total_cost']:.2f} [Comp: {t_datw:.3f}s] | Base {res_base['total_cost']:.2f} [Comp: {t_base:.3f}s] | Optimal {res_optimal['total_cost']:.2f} [Comp: {t_opt:.3f}s] | Parallel Opt {res_optimal_par['total_cost']:.2f} [Comp: {t_opt_par:.3f}s]")

    # Calculate metrics
    def get_metrics(results):
        if not results: return 0, 0, 0, 0, 0
        success_count = sum(1 for r in results if r['success'])
        avg_cost = statistics.mean([r['total_cost'] for r in results])
        avg_assigned = statistics.mean([r['assigned_tasks'] for r in results])
        avg_time = statistics.mean([r.get('execution_time', 0) for r in results])
        avg_mission = statistics.mean([r.get('mission_time', 0) for r in results])
        return success_count, avg_cost, avg_assigned, avg_time, avg_mission

    s_datw, c_datw, a_datw, t_datw_avg, m_datw_avg = get_metrics(datw_results)
    s_base, c_base, a_base, t_base_avg, m_base_avg = get_metrics(baseline_results)
    s_optimal, c_optimal, a_optimal, t_optimal_avg, m_optimal_avg = get_metrics(optimal_results)
    s_par, c_par, a_par, t_par_avg, m_par_avg = get_metrics(optimal_parallel_results)
    
    print("\n--- Comparative Results ---")
    print(f"Trials: {num_trials}")
    print(f"{'Metrics':<20} | {'DATW':<11} | {'Baseline':<11} | {'Optimal':<11} | {'Parallel Opt':<12}")
    print("-" * 75)
    print(f"{'Success Rate':<20} | {s_datw/num_trials*100:6.2f}%    | {s_base/num_trials*100:6.2f}%    | {s_optimal/num_trials*100:6.2f}%    | {s_par/num_trials*100:6.2f}%")
    print(f"{'Avg Assigned Tasks':<20} | {a_datw:<11.2f} | {a_base:<11.2f} | {a_optimal:<11.2f} | {a_par:<12.2f}")
    print(f"{'Avg Computation Time':<20} | {t_datw_avg:7.4f}s  | {t_base_avg:7.4f}s  | {t_optimal_avg:7.4f}s  | {t_par_avg:8.4f}s")
    print(f"{'Avg Mission Time':<20} | {m_datw_avg:6.2f}s   | {m_base_avg:6.2f}s   | {m_optimal_avg:6.2f}s   | {m_par_avg:7.2f}s")

    # Clean up parallel runner pool
    ParallelCentralizedOptimalRunner.close_pool()

if __name__ == "__main__":
    run_experiments()
