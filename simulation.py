
import sys
from shared.simulation_utils import setup_scenario
from algs.runners import ConsensusRunner, CentralizedOptimalRunner
from algs.datw.uav import DATWUAV

# Main entry point for CLI or single run
def run_simulation(num_uavs=3, num_tasks=10, max_iterations=100, verbose=True, uav_class=DATWUAV, seed=None):
    
    # 1. Setup
    tasks, uav_configs = setup_scenario(num_uavs, num_tasks, seed)
    
    # 2. Instantiate UAVs
    uavs = []
    for cfg in uav_configs:
        uavs.append(uav_class(
            cfg['id'], 
            cfg['loc'], 
            cfg['velocity'], 
            cfg['capacity'], 
            cfg['fuel'], 
            cfg['rate'], 
            tasks
        ))

    # 3. Run with Consensus Algorithm (used by all distributed approaches)
    runner = ConsensusRunner(max_iterations=max_iterations, verbose=verbose)
        
    # 4. Run
    results = runner.run(uavs, tasks)
    
    if verbose:
        print(f"Total Global Cost: {results['total_cost']:.2f}")
        print(f"Assigned: {results['assigned_tasks']}/{results['num_tasks']}")
        print(f"Valid: {results.get('valid', 'Unknown')}")
        
    return results

if __name__ == "__main__":
    # Check Imports
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

    visualize = "--visualize" in sys.argv
    use_baseline = "--baseline" in sys.argv
    use_cbba = "--cbba" in sys.argv
    use_optimal = "--optimal" in sys.argv
    
    seed = None
    for arg in sys.argv:
        if arg.startswith("--seed="):
            seed = int(arg.split("=")[1])
            break
            
    if seed is not None:
        import random
        print(f"Running with seed {seed}")
        random.seed(seed)
    
    uav_cls = DATWUAV
    runner_cls = ConsensusRunner
    
    if use_optimal:
        if OptimalUAV:
            uav_cls = OptimalUAV
            runner_cls = CentralizedOptimalRunner
            print("Algorithm: Centralized Optimal (Theoretical Upper Bound)")
        else:
            print("OptimalUAV not found.")
    elif use_baseline:
         if BaselineUAV:
             uav_cls = BaselineUAV
             print("Algorithm: Baseline (Distributed Greedy)")
         else:
             print("BaselineUAV not found.")
    elif use_cbba:
        if CBBAUAV:
            uav_cls = CBBAUAV
            print("Algorithm: CBBA")
        else:
             print("CBBAUAV not found.")
    else:
        print("Algorithm: DATW")
        
    # Run with appropriate runner
    if use_optimal and OptimalUAV:
        # Optimal uses CentralizedOptimalRunner directly
        tasks, uav_configs = setup_scenario(num_uavs=3, num_tasks=10, seed=seed)
        uavs = [OptimalUAV(c['id'], c['loc'], c['velocity'], c['capacity'], c['fuel'], c['rate'], tasks) for c in uav_configs]
        runner = CentralizedOptimalRunner(verbose=True)
        res = runner.run(uavs, tasks)
    else:
        res = run_simulation(verbose=True, uav_class=uav_cls, seed=seed)
    
    if visualize:
        from visualizer import visualize_simulation
        visualize_simulation(res['uavs'], res['tasks'])
