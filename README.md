# Multi-UAV Task Allocation with Time Windows (DATW)

This project implements and compares distributed task allocation algorithms for Multi-UAV systems, focusing on **Time Window Constraints**. It reproduces and extends the concepts from the research paper *"Distributed Task Allocation for a Multi-UAV System with Time Window Constraints"* (Cui et al., 2022).

## Algorithms Implemented

1.  **DATW (Distributed Allocation with Time Windows)**:
    *   **Mechanism**: Uses a PI (Performance Impact) based approach with a specific "Significance" heuristic that accounts for Time Window overlap and slack.
    *   **Phases**: Task Inclusion -> Conflict Resolution -> **Task Reallocation** (Phase 3).
    *   **Performance**: High success rate (~100%) and low global cost.

2.  **CBBA (Consensus Based Bundle Algorithm)**:
    *   **Mechanism**: Standard consensus-based auction.
    *   **Mechanism**: Uses marginal time cost reduction as the bidding metric.
    *   **Limitation**: Lacks the specific Phase 3 reallocation and time-slack optimization of DATW.

3.  **Baseline (Greedy Distance)**:
    *   **Mechanism**: Naive greedy approach. UAVs bid on the closest task regardless of time windows.
    *   **Performance**: Poor success rate (~60-70%) as it fails to prioritize urgent tasks.

## Project Structure

```
drones/
├── algs/                   # Algorithm Implementations
│   ├── datw/               # DATW UAV Logic
│   ├── cbba/               # CBBA UAV Logic
│   └── baseline/           # Greedy Baseline Logic
├── shared/                 # Shared Models and Utils
├── simulation.py           # Main Simulation Engine
├── visualizer.py           # 3D Matplotlib Visualizer
├── run_experiment.py       # Comparative Experiment Script
└── steps_to_run.txt        # Quick run commands
```

## Setup

Requires Python 3.x and the following libraries:
```bash
pip install matplotlib numpy
```

## Usage

### 1. Visual Simulation
Run a single simulation with 3D visualization. Use `--seed=42` to compare the exact same scenario across algorithms.

**Run DATW (Default):**
```bash
python simulation.py --visualize --seed=42
```

**Run CBBA:**
```bash
python simulation.py --visualize --cbba --seed=42
```

**Run Baseline:**
```bash
python simulation.py --visualize --baseline --seed=42
```

*Note: The `--seed` argument is optional but executing with the same seed allows for a fair visual comparison of behavior.*

### 2. Comparative Experiments
Run a batch of experiments (e.g., 30 trials) to gather statistical metrics (Success Rate, Average Cost).

```bash
python run_experiment.py
```

This will output a summary table comparing the three algorithms:
```text
Metrics             | DATW        | CBBA        | Baseline
--------------------|-------------|-------------|-------------
Success Rate        | 100.00%     | 90.00%      | 70.00%
Avg Assigned Tasks  | 10.00       | 9.00        | 7.00
Avg Global Cost     | 430.00      | 465.00      | 520.00

DATW Time Optimization: 45.12% better than Baseline
```

## Key Features
- **Deterministic Seeding**: Ensures reproducibility using `--seed`.
- **3D Visualization**: Shows UAV trajectories, Task locations, and Time Windows.
- **Phase Convergence**: Displays convergence steps for Conflict Resolution phases.
