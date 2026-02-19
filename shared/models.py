
import math
import copy
from shared.utils import calculate_distance

class Task:
    def __init__(self, task_id, location, duration, time_window, priority=1):
        """
        Initialize a Task.
        
        Args:
            task_id (int): Unique identifier for the task.
            location (tuple): (x, y, z) coordinates.
            duration (float): Time required to execute the task (Dj).
            time_window (tuple): (start, end) time window [Tj_start, Tj_end].
            priority (int): Priority of the task (default 1, not explicitly used in DATW but good for extension).
        """
        self.id = task_id
        self.location = location
        self.duration = duration
        self.time_window = time_window # (early_start, late_start)
        self.priority = priority

    def __repr__(self):
        return f"Task(id={self.id}, loc={self.location}, dur={self.duration}, tw={self.time_window})"


class UAV:
    """
    Base UAV class for Distributed Consensus Algorithms (CBBA, DATW, etc.).
    Manages state (Z, Q, T, s), communication, and generic bundle building.
    Specific scoring logic must be implemented by subclasses.
    """
    def __init__(self, uav_id, location, velocity, capacity, fuel, fuel_consumption_rate, all_tasks):
        """
        Initialize UAV with distributed consensus properties.
        """
        self.id = uav_id
        self.location = location  # Initial location (x, y, z)
        self.velocity = velocity
        self.capacity = capacity  # Li
        self.fuel = fuel  # Foi
        self.fuel_consumption_rate = fuel_consumption_rate # vmi
        self.min_fuel_threshold = 0 # Delta
        
        self.all_tasks = {t.id: t for t in all_tasks} # Nt
        self.tasks = []  # Pi: Ordered list of assigned tasks (Task objects)
        
        # Distributed Consensus State
        self.Z = {t.id: -1 for t in all_tasks} # Assignment map
        self.Q = {t.id: float('inf') for t in all_tasks} # Significance map
        self.T = {t.id: float('inf') for t in all_tasks} # Start time map
        self.s = {} # Map uav_id -> timestamp (Vector s)

    def calculate_arrival_time(self, task_sequence, target_task, insertion_index):
        """
        Calculate arrival time for target_task if inserted at insertion_index.
        """
        if insertion_index == 0:
            prev_loc = self.location
            prev_finish_time = 0 
        else:
            # Simple approximation for generic check, or use full recalc
            return self._calculate_times_for_sequence(task_sequence[:insertion_index] + [target_task])[insertion_index]

        dist = calculate_distance(prev_loc, target_task.location)
        arrival_time = prev_finish_time + (dist / self.velocity)
        return arrival_time

    def _calculate_times_for_sequence(self, sequence):
        """
        Calculate start times for a sequence of tasks.
        Returns list of start times.
        """
        times = []
        curr_loc = self.location
        curr_time = 0
        
        for task in sequence:
            dist = calculate_distance(curr_loc, task.location)
            arrival = curr_time + (dist / self.velocity)
            start_time = max(arrival, task.time_window[0]) 
            times.append(start_time)
            curr_time = start_time + task.duration
            curr_loc = task.location
            
        return times

    def calculate_local_time_cost(self, sequence):
        """
        Calculate generic execution cost (Sum of completion times).
        Used by many scoring functions.
        """
        times = self._calculate_times_for_sequence(sequence)
        cost = 0
        for i, task in enumerate(sequence):
            completion_time = times[i] + task.duration
            cost += completion_time
        return cost

    def calculate_significance(self, task, sequence):
        """
        Calculate significance score for a task (q_ij).
        Must be implemented by subclass.
        """
        raise NotImplementedError("Subclasses must implement calculate_significance")

    def calculate_marginal_significance(self, task, sequence):
        """
        Calculate marginal significance score (q*_ij) and best position.
        Must be implemented by subclass.
        Returns: (score, best_index)
        """
        raise NotImplementedError("Subclasses must implement calculate_marginal_significance")

    # --- Phase 1: Task Inclusion (Generic Bundle Building) ---
    def task_inclusion(self):
        """
        Generic Task Inclusion Phase (Algorithm 1).
        Iteratively adds tasks that maximize marginal significance.
        """
        # print(f"UAV {self.id} running Task Inclusion...")
        updated = False
        while len(self.tasks) < self.capacity:
            candidates = []
            for task_id, task in self.all_tasks.items():
                if task in self.tasks:
                    continue
                if self.Z[task_id] == self.id: 
                    continue # Already mine
                
                # Calculate marginal significance (Subclass specific logic)
                marg, pos = self.calculate_marginal_significance(task, self.tasks)
                
                if marg == float('inf'):
                    continue
                
                # Condition (a) check Qij
                if marg >= self.Q[task_id]:
                    continue
                    
                # Condition (c) Power/Fuel constraint check
                # Simplified check here, can be refined
                seq_with_task = self.tasks[:pos] + [task] + self.tasks[pos:]
                cost_new = self.calculate_local_time_cost(seq_with_task)
                remaining_fuel = self.fuel - (self.fuel_consumption_rate * cost_new)
                if remaining_fuel < self.min_fuel_threshold:
                     continue
                
                candidates.append((task, marg, pos))

            if not candidates:
                break
                
            # Select best task
            best_candidate = None
            max_diff = -float('inf')
            
            for task, marg, pos in candidates:
                # Maximize (Qij - q*ij) implies we want small q*ij relative to current Qij
                # or just Maximize Score if Q is score?
                # DATW: Minimize Cost -> Marg is cost. Q is cost. 
                # We want (Current Best Cost Q) - (My Matrix Cost).
                # If My Cost < Current Best, diff is positive. Maximize it.
                diff = self.Q[task.id] - marg
                
                if diff > max_diff:
                    max_diff = diff
                    best_candidate = (task, marg, pos)
            
            if best_candidate and max_diff > 0: # Ensure improvement
                tk, marg, pos = best_candidate
                self.tasks.insert(pos, tk)
                self.Z[tk.id] = self.id
                self.Q[tk.id] = marg
                self._update_time_vector()
                updated = True
            else:
                break
        
        # Update Q for owned tasks
        for task in self.tasks:
             self.Q[task.id] = self.calculate_significance(task, self.tasks)
             
        return updated

    def _update_time_vector(self):
        times = self._calculate_times_for_sequence(self.tasks)
        for i, task in enumerate(self.tasks):
            self.T[task.id] = times[i]

    # --- Phase 2: Conflict Resolution (Generic) ---
    def communicate(self, other_uav):
        """Exchange state."""
        return {
            'id': self.id,
            'Z': self.Z.copy(),
            'Q': self.Q.copy(),
            'T': self.T.copy(),
            's': self.s.copy()
        }

    def update_state(self, other_info, current_time):
        """
        Algorithm 2: Consensus Stage Rules (Table 2).
        Implementation of standard CBBA consensus rules.
        """
        sender_id = other_info['id']
        sender_Z = other_info['Z']
        sender_Q = other_info['Q']
        sender_T = other_info['T']
        
        self.s[sender_id] = current_time 
        changes = False
        
        for j, task in self.all_tasks.items():
            z_i = self.Z[j]
            z_h = sender_Z[j]
            
            if z_h == sender_id and z_i == self.id:
                # Conflict: Both claim it
                if sender_Q[j] < self.Q[j] or (abs(sender_Q[j] - self.Q[j]) < 1e-9 and sender_id < self.id):
                     self.Z[j] = z_h
                     self.Q[j] = sender_Q[j]
                     self.T[j] = sender_T[j]
                     changes = True
            
            elif z_h == sender_id and z_i != self.id:
                 # He claims it, I don't (or I think someone else does)
                 # Update if his claim is better than what I know
                 if self.Q[j] > sender_Q[j] or self.Z[j] == -1: 
                     self.Z[j] = z_h
                     self.Q[j] = sender_Q[j]
                     self.T[j] = sender_T[j]
                     changes = True
                 
            elif z_h == -1 and z_i == sender_id:
                 # He released it (was his, now -1)
                 self.Z[j] = -1
                 self.Q[j] = float('inf')
                 self.T[j] = float('inf')
                 changes = True
                 
        return changes

    def task_removal(self):
        """
        Algorithm 2: Task Removal Stage (Generic).
        Remove tasks that are lost in consensus or violate constraints.
        """
        removed = False
        
        # 1. Remove tasks lost in consensus (Z[j] != id)
        to_remove_A = []
        for task in self.tasks:
            if self.Z[task.id] != self.id:
                to_remove_A.append(task)
        
        for task in to_remove_A:
            self.tasks.remove(task)
            removed = True
            
        # 2. Remove tasks violating time constraints (Cascading effect)
        while True:
            times = self._calculate_times_for_sequence(self.tasks)
            to_remove_B = []
            for i, task in enumerate(self.tasks):
                t_start = times[i]
                if not (task.time_window[0] <= t_start <= task.time_window[1]):
                    to_remove_B.append(task)
                    # Removing one might fix others or break others? 
                    # Usually breaks succeeding ones. Remove first violation and re-loop roughly.
                    break 
            
            if not to_remove_B:
                break
                
            for task in to_remove_B:
                self.tasks.remove(task)
                self.Z[task.id] = -1
                self.Q[task.id] = float('inf')
                self.T[task.id] = float('inf')
                removed = True
                
        if removed:
            self._update_time_vector()
            for task in self.tasks:
                self.Q[task.id] = self.calculate_significance(task, self.tasks)
                
        return removed

    def secondary_inclusion(self):
        """
        Optional Phase 3. Default to False (No-op).
        Score-based reallocation.
        """
        return False
