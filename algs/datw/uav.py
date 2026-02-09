
import math
import copy
from shared.utils import calculate_distance
from shared.models import Task

class UAV:
    def __init__(self, uav_id, location, velocity, capacity, fuel, fuel_consumption_rate, all_tasks):
        """
        Initialize UAV with DATW properties.
        """
        self.id = uav_id
        self.location = location  # Initial location (x, y, z)
        self.velocity = velocity
        self.capacity = capacity  # Li
        self.fuel = fuel  # Foi
        self.fuel_consumption_rate = fuel_consumption_rate # vmi
        self.min_fuel_threshold = 0 # Delta, assumed 0 for now or passed in
        
        self.all_tasks = {t.id: t for t in all_tasks} # Nt
        self.tasks = []  # Pi: Ordered list of assigned tasks (Task objects)
        
        # DATW State Lists
        # Z: Task assignment map. Z[j] = uav_id if assigned to uav_id, else -1 (infinity)
        self.Z = {t.id: -1 for t in all_tasks} 
        
        # Q: Significance map. Q[j] = significance value
        self.Q = {t.id: float('inf') for t in all_tasks}
        
        # T: Start time map. T[j] = start time
        self.T = {t.id: float('inf') for t in all_tasks}
        
        # s: Timestamp list for each UAV (assuming n UAVs with ids 0 to n-1)
        # We don't know N here easily without passing it, let's assume dynamic or passed in map
        self.s = {} # Map uav_id -> timestamp

    def calculate_arrival_time(self, task_sequence, target_task, insertion_index):
        """
        Calculate arrival time for target_task if inserted at insertion_index in task_sequence.
        formula (1)
        """
        # Determine previous location and time
        if insertion_index == 0:
            prev_loc = self.location
            prev_finish_time = 0 # Start time of mission (global clock 0)
        else:
            prev_task = task_sequence[insertion_index - 1]
            prev_loc = prev_task.location
            # We need the start time of the previous task in this specific sequence context
            # This is tricky because T is global map. 
            # We should recalculate times for the temporary sequence.
            # So let's re-eval the whole sequence time for accuracy or just the segment.
            # To be safe and simple, let's calculate times for the whole sequence.
            return self._calculate_times_for_sequence(task_sequence[:insertion_index] + [target_task])[insertion_index]

        dist = calculate_distance(prev_loc, target_task.location)
        arrival_time = prev_finish_time + (dist / self.velocity)
        return arrival_time

    def _calculate_times_for_sequence(self, sequence):
        """
        Calculate start times for a sequence of tasks starting from UAV initial state.
        Returns list of start times.
        """
        times = []
        curr_loc = self.location
        curr_time = 0
        
        for task in sequence:
            dist = calculate_distance(curr_loc, task.location)
            arrival = curr_time + (dist / self.velocity)
            start_time = max(arrival, task.time_window[0]) # Wait if arrived early? 
            # The paper says: Tj(Pi) = Tk(Pi) + Dk + dist/v. 
            # And constraint (9) says Tj_start <= Tj(Pi) <= Tj_end.
            # It implies we can wait? "validity interval for tj is given as...".
            # Usually in these problems, you can wait. "Time window constraints" usually means you can start anytime in [start, end].
            # If you arrive early, you wait until start. If you arrive after end, it's invalid.
            # However, formula (1) doesn't explicitly show waiting logic, it just sums up.
            # But standard TW problems allow waiting. Let's assume waiting is allowing but we check validity later.
            # Actually, standard CBBA allows waiting. 
            # Let's use max(arrival, early_start).
            
            times.append(start_time)
            curr_time = start_time + task.duration
            curr_loc = task.location
            
        return times

    def calculate_local_time_cost(self, sequence):
        """
        Calculate F(Pi) - Sum of completion times?
        Paper says: F(Pi) = Sum(Fk(Pi)) for all k in Pi.
        Fk(Pi) = Tk(Pi) + Dk.
        """
        times = self._calculate_times_for_sequence(sequence)
        cost = 0
        for i, task in enumerate(sequence):
            completion_time = times[i] + task.duration
            cost += completion_time
        return cost

    def calculate_significance(self, task, sequence):
        """
        Calculate significance q_ij(Pi - tj)
        Formula (12): [F(Pi) - F(Pi-tj)] * [Tj(Pi) - Tj_start]
        """
        # If task not in sequence, return inf
        if task not in sequence:
            return float('inf')
        
        # Pi
        cost_with = self.calculate_local_time_cost(sequence)
        
        # Pi - tj
        seq_without = [t for t in sequence if t.id != task.id]
        cost_without = self.calculate_local_time_cost(seq_without)
        
        # Tj(Pi)
        times = self._calculate_times_for_sequence(sequence)
        idx = sequence.index(task)
        start_time = times[idx]
        
        term1 = cost_with - cost_without
        term2 = start_time - task.time_window[0]
        
        return term1 * term2

    def calculate_marginal_significance(self, task, sequence):
        """
        Calculate marginal significance q*_ij(Pi + tj)
        Formula (13): min_k [F(Pi +k tj) - F(Pi)] * [Tj(Pi) - Tj_start]
        """
        if task in sequence:
            return float('inf')
            
        best_marginal = float('inf')
        best_idx = -1
        
        current_cost = self.calculate_local_time_cost(sequence)
        
        # Try inserting at every position
        for k in range(len(sequence) + 1):
            new_seq = sequence[:k] + [task] + sequence[k:]
            
            # Check feasibility (time windows & power) - This is checked in Phase 1 loop, 
            # but usually marginal significance is just calculation.
            # The paper says: "Conditions (a), (b), (c)". 
            # Condition (b) says "after inserting... all tasks satisfy time window".
            # We should probably filter invalid insertions here or outside. 
            # Formula (13) calculates value. If invalid, maybe it's inf?
            
            # Check time constraints for new_seq
            times = self._calculate_times_for_sequence(new_seq)
            valid_time = True
            for i, t in enumerate(new_seq):
                if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                    valid_time = False
                    break
            if not valid_time:
                continue

            new_cost = self.calculate_local_time_cost(new_seq)
            
            # Tj(Pi) -> Start time of task in new sequence
            start_time = times[k]
            
            term1 = new_cost - current_cost
            term2 = start_time - task.time_window[0]
            
            val = term1 * term2
            
            if val < best_marginal:
                best_marginal = val
                best_idx = k
                
        return best_marginal, best_idx

    # --- Phase 1: Task Inclusion ---
    def task_inclusion(self):
        """
        Algorithm 1: Task Inclusion Phase
        """
        print(f"UAV {self.id} running Task Inclusion...")
        updated = False
        while len(self.tasks) < self.capacity:
            best_task = None
            best_marginal = float('inf')
            best_pos = -1
            
            # Identify candidate tasks P(Pi)
            candidates = []
            for task_id, task in self.all_tasks.items():
                if task in self.tasks:
                    continue
                
                # Condition (a): Zij != i (implied if not in self.tasks, check world state Z later if needed)
                # The paper says "Zij != i and q* < Qij". 
                # Note: In distributed context, Z represents *belief*. 
                # If Z[task_id] == self.id, we already own it. 
                # But here we loop over all tasks. 
                if self.Z[task_id] == self.id: 
                    continue # Already mine
                
                # Calculate marginal significance
                # This returns min over all positions.
                marg, pos = self.calculate_marginal_significance(task, self.tasks)
                
                if marg == float('inf'):
                    continue
                
                # Condition (a) check Qij
                if marg >= self.Q[task_id]:
                    continue
                    
                # Condition (c) Power constraint
                # Fri = Foi - vmi * F(Pi+tj) >= Delta
                # F(Pi+tj) can be approximate by current_cost + (marg / (start-early_start) ?) 
                # No, better calculate explicitly.
                # Re-construct seq to check power
                seq_with_task = self.tasks[:pos] + [task] + self.tasks[pos:]
                cost_new = self.calculate_local_time_cost(seq_with_task)
                remaining_fuel = self.fuel - (self.fuel_consumption_rate * cost_new)
                if remaining_fuel < self.min_fuel_threshold:
                    continue
                
                # Add to candidates
                candidates.append((task, marg, pos))

            if not candidates:
                break
                
            # Select best task tk = argmax (Qij - q*ij)
            # Paper Eq (14)
            best_candidate = None
            max_diff = -float('inf')
            
            for task, marg, pos in candidates:
                diff = self.Z[task.id] # Wait, Qij is significance. check self.Q
                diff = self.Q[task.id] - marg
                
                if diff > max_diff:
                    max_diff = diff
                    best_candidate = (task, marg, pos)
            
            if best_candidate:
                tk, marg, pos = best_candidate
                # Insert task
                self.tasks.insert(pos, tk)
                self.Z[tk.id] = self.id
                self.Q[tk.id] = marg
                
                # Update T (start times) for all tasks in Pi
                self._update_time_vector()
                updated = True
            else:
                break
        
        # After loop, update Q for all tasks in Pi?
        # "Since inclusion ... may change significance ... list Q is required to update accordingly"
        # Re-calc Q for all owned tasks
        for task in self.tasks:
             self.Q[task.id] = self.calculate_significance(task, self.tasks)
             
        return updated

    def _update_time_vector(self):
        times = self._calculate_times_for_sequence(self.tasks)
        for i, task in enumerate(self.tasks):
            self.T[task.id] = times[i]

    # --- Phase 2: Conflict Resolution ---
    def communicate(self, other_uav):
        """
        Exchange Z, Q, T, s with neighbor.
        Simplified: We just return our state, simulation handles the exchange and `update_state`.
        """
        return {
            'id': self.id,
            'Z': self.Z.copy(),
            'Q': self.Q.copy(),
            'T': self.T.copy(),
            's': self.s.copy()
        }

    def update_state(self, other_info, current_time):
        """
        Algorithm 2: Consensus Stage Rules (Table 2)
        """
        sender_id = other_info['id']
        sender_Z = other_info['Z']
        sender_Q = other_info['Q']
        sender_T = other_info['T']
        sender_s = other_info['s']
        
        # Update timestamp for sender
        self.s[sender_id] = current_time 
        # Update other timestamps? "s_ih = max_alpha ..." Formula (11) is for mesh relay? 
        # If mesh, we assume full connectivity for now (G_ih = 1).
        # So just direct update.
        
        changes = False
        
        for j, task in self.all_tasks.items():
            # Apply Table 2 rules
            z_i = self.Z[j]
            z_h = sender_Z[j]
            
            # Simulating Table 2 logic
            # Columns: Zhj (sender belief), Zij (my belief)
            action = 'leave'
            
            # Row 1: h, i
            if z_h == sender_id and z_i == self.id:
                if sender_Q[j] < self.Q[j]:
                    action = 'update'
                else: 
                     # Implicitly if Q_h > Q_i, sender should yield. 
                     # But here we are receiver. If my Q is better (lower), I ignore him?
                     # Table says: if Q_hj < Q_ij: update. (Accept his better bid)
                     pass
            
            # Row 2: h, h -> update (Refresh info)
            elif z_h == sender_id and z_i == sender_id:
                action = 'update'
            
            # Row 3: h, alpha -> check timestamp or Q
            # ... Implementing full table logic is tedious but necessary.
            # Simplified Logic adhering to Bundle Algorithm principles:
            # 1. Provide latest info.
            # 2. Minimize Q.
            
            # Let's try to map strictly to Table 2
            # Needs timestamps for 3rd party (s_ha vs s_ia)
            # We need s vector. 
            pass # TODO: Implement full Table 2 logic
            
            # Fallback simple logic for MVP with Tie-Breaking:
            
            # If he assigns to himself and I assigned to myself
            if z_h == sender_id and z_i == self.id:
                # Compare Q. If sender is better (lower Q), I update.
                # If equal Q, prefer lower ID (sender_id < self.id).
                if sender_Q[j] < self.Q[j] or (abs(sender_Q[j] - self.Q[j]) < 1e-9 and sender_id < self.id):
                     self.Z[j] = z_h
                     self.Q[j] = sender_Q[j]
                     self.T[j] = sender_T[j]
                     changes = True
            
            # If he assigns to himself, and I think it's unassigned or assigned to someone else (not me)
            # Update if his info is valid claim
            elif z_h == sender_id and z_i != self.id:
                 # Check if he outbids current owner (if any)
                 # Or if I have no info.
                 # Simplified: take his info if it looks better or newer.
                 # Assuming he knows best about his own tasks.
                 # But if I think UAV k has it with Q_k < Q_h, I shouldn't accept h.
                 if self.Q[j] > sender_Q[j] or self.Z[j] == -1: # Naive update
                     self.Z[j] = z_h
                     self.Q[j] = sender_Q[j]
                     self.T[j] = sender_T[j]
                     changes = True
                 
            # If he thinks nobody assigned it (reset), but I think ...
            elif z_h == -1 and z_i == sender_id:
                 # He released it.
                 self.Z[j] = -1
                 self.Q[j] = float('inf')
                 self.T[j] = float('inf')
                 changes = True
                 
        return changes

    def task_removal(self):
        """
        Algorithm 2: Task Removal Stage
        A = {tj in Pi | Zij != i} -> Lost tasks
        B = {tj in Pi | Time constraints violated}
        """
        # Set A: Tasks I have in Pi but my Z says belong to someone else 
        # (updated during consensus)
        
        # 1. Remove tasks in A
        # "Release a task tk... which maximizes (qij - Qij)"
        # Actually, if Z says it's not mine, I MUST drop it?
        # Protocol says: "max(qij - Qij) > 0".
        # qij is current significance, Qij is stored (network) significance.
        # If network says someone has better Q (lower), Qij stored is lower.
        # qij > Qij means my current cost is worse than what network thinks (or someone else's).
        
        # In standard CBBA: release if you are outbid.
        # Here: `Zij != i` means I acknowledged someone else won it in Consensus. 
        # So I should remove it.
        
        removed = False
        
        # Filter sequences
        # We iterate and remove.
        # Check A
        to_remove_A = []
        for task in self.tasks:
            if self.Z[task.id] != self.id:
                to_remove_A.append(task)
        
        for task in to_remove_A:
            self.tasks.remove(task)
            removed = True
            
        # Check B (Time violations)
        # Re-calc times
        times = self._calculate_times_for_sequence(self.tasks)
        to_remove_B = []
        for i, task in enumerate(self.tasks):
            t_start = times[i]
            if not (task.time_window[0] <= t_start <= task.time_window[1]):
                to_remove_B.append(task)
        
        for task in to_remove_B:
            if task in self.tasks:
                self.tasks.remove(task)
                self.Z[task.id] = -1
                self.Q[task.id] = float('inf')
                self.T[task.id] = float('inf')
                removed = True
                
        # If removal happened, update time and significance for remaining
        if removed:
            self._update_time_vector()
            for task in self.tasks:
                self.Q[task.id] = self.calculate_significance(task, self.tasks)
                
        return removed

    # --- Phase 3: Task Reallocation ---
    def secondary_inclusion(self):
        """
        Algorithm for Phase 3: Secondary Inclusion
        Assign tasks in Cai (unassigned tasks) to UAVs.
        Selection criterion: argmin q*ij (Minimize marginal increase)
        """
        # Identify unassigned tasks
        unassigned_ids = [tid for tid, owner in self.Z.items() if owner == -1]
        
        updated = False
        while len(self.tasks) < self.capacity:
            candidates = []
            
            for tid in unassigned_ids:
                if tid in [t.id for t in self.tasks]:
                    continue
                
                # Check condition (d) implicitly: Z[tid] == -1
                
                task = self.all_tasks[tid]
                marg, pos = self.calculate_marginal_significance(task, self.tasks)
                
                if marg == float('inf'):
                    continue
                
                # Check power constraint (Condition f)
                seq_with_task = self.tasks[:pos] + [task] + self.tasks[pos:]
                cost_new = self.calculate_local_time_cost(seq_with_task)
                remaining_fuel = self.fuel - (self.fuel_consumption_rate * cost_new)
                if remaining_fuel < self.min_fuel_threshold:
                    continue
                
                candidates.append((task, marg, pos))
                
            if not candidates:
                break
            
            # Select best task tk = argmin q*ij (Formula 16)
            # Find candidate with minimum marginal significance
            best_candidate = min(candidates, key=lambda x: x[1])
            
            if best_candidate:
                tk, marg, pos = best_candidate
                self.tasks.insert(pos, tk)
                self.Z[tk.id] = self.id
                self.Q[tk.id] = marg
                self._update_time_vector()
                updated = True
                
                # Remove from local unassigned list for next iteration
                if tk.id in unassigned_ids:
                    unassigned_ids.remove(tk.id)
            else:
                break
                
        return updated 
