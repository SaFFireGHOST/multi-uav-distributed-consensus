
from shared.models import UAV

class DATWUAV(UAV):
    """
    DATW (Distributed Allocation with Time Windows) UAV Implementation.
    Implements specific scoring logic (Time + Slack) and Reallocation phase.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def calculate_significance(self, task, sequence):
        """
        Calculate significance q_ij(Pi - tj)
        Formula (12): [F(Pi) - F(Pi-tj)] * [Tj(Pi) - Tj_start]
        """
        if task not in sequence:
            return float('inf')
        
        cost_with = self.calculate_local_time_cost(sequence)
        
        seq_without = [t for t in sequence if t.id != task.id]
        cost_without = self.calculate_local_time_cost(seq_without)
        
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
            return float('inf'), -1
            
        best_marginal = float('inf')
        best_idx = -1
        
        current_cost = self.calculate_local_time_cost(sequence)
        
        # Try inserting at every position
        for k in range(len(sequence) + 1):
            new_seq = sequence[:k] + [task] + sequence[k:]
            
            # Check Feasibility
            times = self._calculate_times_for_sequence(new_seq)
            valid_time = True
            for i, t in enumerate(new_seq):
                if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                    valid_time = False
                    break
            if not valid_time:
                continue

            new_cost = self.calculate_local_time_cost(new_seq)
            
            start_time = times[k]
            
            term1 = new_cost - current_cost
            term2 = start_time - task.time_window[0]
            
            val = term1 * term2
            
            if val < best_marginal:
                best_marginal = val
                best_idx = k
                
        return best_marginal, best_idx

    def secondary_inclusion(self):
        """
        Algorithm for Phase 3: Secondary Inclusion
        Assign tasks in Cai (unassigned tasks) to UAVs.
        Selection criterion: argmin q*ij (Minimize marginal increase)
        """
        unassigned_ids = [tid for tid, owner in self.Z.items() if owner == -1]
        
        updated = False
        while len(self.tasks) < self.capacity:
            candidates = []
            
            for tid in unassigned_ids:
                if tid in [t.id for t in self.tasks]:
                    continue
                
                task = self.all_tasks[tid]
                marg, pos = self.calculate_marginal_significance(task, self.tasks)
                
                if marg == float('inf'):
                    continue
                
                # Check power constraint
                seq_with_task = self.tasks[:pos] + [task] + self.tasks[pos:]
                cost_new = self.calculate_local_time_cost(seq_with_task)
                remaining_fuel = self.fuel - (self.fuel_consumption_rate * cost_new)
                if remaining_fuel < self.min_fuel_threshold:
                    continue
                
                candidates.append((task, marg, pos))
                
            if not candidates:
                break
            
            # Select best task tk = argmin q*ij (Formula 16)
            best_candidate = min(candidates, key=lambda x: x[1])
            
            if best_candidate:
                tk, marg, pos = best_candidate
                self.tasks.insert(pos, tk)
                self.Z[tk.id] = self.id
                self.Q[tk.id] = marg
                self._update_time_vector()
                updated = True
                
                if tk.id in unassigned_ids:
                    unassigned_ids.remove(tk.id)
            else:
                break
                
        return updated
