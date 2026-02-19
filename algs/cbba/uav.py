from shared.models import UAV

class CBBAUAV(UAV):
    """
    CBBA (Consensus Based Bundle Algorithm) Implementation.
    Differences from DATW:
    1. Scoring: Pure marginal time cost (Minimize increase in total time).
       Does NOT use the (Start Time - Window Start) weighting factor.
    2. No Phase 3: Standard CBBA does not strictly include the reallocation phase 
       described in the DATW paper.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
    def calculate_marginal_significance(self, task, sequence):
        """
        CBBA Marginal Score: 
        c_ij = - ( J(Pi + tj) - J(Pi) )
        We want to MINIMIZE Cost J, which is equivalent to MAXIMIZING Score c_ij.
        
        Cost J = Total Completion Time (Sum of finish times).
        """
        if task in sequence:
            return float('inf'), -1
            
        best_marginal = float('inf')
        best_idx = -1
        
        current_cost = self.calculate_local_time_cost(sequence)
        
        # Try inserting at every position
        for k in range(len(sequence) + 1):
            new_seq = sequence[:k] + [task] + sequence[k:]
            
            # Check Feasibility (Time Windows)
            times = self._calculate_times_for_sequence(new_seq)
            valid_time = True
            for i, t in enumerate(new_seq):
                if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                    valid_time = False
                    break
            if not valid_time:
                continue

            new_cost = self.calculate_local_time_cost(new_seq)
            
            # Marginal Cost = Increase in local time cost
            marginal_cost = new_cost - current_cost
            
            if marginal_cost < best_marginal:
                best_marginal = marginal_cost
                best_idx = k
                
        return best_marginal, best_idx

    def calculate_significance(self, task, sequence):
        """
        Recalculate significance for a task already in the list.
        Cost without it vs Cost with it.
        """
        if task not in sequence:
            return float('inf')
            
        cost_with = self.calculate_local_time_cost(sequence)
        seq_without = [t for t in sequence if t.id != task.id]
        cost_without = self.calculate_local_time_cost(seq_without)
        
        return cost_with - cost_without

    def secondary_inclusion(self):
        """
        CBBA does not have Phase 3 (Reallocation).
        Return False to skip.
        """
        return False
