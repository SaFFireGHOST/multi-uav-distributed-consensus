
from shared.models import UAV
from shared.utils import calculate_distance

class BaselineUAV(UAV):
    """
    Baseline UAV implementation using Greedy Distance-Based Auction.
    Significance = Distance cost only.
    No complex time-slack optimization.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
    def calculate_significance(self, task, sequence):
        """
        Baseline Significance: Just the distance cost from previous task.
        We minimize distance, so significance = distance.
        (Lower is better).
        """
        if task not in sequence:
            return float('inf')
            
        # Find distance from previous task in sequence
        idx = sequence.index(task)
        if idx == 0:
            prev_loc = self.location
        else:
            prev_loc = sequence[idx-1].location
            
        return calculate_distance(prev_loc, task.location)

    def calculate_marginal_significance(self, task, sequence):
        """
        Baseline: Simple distance-based greedy.
        Appends tasks to end of sequence (Nearest Neighbor).
        Returns (distance, position) or (inf, -1) if time window invalid.
        """
        if task in sequence:
            return float('inf'), -1
        
        # Append to end
        k = len(sequence)
        new_seq = sequence + [task]
        
        # Check Time Window Validity
        times = self._calculate_times_for_sequence(new_seq)
        
        # Check if valid
        valid = True
        for i, t in enumerate(new_seq):
             if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                 valid = False
                 break
        
        if not valid:
            return float('inf'), -1
            
        # Calculate distance cost (marginal)
        # Distance from last task to new task
        if sequence:
            last_loc = sequence[-1].location
        else:
            last_loc = self.location
            
        dist = calculate_distance(last_loc, task.location)
        
        return dist, k

    def task_inclusion(self):
        """
        Baseline Greedy Inclusion:
        Iteratively pick the feasible task with smallest distance from current end.
        """
        updated = False
        while len(self.tasks) < self.capacity:
            best_task = None
            min_dist = float('inf')
            
            # Find closest valid task
            for task_id, task in self.all_tasks.items():
                if task in self.tasks:
                    continue
                if self.Z[task_id] == self.id:
                    continue
                
                dist, pos = self.calculate_marginal_significance(task, self.tasks)
                
                if dist == float('inf'):
                    continue
                
                if dist >= self.Q[task_id]:
                    continue
                    
                if dist < min_dist:
                    min_dist = dist
                    best_task = task
            
            if best_task:
                dist, pos = self.calculate_marginal_significance(best_task, self.tasks)
                self.tasks.append(best_task)
                self.Z[best_task.id] = self.id
                self.Q[best_task.id] = dist
                self._update_time_vector()
                updated = True
            else:
                break
                
        return updated
        
    def secondary_inclusion(self):
        """Reuse same greedy logic for Phase 3 reallocation."""
        return self.task_inclusion()
