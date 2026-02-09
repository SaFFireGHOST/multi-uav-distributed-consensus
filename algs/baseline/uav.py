
from algs.datw.uav import UAV
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
        Baseline Marginal: Distance from end of current path to new task.
        Greedy approach: Always append to end? 
        Or insert where distance is minimal? 
        "Greedy distance" usually means picking the task closest to current position (or end of path).
        Let's assume insertion at optimal position to minimize distance increase (TSP-like) 
        BUT with Time Window constraint check.
        """
        if task in sequence:
            return float('inf'), -1
            
        best_marginal = float('inf')
        best_idx = -1
        
        # Try inserting at every position to find min distance increase
        # But wait! "Baseline... no optimization". 
        # "UAV is assigned to one with least distance".
        # This implies: Select unassigned task closest to ME (current end of path).
        # So we only check appending to end?
        # Appending to end is the "pure greedy" strategy (Nearest Neighbor).
        # Let's do Nearest Neighbor (Append only).
        
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
                
                # Check marginal (distance)
                # We only support appending (from calculate_marginal_significance logic above)
                dist, pos = self.calculate_marginal_significance(task, self.tasks)
                
                if dist == float('inf'):
                    continue
                
                # Check against current belief (Consensus)
                # If someone else claims it with lower distance?
                # Q stores distance now.
                if dist >= self.Q[task_id]:
                    continue
                    
                if dist < min_dist:
                    min_dist = dist
                    best_task = task
            
            if best_task:
                # Add it
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
        # Baseline might not even have "secondary inclusion" or "reallocation".
        # But to be fair comparison of *metrics*, let's allow it to try picking up leftovers
        # using same greedy logic.
        return self.task_inclusion()
