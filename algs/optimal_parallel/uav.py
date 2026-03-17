
from shared.utils import calculate_distance

class OptimalUAV:
    """
    Minimal UAV class for centralized optimal allocation.
    Used only as a data container - all logic is in CentralizedOptimalRunner.
    """
    def __init__(self, uav_id, location, velocity, capacity, fuel, fuel_consumption_rate, all_tasks):
        self.id = uav_id
        self.location = location
        self.velocity = velocity
        self.capacity = capacity
        self.fuel = fuel
        self.fuel_consumption_rate = fuel_consumption_rate
        self.min_fuel_threshold = 0
        
        self.all_tasks = {t.id: t for t in all_tasks}
        self.tasks = []

    def _calculate_times_for_sequence(self, sequence):
        """Calculate start times for a task sequence."""
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
        """Calculate total completion time cost."""
        times = self._calculate_times_for_sequence(sequence)
        cost = 0
        for i, task in enumerate(sequence):
            completion_time = times[i] + task.duration
            cost += completion_time
        return cost
    
    def is_feasible(self, task):
        """Check if adding task to end of sequence is feasible (time windows)."""
        new_seq = self.tasks + [task]
        times = self._calculate_times_for_sequence(new_seq)
        
        for i, t in enumerate(new_seq):
            if not (t.time_window[0] <= times[i] <= t.time_window[1]):
                return False
        return True
