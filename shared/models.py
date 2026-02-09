
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
