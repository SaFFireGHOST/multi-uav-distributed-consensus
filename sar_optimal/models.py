import math
import sys
import os

# Import the distance function from the root shared folder
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from shared.utils import calculate_distance

class Task:
    """
    Search and Rescue Task Definition.
    Fully constrained by Causality (predecessors) and Environment limits (deadline).
    """
    def __init__(self, task_id, location, duration, deadline=float('inf'), predecessors=None):
        self.id = task_id
        self.location = location
        self.duration = duration
        
        # execution limits
        self.deadline = deadline
        self.predecessors = predecessors if predecessors is not None else []
        
    def __repr__(self):
        deps = f", deps={self.predecessors}" if self.predecessors else ""
        dl = f", deadline={self.deadline}" if self.deadline != float('inf') else ""
        return f"Task(id={self.id}, dur={self.duration}{deps}{dl})"

class UAV:
    """
    Simplified Drone Agent for the Centralized Optimal Algorithm.
    Since scheduling is centralized, we do not need distributed consensus variables (Z, Q, T).
    """
    def __init__(self, uav_id, location, velocity):
        self.id = uav_id
        self.location = location
        self.velocity = velocity
        
        # Pi: Ordered list of assigned tasks (Task objects)
        self.tasks = [] 
        
    def __repr__(self):
        return f"UAV(id={self.id}, tasks={[t.id for t in self.tasks]})"
