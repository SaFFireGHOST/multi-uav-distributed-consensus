import math

def calculate_distance(loc1, loc2):
    """
    Calculate Euclidean distance between two 3D points.
    loc1, loc2: tuples or lists (x, y, z)
    """
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(loc1, loc2)))

def calculate_arrival_time(start_time, duration, distance, velocity):
    """
    Calculate arrival time at next task.
    start_time: start time of previous task
    duration: duration of previous task
    distance: distance to next task
    velocity: UAV velocity
    """
    return start_time + duration + (distance / velocity)
