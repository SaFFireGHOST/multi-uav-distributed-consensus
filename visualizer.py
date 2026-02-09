
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def visualize_simulation(uavs, tasks):
    """
    Visualize the UAV paths and tasks using Matplotlib Animation.
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Setup static elements
    # Tasks
    for t in tasks:
        ax.scatter(t.location[0], t.location[1], t.location[2], c='red', marker='x', s=50)
        ax.text(t.location[0], t.location[1], t.location[2], f'T{t.id}\n[{t.time_window[0]}-{t.time_window[1]}]', fontsize=8)
        
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('UAV Task Allocation Simulation')
    
    # Pre-calculate interpolated paths for smooth animation
    # We need a common time axis.
    # Max time?
    max_time = 0
    uav_schedules = []
    
    colors = plt.cm.jet(np.linspace(0, 1, len(uavs)))
    
    for uav in uavs:
        # Schedule: list of (position, time) tuples
        # Start
        schedule = [(uav.location, 0)]
        
        # We need precise arrival/departure times
        # Recalculate based on assignments
        curr_time = 0
        curr_loc = uav.location
        
        # Calculate times again locally to be sure
        times = uav._calculate_times_for_sequence(uav.tasks)
        
        for i, task in enumerate(uav.tasks):
            start_time = times[i]
            # Flight phase: curr_loc -> task.location
            # Arrival at start_time.
            # But wait! If start_time > arrival_time (waiting), we hold position?
            dist = np.linalg.norm(np.array(uav.location) - np.array(task.location)) # Initial estimate
            
            # Let's break down path segments
            # Segment 1: Fly to task
            # Segment 2: Execute task (Duration)
            
            # Fly
            schedule.append((task.location, start_time))
            
            # Execute
            end_time = start_time + task.duration
            schedule.append((task.location, end_time))
            
            if end_time > max_time:
                max_time = end_time
                
            curr_time = end_time
            curr_loc = task.location
            
        uav_schedules.append(schedule)
        
    # Interpolate positions for frames
    # Frame rate = 10 frames per sim-second?
    fps = 10
    total_frames = int(max_time * fps)
    
    lines = [ax.plot([], [], [], label=f'UAV {u.id}', color=colors[i])[0] for i, u in enumerate(uavs)]
    points = [ax.plot([], [], [], marker='o', color=colors[i])[0] for i, u in enumerate(uavs)]
    
    # Trajectory history
    history_x = [[] for _ in uavs]
    history_y = [[] for _ in uavs]
    history_z = [[] for _ in uavs]

    def update(frame):
        current_sim_time = frame / fps
        ax.set_title(f"Time: {current_sim_time:.1f}s")
        
        for i, uav in enumerate(uavs):
            sched = uav_schedules[i]
            # Find relevant segment
            pos = sched[-1][0] # Default to end
            
            for j in range(len(sched) - 1):
                p1, t1 = sched[j]
                p2, t2 = sched[j+1]
                
                if t1 <= current_sim_time <= t2:
                    # Interpolate
                    if t2 == t1:
                        target = p2
                    else:
                        ratio = (current_sim_time - t1) / (t2 - t1)
                        # Linear interp
                        target = (
                            p1[0] + (p2[0] - p1[0]) * ratio,
                            p1[1] + (p2[1] - p1[1]) * ratio,
                            p1[2] + (p2[2] - p1[2]) * ratio
                        )
                    pos = target
                    break
            
            # Update history
            history_x[i].append(pos[0])
            history_y[i].append(pos[1])
            history_z[i].append(pos[2])
            
            # Update Line (Trail)
            lines[i].set_data(history_x[i], history_y[i])
            lines[i].set_3d_properties(history_z[i])
            
            # Update Point (Current Pos)
            points[i].set_data([pos[0]], [pos[1]])
            points[i].set_3d_properties([pos[2]])
            
        return lines + points

    ani = animation.FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False, repeat=False)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    pass
