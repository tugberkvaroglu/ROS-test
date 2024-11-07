import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

class TrajectoryTracker:
    def __init__(self):
        self.positions = []  # Store robot's x, y positions
    
    def odom_callback(self, msg):
        # Extract x and y position from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions.append((x, y))

    def plot_trajectory(self):
        if len(self.positions) > 1:
            positions = np.array(self.positions)
            plt.figure(figsize=(8, 6))
            plt.plot(positions[:, 0], positions[:, 1], label='Robot Trajectory')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('Robot Trajectory in Simulation')
            plt.legend()
            plt.grid(True)
            plt.show()
        else:
            print("Not enough data to plot trajectory.")
