import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import math

class WallFollowingObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('wall_following_obstacle_avoidance')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.twist = Twist()
        self.avoiding_obstacle = False
        self.following_wall = False
        self.initial_heading = None  # Will store initial yaw angle
        self.current_heading = 0.0  # Track current yaw angle

    def odom_callback(self, msg):
        # Extract yaw from the Odometry quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_heading = yaw

        if self.initial_heading is None:
            self.initial_heading = yaw  # Store the initial heading

    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to Euler angles."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def scan_callback(self, msg):
        # Define three regions based on the laser scan ranges
        left_distances = msg.ranges[45:135]  # Left side
        front_distances = msg.ranges[0:45]  # Front
        right_distances = msg.ranges[135:179]  # Right side

        min_left = min(left_distances) if left_distances else float('inf')
        min_front = min(front_distances) if front_distances else float('inf')
        min_right = min(right_distances) if right_distances else float('inf')

        obstacle_distance = 1.0
        wall_following_distance = 0.5

        if min_front <= obstacle_distance:
            self.avoiding_obstacle = True
            self.following_wall = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5

        elif self.avoiding_obstacle and self.following_wall:
            if min_left <= wall_following_distance:
                self.twist.linear.x = 0.3
                self.twist.angular.z = -0.2
            elif min_left > wall_following_distance:
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0.2
            else:
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0.0

            if min_front > obstacle_distance and min_left > obstacle_distance:
                self.avoiding_obstacle = False
                self.following_wall = False

        else:
            self.twist.linear.x = 0.3
            self.twist.angular.z = self.correct_heading()

        self.cmd_pub.publish(self.twist)

    def correct_heading(self):
        """Correct the robot's heading to align with the initial heading."""
        if self.initial_heading is not None:
            heading_error = self.initial_heading - self.current_heading
            return 0.5 * heading_error  # Proportional gain factor
        return 0.0

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

# Main function to run the node
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingObstacleAvoidance()
    trajectory_tracker = TrajectoryTracker()
    odom_sub = node.create_subscription(
        Odometry,                        # Message type
        'odom',                          # Topic name
        trajectory_tracker.odom_callback, # Callback function
        10                               # QoS (queue size)
    )
    try:
        rclpy.spin(node)
        trajectory_tracker.plot_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
