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
        self.recovering_position = False  # New flag for recovery phase
        self.initial_heading = None  # Will store initial yaw angle
        self.current_heading = 0.0  # Track current yaw angle
        self.starting_x = None  # Track the robot's starting x position
        self.current_x = 0.0  # Track the current x position
        self.passed_starting_x = False  # Flag to track if the robot has passed the starting x position

    def odom_callback(self, msg):
        # Extract x, y, and yaw from the Odometry message
        self.current_x = msg.pose.pose.position.x  # Update current x position
        self.current_y = msg.pose.pose.position.y

        if self.starting_x is None:
            self.starting_x = self.current_x  # Store the starting x position

        # Update passed_starting_x flag if the robot crosses the starting x position
        if self.starting_x is not None and self.current_x > self.starting_x:
            self.passed_starting_x = True

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
        # Define regions based on the laser scan ranges
        front_distances = msg.ranges[0:60] + msg.ranges[300:359]  # Front view
        left_distances = msg.ranges[60:120]  # Left side
        right_distances = msg.ranges[240:300]  # Right side

        min_front = min(front_distances) if front_distances else float('inf')
        min_left = min(left_distances) if left_distances else float('inf')
        min_right = min(right_distances) if right_distances else float('inf')

        # Log the distances for debugging
        self.get_logger().info(f'Left: {min_left}, Front: {min_front}, Right: {min_right}')

        # Thresholds
        obstacle_distance = 0.8  # Reduced threshold for better reaction
        wall_following_distance = 0.7  # Desired distance for wall following
        y_return_threshold = 0.3  # Stricter constraint for returning to y-axis

        # Check if the robot should align with the y-axis
        if abs(self.current_y - 0.0) > y_return_threshold and not self.avoiding_obstacle and not self.following_wall:
            self.get_logger().info(f'Returning to y-axis. Current x: {self.current_x}, Current y: {self.current_y}')

            # Check if the robot is aligned with the y-axis
            heading_towards_y_axis = abs(self.current_heading) < 0.1 or abs(self.current_heading - math.pi) < 0.1

            if not heading_towards_y_axis:
                # Turn the robot to face the y-axis
                self.twist.linear.x = 0.0  # Stop linear movement while turning
                self.twist.angular.z = 0.5  # Rotate to align with y-axis
            else:
                # Move forward towards the y=0 axis
                self.twist.linear.x = 0.3  # Move forward
                self.twist.angular.z = 0.0  # Stop turning
        else:
            # Normal obstacle avoidance and wall-following behavior
            if min_front <= obstacle_distance:
                # Obstacle detected directly in front
                self.avoiding_obstacle = True
                self.following_wall = True
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.6  # Turn away from the obstacle (increased speed)
            elif self.avoiding_obstacle and self.following_wall:
                # Continue wall-following behavior
                if min_left <= wall_following_distance:
                    # Adjust to stay at a safe distance from the wall
                    self.twist.linear.x = 0.3
                    self.twist.angular.z = -0.2
                elif min_left > wall_following_distance:
                    # Steer closer to the wall
                    self.twist.linear.x = 0.3
                    self.twist.angular.z = 0.3
                else:
                    # Default forward movement with slight adjustment
                    self.twist.linear.x = 0.3
                    self.twist.angular.z = 0.2

                # Check if obstacle avoidance is complete
                if min_front > obstacle_distance and min_left > obstacle_distance:
                    self.avoiding_obstacle = False
                    self.following_wall = False
            else:
                # Normal movement when no obstacle is detected
                self.twist.linear.x = 0.4  # Slightly faster normal speed
                self.twist.angular.z = 0.0

        self.cmd_pub.publish(self.twist)



    def should_correct_trajectory(self, threshold=0.1):
        """
        Check if the robot is near or has passed the starting x position.
        :param threshold: Acceptable difference to consider "near"
        :return: True if near starting x or has passed it, False otherwise
        """
        return self.passed_starting_x or abs(self.current_x - self.starting_x) <= threshold

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
            plt.savefig("/home/tugberk/Desktop/Projects/ROS-test/src/drone_avoidance/plots/trajectory.png")
            plt.close()
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
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_tracker.plot_trajectory()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
