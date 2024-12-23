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
        # Define three regions based on the laser scan ranges
        left_distances = msg.ranges[60:135]  # Left side
        front_distances = msg.ranges[0:60]  # Front
        right_distances = msg.ranges[135:179]  # Right side

        min_left = min(left_distances) if left_distances else float('inf')
        min_front = min(front_distances) if front_distances else float('inf')
        min_right = min(right_distances) if right_distances else float('inf')

        # Logging the distances for debugging
        self.get_logger().info(f'Left: {min_left}, Front: {min_front}, Right: {min_right}')

        obstacle_distance = 1.0

        if min_front <= obstacle_distance:
            self.avoiding_obstacle = True
            self.following_wall = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5

        elif self.avoiding_obstacle and self.following_wall:
            wall_following_distance = 0.6
            if min_left <= wall_following_distance:
                self.twist.linear.x = 0.3
                self.twist.angular.z = -0.2
            elif min_left > wall_following_distance:
                self.twist.linear.x = 0.4
                self.twist.angular.z = 0.3
            else:
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0.2

            if min_front > obstacle_distance and min_left > obstacle_distance:
                self.avoiding_obstacle = False
                self.following_wall = False

        else:
            if self.should_correct_trajectory():  # Check if the robot should correct its trajectory
                self.get_logger().info("Correcting Headding")
                self.twist.linear.x = 0.3
                self.twist.angular.z = self.correct_heading()
            else:
                self.get_logger().info("Going to the Y axis.")
                # Calculate the angle to the y-axis
                desired_heading = math.atan2(-self.current_y, 0)  # Heading towards y=0
                heading_error = desired_heading - self.current_heading

                # Apply proportional control for smoother turning
                # make the angular velocity descent
                angular_velocity = 0.3 * heading_error + 0.1 * self.twist.angular.z


                self.twist.linear.x = 0.3  # Maintain forward movement
                self.twist.angular.z = 0.0 if abs(angular_velocity) < 0.1 else angular_velocity


        self.cmd_pub.publish(self.twist)
    
    def should_correct_trajectory(self, threshold=0.05, threshold_exit=0.1):
        """
        Check if the robot is near or has passed the starting x position.
        :param threshold: Acceptable difference to consider "near" (entry threshold).
        :param threshold_exit: Larger acceptable difference to consider "far" (exit threshold).
        :return: True if correction is needed, False otherwise.
        """
        if self.passed_starting_x:
            # Allow correction for a short distance beyond the starting position
            if abs(self.current_x - self.starting_x) <= threshold_exit:
                self.get_logger().info("Passed starting x but still near. Correcting trajectory.")
                return True
            else:
                self.get_logger().info("Passed starting x and far away. No correction needed.")
                return False

        # Check if the robot is near the starting x position
        if abs(self.current_x - self.starting_x) <= threshold:
            self.get_logger().info("Near starting x. Correcting trajectory.")
            return True

        # Default case: No correction needed
        self.get_logger().info("No correction needed.")
        return False

    def correct_heading(self):
        """Correct the robot's heading to align with the initial heading using a gradient-descent-like approach."""
        if self.initial_heading is not None:
            heading_error = self.normalize_angle(self.initial_heading - self.current_heading)

            # Gradual reduction of correction (gradient descent-like)
            learning_rate = 0.3  # Determines the reduction rate for yaw correction
            correction = learning_rate * heading_error

            # Apply a maximum limit to correction to avoid aggressive turns
            max_correction = 0.1  # Limit the maximum yaw correction
            correction = max(-max_correction, min(correction, max_correction))

            return correction

        return 0.0

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

class TrajectoryTracker:
    def __init__(self):
        self.correction_timeout = 2.0  # Maximum time to stay in trajectory correction mode
        self.correction_start_time = None
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