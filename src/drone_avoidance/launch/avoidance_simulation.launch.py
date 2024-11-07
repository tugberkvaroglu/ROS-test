from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with the modified world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/tugberk/Desktop/ROS-test/src/drone_avoidance/worlds/obstacle_world.sdf'],
            output='screen'
        ),
        # Run the obstacle avoidance node
        Node(
            package='drone_avoidance',
            executable='object_avoidance',
            output='screen'
        ),
    ])
