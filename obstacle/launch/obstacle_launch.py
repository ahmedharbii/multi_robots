from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle',
            namespace='obstacle',
            executable='odom',
            name='odom'
        ),
        Node(
            package='obstacle',
            namespace='obstacle',
            executable='lidar',
            name='lidar'
        ),
        # Node(
        #     package='obstacle',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])