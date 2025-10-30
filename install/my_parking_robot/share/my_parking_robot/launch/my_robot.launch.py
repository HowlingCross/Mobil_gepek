from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_parking_robot',
            executable='my_parking_robot',
            name='my_parking_robot',
            output='screen'
        )
    ])

