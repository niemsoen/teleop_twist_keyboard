from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_launched',
            output='screen',
            prefix='xterm -e',
            parameters=[
                {'speed': 3.141},
                {'turn': 3.141}
            ]
        )
    ])

