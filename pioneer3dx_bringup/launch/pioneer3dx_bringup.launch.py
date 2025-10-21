from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosaria',
            executable='RosAria',
            name='rosaria',
            namespace='pioneer3DX',
            output='screen',
        ),
    ])
