from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    lidar_launch = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )


    return LaunchDescription([
        Node(
            package='rosaria',
            executable='RosAria',
            name='rosaria',
            namespace='pioneer3DX',
            output='screen',
        ),
    ]
)
