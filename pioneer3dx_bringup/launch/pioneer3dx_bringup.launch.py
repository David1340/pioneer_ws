from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    lidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a2m8_launch.py'
    )

    description_launch = os.path.join(
        get_package_share_directory('pioneer3dx_description'),
        'launch',
        'rsp.launch.py'
    )

    return LaunchDescription([
        Node(
            package='rosaria',
            executable='RosAria',
            name='rosaria',
            namespace='pioneer3DX',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch)
        )

    ])

