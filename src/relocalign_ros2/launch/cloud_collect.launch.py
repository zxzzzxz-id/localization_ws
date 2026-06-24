import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('relocalign_ros2'),
        'config',
        'cloud_collect.yaml'
    )

    cloud_topic = LaunchConfiguration('cloud_topic')
    pcd_path = LaunchConfiguration('pcd_path')

    cloud_collect_node = Node(
        package='relocalign_ros2',
        executable='cloud_collect_node',
        name='cloud_collect',
        output='screen',
        parameters=[
            config_path,
            {
                'cloud_topic': cloud_topic,
                'pcd_path': pcd_path,
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('pcd_path', default_value='/tmp/cloud_collect.pcd'),
        cloud_collect_node
    ])
