import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 参数文件位于当前包的 share/config 目录
    config_path = os.path.join(
        get_package_share_directory('relocalign_ros2'),
        'config',
        'cloud_collect.yaml'
    )

    cloud_collect_node = Node(
        package='relocalign_ros2',
        executable='cloud_collect_node',
        name='cloud_collect',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([
        cloud_collect_node
    ])
