import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('relocalign_ros2')

    default_config_path = os.path.join(pkg_share, 'config', 'relocalign.yaml')

    config_path = LaunchConfiguration('config_path')
    map_path = LaunchConfiguration('map_path')
    cloud_topic = LaunchConfiguration('cloud_topic')
    is_livox_custom = LaunchConfiguration('is_livox_custom')
    frame_count = LaunchConfiguration('frame_count')

    reloc_node = Node(
        package='relocalign_ros2',
        executable='reloc_livox_node',
        name='relocalign_pub_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'map_path': map_path,
            'cloud_topic': cloud_topic,
            'is_livox_custom': is_livox_custom,
            'frame_count': frame_count
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config_path),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('is_livox_custom', default_value='false'),
        DeclareLaunchArgument('frame_count', default_value='5'),
        reloc_node,
    ])
