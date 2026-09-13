from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def flatten_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    result = {}

    def visit(value, prefix=""):
        for key, child in value.items():
            name = f"{prefix}/{key}" if prefix else key
            if isinstance(child, dict):
                visit(child, name)
            else:
                result[name] = child

    visit(data)
    return result


def generate_launch_description():
    package_dir = Path(get_package_share_directory("fast_lio"))
    params = flatten_yaml(package_dir / "config" / "mapping" / "mid360.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 自车几何滤波（self_filter）靠 /robot_description 拿机器人几何，
        # 所以默认把 robot_state_publisher 一起拉起来，避免漏启动导致滤波静默失效。
        # 如果链路脚本里已经单独开了 01_robot_state_publisher，就把这个设成 false。
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true",
            description="是否同时启动 robot_state_publisher（self_filter 需要 /robot_description）"),
        Node(
            package="robot_description",
            executable="start_robot_state_publisher.py",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fast_lio",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(package_dir / "rviz_cfg" / "mapping_mid360.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
