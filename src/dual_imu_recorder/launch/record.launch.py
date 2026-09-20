from datetime import datetime
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


TOPICS = [
    f"/{name}/{topic}"
    for name in ("imu1", "imu2")
    for topic in ("imu/data", "imu/mag", "imu/temperature", "hipnuc/imu")
] + ["/diagnostics"]


def start_recording(context):
    port_1 = LaunchConfiguration("port_1").perform(context)
    port_2 = LaunchConfiguration("port_2").perform(context)
    if not port_1 or not port_2 or os.path.realpath(port_1) == os.path.realpath(port_2):
        raise RuntimeError("port_1 and port_2 must be different serial devices")

    bag_dir = Path(LaunchConfiguration("bag_dir").perform(context)).expanduser()
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_path = bag_dir / f"dual_imu_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"

    params_1 = LaunchConfiguration("params_file_1")
    params_2 = LaunchConfiguration("params_file_2")

    def imu_node(name, port, baudrate, frame_id, params_file):
        return Node(
            package="hipnuc_imu",
            executable="serial_node",
            namespace=name,
            name="hipnuc_serial",
            output="screen",
            parameters=[
                params_file,
                {
                    "port": port,
                    "baudrate": ParameterValue(LaunchConfiguration(baudrate), value_type=int),
                    "frame_id": ParameterValue(LaunchConfiguration(frame_id), value_type=str),
                    "use_sim_time": False,
                    "publish_imu": True,
                    "publish_mag": True,
                    "publish_temperature": True,
                    "publish_hipnuc": True,
                },
            ],
        )

    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--output", str(bag_path), *TOPICS],
        output="screen",
    )
    return [
        LogInfo(msg=f"Recording both IMUs to {bag_path}"),
        RegisterEventHandler(
            OnProcessExit(target_action=recorder, on_exit=[Shutdown(reason="rosbag recorder exited")])
        ),
        recorder,
        imu_node("imu1", port_1, "baudrate_1", "frame_id_1", params_1),
        imu_node("imu2", port_2, "baudrate_2", "frame_id_2", params_2),
    ]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("hipnuc_imu"), "config", "serial.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("port_1", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("port_2", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("baudrate_1", default_value="115200"),
            DeclareLaunchArgument("baudrate_2", default_value="115200"),
            DeclareLaunchArgument("frame_id_1", default_value="imu1_link"),
            DeclareLaunchArgument("frame_id_2", default_value="imu2_link"),
            DeclareLaunchArgument("params_file_1", default_value=default_params),
            DeclareLaunchArgument("params_file_2", default_value=default_params),
            DeclareLaunchArgument("bag_dir", default_value="~/imu_bags"),
            OpaqueFunction(function=start_recording),
        ]
    )
