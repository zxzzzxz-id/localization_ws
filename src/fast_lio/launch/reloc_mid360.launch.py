from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
    params.update(flatten_yaml(package_dir / "config" / "reloc" / "mid360.yaml"))
    # Localization must never overwrite the prior map on shutdown.
    params["pcd_save/pcd_save_en"] = False
    relocalization_params = str(
        package_dir / "config" / "reloc" / "relocalization.yaml")
    map_pcd = LaunchConfiguration("map_pcd")
    start_map = LaunchConfiguration("start_map")
    start_relocalizer = LaunchConfiguration("start_relocalizer")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Grid mapping is optional: standalone localization must work without it.
    try:
        pcd_package_dir = Path(get_package_share_directory("pcd_to_occupancy_map"))
    except PackageNotFoundError:
        grid_action = LogInfo(
            msg="pcd_to_occupancy_map not found; skipping optional grid map node.",
            condition=IfCondition(start_map),
        )
    else:
        grid_action = Node(
            package="pcd_to_occupancy_map",
            executable="pcd_to_occupancy_map",
            name="pcd_to_occupancy_map",
            output="screen",
            parameters=[
                str(pcd_package_dir / "config" / "pcd_to_occupancy_map.yaml"),
                {"pcd_file": map_pcd, "use_sim_time": use_sim_time},
            ],
            condition=IfCondition(start_map),
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 同 mapping：base_link -> livox_frame -> imu_link 由 fastlio_mapping 按 YAML
        # 外参发布；icp_relocalizer 收到 /cloud_registered_body（imu_link 系）后靠这条
        # TF 变换到 base_link，因此不再需要 robot_state_publisher / URDF。
        DeclareLaunchArgument(
            "map_pcd",
            default_value=str(package_dir / "PCD" / "scans.pcd"),
            description="先验 PCD 地图路径"),
        DeclareLaunchArgument(
            "start_map", default_value="true",
            description="是否启动栅格图节点（自动检测包，缺少时跳过）"),
        DeclareLaunchArgument(
            "start_relocalizer", default_value="true",
            description="是否启动 ICP 重定位节点"),
        # fast_lio 本体：mapping 配置叠加 reloc 配置（reloc_en=true）
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fast_lio",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
        ),
        # 先验 PCD -> /map；未安装栅格包时只提示并跳过。
        grid_action,
        # ICP 重定位 -> /reloc/cloud_align
        Node(
            package="fast_lio",
            executable="icp_relocalizer",
            name="icp_relocalizer",
            output="screen",
            parameters=[relocalization_params, {"pcd_file": map_pcd, "use_sim_time": use_sim_time}],
            condition=IfCondition(start_relocalizer),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(package_dir / "rviz_cfg" / "reloc_mid360.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
