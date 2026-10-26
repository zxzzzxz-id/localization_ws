# localization_ws

基于 Livox MID-360 与 ROS 2 FAST-LIO 的独立工作空间，支持建图、定位和先验地图重定位。

| 模块 | 功能 |
| --- | --- |
| [`src/fast_lio`](src/fast_lio/README.md) | 独立 Git 子模块；FAST-LIO、自车点云过滤与 ICP/GICP 重定位 |
| `src/livox_ros_driver2` | 雷达驱动，发布 `/livox/lidar`、`/livox/imu` |

主要输出：`/Odometry`、`/OdometryHighFreq`、`/cloud_registered`，以及机器人中心 TF。

## 环境与构建

Ubuntu 22.04、ROS 2 Humble；需安装 Livox-SDK2（头文件与动态库）、PCL、Eigen3、OpenMP、Boost 和 APR。

```bash
git clone --recurse-submodules https://github.com/zxzzzxz-id/localization_ws.git
cd localization_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

各终端先进入工作空间并执行 `source install/setup.bash`。驱动单独运行，建图与重定位二选一。

```bash
# 终端 1：雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2：建图；Ctrl+C 后保存 src/fast_lio/PCD/scans.pcd
ros2 launch fast_lio mapping_mid360.launch.py

# 或：使用已有地图定位、重定位；退出时不会覆盖地图
ros2 launch fast_lio reloc_mid360.launch.py \
  map_pcd:="$PWD/src/fast_lio/PCD/scans.pcd"
```

重定位默认自动触发一次，累积点云期间保持静止，起点应接近地图中的初始位置。
再次触发：`ros2 topic pub --once /reloc/request std_msgs/msg/Empty '{}'`。

有 `pcd_to_occupancy_map` 包时默认启动栅格节点，缺少时自动跳过；`start_map:=false` 可手动关闭。
两种模式均支持 `use_rviz:=false`；回放时加 `use_sim_time:=true`，配合 `ros2 bag play <bag> --clock`。
地图和录包不随 Git 分发，克隆后需自行准备。

## 常用配置

| 内容 | 文件 |
| --- | --- |
| 输入话题、安装外参、自车过滤、建图参数 | [`mid360.yaml`](src/fast_lio/config/mapping/mid360.yaml) |
| 重定位输入开关 | [`reloc/mid360.yaml`](src/fast_lio/config/reloc/mid360.yaml) |
| ICP/GICP、初值与验收门限 | [`relocalization.yaml`](src/fast_lio/config/reloc/relocalization.yaml) |
| 雷达 IP 与网络端口 | [`MID360_config.json`](src/livox_ros_driver2/config/MID360_config.json) |

## 子模块协作

更新工作空间后同步算法版本：

```bash
git pull --ff-only
git submodule update --init --recursive
```

算法改动在 `src/fast_lio` 内切到开发分支后提交、推送，再到外层提交、推送子模块版本指针。
导航仓库可引用同一个 [fast_lio 仓库](https://github.com/zxzzzxz-id/fast_lio)。
