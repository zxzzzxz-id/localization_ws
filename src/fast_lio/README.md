# FAST-LIO：MID-360 建图与定位

配准、定位、IMU 处理和自车过滤采用 `localization_self_ws/src/fast_lio` 当前源码。
独立工作空间只需 FAST-LIO 和雷达驱动；栅格与坡道/台阶识别是导航集成时可选的 `pcd_to_occupancy_map` 包功能。
建图与定位接口、配置和运行方式如下。

## 构建与启动

```bash
cd /home/zxz/workspace/fast_lio_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 建图：退出时保存 src/fast_lio/PCD/scans.pcd
ros2 launch fast_lio mapping_mid360.launch.py

# 定位：与建图二选一；不会在退出时覆盖先验图
ros2 launch fast_lio reloc_mid360.launch.py \
  map_pcd:=/home/zxz/workspace/fast_lio_ws/src/fast_lio/PCD/scans.pcd
```

雷达驱动需提供 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）和 `/livox/imu`。
原始 `/livox/lidar/pointcloud` 由地面分割使用。两种模式均支持 `use_rviz:=false`；
回放需加 `use_sim_time:=true` 并通过 `ros2 bag play <bag> --clock` 提供时钟。
定位默认 `start_map:=true`，自动检测栅格包：存在则启动，缺少则提示并跳过，不影响定位和重定位。
`start_map:=false` 可手动关闭栅格节点，`start_relocalizer:=false` 可关闭 ICP 节点。
雷达驱动需另行启动：`ros2 launch livox_ros_driver2 msg_MID360_launch.py`。
导航集成时，安装并 source `pcd_to_occupancy_map` 包后，该节点会随定位启动。

## 配置与数据流

| 功能 | 配置 |
| --- | --- |
| 输入、外参、IMU 噪声、建图与自车过滤 | `config/mapping/mid360.yaml` |
| 开启重定位输入 | `config/reloc/mid360.yaml` |
| 多帧配准与验收门限 | `config/reloc/relocalization.yaml` |
| 栅格与坡道/台阶识别 | `../pcd_to_occupancy_map/config/pcd_to_occupancy_map.yaml` |

自车过滤将各输入分支的点通过完整雷达外参转换到 `base_link`，删除
`box_min=[-0.4,-0.4,-0.05]`、`box_max=[0.4,0.4,1.45]` 内的点。已移除球形盲区：盒外有限点不再因为离雷达较近而被删除。
box 是当前机器人的过滤包络，换车或改安装时需要核对尺寸与外参，不依赖 URDF。

定位流程：静止累积 30 个非空帧 → 时间一致性过滤 → 粗 ICP → 先验地图一致性过滤 →
精 GICP → 验收。有效点不足时保留点云再追加 20 帧，仍不足则清空重试。
默认启动等待 5 秒、yaw 修正上限 35°、平移修正上限 2 m、重叠率至少 0.35、对应点至少 300。
它使用附近初值做局部配准，累积期间应保持静止。

成功时向 `/reloc/cloud_align` 发布 `geometry_msgs/msg/PoseStamped`，FAST-LIO 重置
EKF、局部地图和高频预测状态，并同步旋转世界系速度与重力。IMU 再初始化也考虑当前姿态。
默认 `one_shot: true`。需要再次定位时：

```bash
ros2 topic pub --once /reloc/request std_msgs/msg/Empty '{}'
```

`auto_relocalize_on_start: false` 可改为仅接受外部触发。每次外部触发都清空累积并重新取初值。
调试点云位于 `/reloc/debug/{accumulated_raw,time_filtered,coarse_aligned,map_consistent,fine_aligned}`。

## 坐标系与输出

约定 `T_B^A` 将 B 系点转换到 A 系。YAML 内部外参为 `T_imu^lidar`，安装外参为
`T_lidar^base`；代码计算 `T_base^odom = T_imu^odom (T_imu^lidar)^-1 (T_lidar^base)^-1`。

```text
map -> odom -> base_link -> livox_frame -> imu_link
           \-> base_link_hf
```

`fastlio_mapping` 发布机器人中心 `/Odometry`、200 Hz 目标节拍的 `/OdometryHighFreq`、
`/cloud_registered`、`/cloud_registered_body` 及上述机器人/传感器 TF。
`zero_at_start: true` 使首帧机器人中心位姿为零。

重定位通过状态重置把 odom 对齐到先验图，`icp_relocalizer` 发布单位 `map -> odom`；
这条静态 TF 在启动时就发布，它存在不代表已经定位成功。成功需要检查 `ACCEPT` 日志和
`/reloc/cloud_align`。该实现没有连续回环优化。

`reloc_mid360.launch.py` 启动定位、重定位及 RViz，并在栅格包存在且 `start_map:=true` 时
启动栅格节点；栅格节点和配准节点共用 `map_pcd`。可选的独立栅格包输出 `/map`、`/terrain/type`、`/terrain/direction`、`/terrain/colored`，
参数及地形算法保留本工作区原实现。`fast_lio/src/pcd_to_occupancy_map.cpp` 是未参与构建的
历史文件；不要将其加入 fast_lio 的构建，否则容易混淆两个栅格实现。

## 检查

```bash
ros2 topic hz /Odometry
ros2 topic hz /OdometryHighFreq
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /reloc/cloud_align --once
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
```

内置 `imu_gravity_reinitialization` 覆盖正常、倒装、倾斜及多次重置的静止 IMU 初始化和传播。
测试通过仅证明相应回归用例，实车配准精度仍需用真实地图与传感器数据验证。

## 本机验证（2026-10-25）

- 整个工作区 6 个包编译通过。地面分割库及 ROS 节点分别补充 `find_package(Boost REQUIRED COMPONENTS date_time)`，消除缺少 `Boost::date_time` 导入目标的错误。
- IMU 回归测试 1 项通过，覆盖 7 种姿态/重复重置情况。
- 隔离 ROS domain 的合成配准检查通过：稀疏点云追加后拒绝、已知位姿配准、one_shot 阻止重复输出、外部请求重新触发。
- 原栅格包在含坡道的合成 PCD 上成功输出 `/map`、`/terrain/type`、`/terrain/direction` 和 `/terrain/colored`；三张栅格几何一致（140×76，0.05 m）。
- 完整定位 launch 输入 99 帧合成 Livox 和 1992 条 IMU，输出 96 条 `/Odometry`、1930 条 `/OdometryHighFreq`、94 帧 body 点云和 1 次成功重定位。退出时三个节点均正常结束，原先验 PCD 哈希不变。
- 为修复测试发现的 Ctrl+C 析构崩溃，FAST-LIO 在高频线程结束后主动释放全局预处理发布器并调用 `rclcpp::shutdown()`。这部分生命周期处理是与来源算法代码之外的附加修正。
- SHA-256 校验确认两个 `pcd_to_occupancy_map.cpp`、独立栅格包源码和配置、原先验 PCD 以及导航/地面分割算法未改动；来源工作区未改动。

本机地面分割链接仍有 PCL 1.12/1.15 混用警告，尚未验证实车地面分割或导航性能。
上述合成实验用于接口和回归验证，不代表实车定位精度。
原文件与旧 build/install 备份、测试脚本和日志位于 `/tmp/fastlio_nav2_migration`（临时目录，需长期保留时另存）。

## 自车过滤修正

所有支持的点云输入分支（含特征提取）统一在构造特征前过滤车体点，并对最终面点/角点再次检查，防止特征均值落回车体内。
不再读取 `preprocess/blind`，也不再按雷达距离删除点；NaN/Inf 坐标仍作为无效数据剔除。
配置中的包围盒必须含有限数值且逐轴 min < max，否则启用过滤时节点返回失败。
特征提取补充空扫描线和邻域边界保护；零方向向量仅在需要计算几何方向的特征步骤中跳过。

本次修正验证：`colcon build --packages-select fast_lio` 通过；CTest 的 IMU 回归与自车预处理两项测试通过，其中自车过滤包含 45 项检查。
另有 5 类非法启动参数检查通过。完整合成 LiDAR/IMU 定位链路输出 96 条 `/Odometry`、1927 条 `/OdometryHighFreq`，接受 1 次重定位，正常结束。
上述结果未包含实车精度验证。当前 `self_filter` 的包围盒数值保持原配置，独立栅格包及两个 `pcd_to_occupancy_map.cpp` 未修改。
