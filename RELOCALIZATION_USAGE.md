# FAST-LIO Relocalization Usage

本文档说明当前 FAST-LIO 重定位链路的启动、请求触发、结果查看、调试话题和常用参数。

## 1. 功能概览

当前重定位链路用于把 FAST-LIO 的实时局部点云对齐到先验 PCD 地图，并把结果通过 `/reloc/cloud_align` 交给 FAST-LIO。FAST-LIO 收到后会重置 EKF 位姿和局部地图。

```text
FAST-LIO
  -> /cloud_registered_body
  -> icp_relocalizer
  -> /reloc/cloud_align
  -> FAST-LIO reset EKF + rebuild local map
```

重定位节点内部流程：

```text
多帧静止点云累计
  -> 时间一致性过滤
  -> Coarse ICP
  -> 地图一致性过滤
  -> Fine GICP
  -> overlap / 姿态 / 修正量验收
  -> 发布 /reloc/cloud_align
```

## 2. 启动

编译并 source：

```bash
colcon build --packages-select fast_lio
source install/setup.bash
```

启动完整重定位链路：

```bash
ros2 launch fast_lio reloc_mid360.launch.py
```

这个 launch 会启动：

```text
fastlio_mapping
pcd_to_occupancy_map
icp_relocalizer
rviz2
```

如果不需要 RViz：

```bash
ros2 launch fast_lio reloc_mid360.launch.py use_rviz:=false
```

如果临时不启动重定位节点：

```bash
ros2 launch fast_lio reloc_mid360.launch.py start_relocalizer:=false
```

如果使用指定先验 PCD：

```bash
ros2 launch fast_lio reloc_mid360.launch.py map_pcd:=/absolute/path/to/scans.pcd
```

## 3. 自动重定位和外部请求

配置文件：

```text
src/fast_lio/config/reloc/relocalization.yaml
```

默认配置：

```yaml
request_topic: "/reloc/request"
one_shot: true
auto_relocalize_on_start: true
```

含义：

- `auto_relocalize_on_start: true`：启动后自动累计点云并尝试重定位一次。
- `one_shot: true`：成功 ACCEPT 并发布结果后停止后续自动匹配。
- 外部请求仍然可以重新唤醒重定位流程。

外部触发一次重定位：

```bash
ros2 topic pub --once /reloc/request std_msgs/msg/Empty "{}"
```

收到请求后，`icp_relocalizer` 会：

```text
清空旧累计点云
解除 one_shot 停止状态
立即读取当前 FAST-LIO odom -> base_link 作为本轮初值
重新累计点云
运行同一套 Coarse ICP + Fine GICP 流程
成功后发布 /reloc/cloud_align
```

如果只想手动请求，不想启动后自动重定位：

```yaml
auto_relocalize_on_start: false
```

## 4. 查看重定位结果

查看重定位节点输出：

```bash
ros2 topic echo /reloc/cloud_align
```

只看一条：

```bash
ros2 topic echo --once /reloc/cloud_align
```

`/reloc/cloud_align` 类型是：

```text
geometry_msgs/msg/PoseStamped
```

含义：

```text
header.frame_id = odom
pose = base_link 在 odom/map 坐标系下的重定位位姿
```

确认 FAST-LIO 是否吃到结果：

```bash
ros2 topic echo /Odometry
ros2 run tf2_ros tf2_echo odom base_link
```

重定位成功后，`/Odometry` 和 `odom -> base_link` 会跳到 `/reloc/cloud_align` 对应的位置附近。

## 5. RViz 调试点云

打开 RViz 后可以添加这些 PointCloud2：

```text
/reloc/debug/accumulated_raw
/reloc/debug/time_filtered
/reloc/debug/coarse_aligned
/reloc/debug/map_consistent
/reloc/debug/fine_aligned
```

建议同时查看：

```text
/map
/cloud_registered
/reloc/debug/fine_aligned
```

各调试点云含义：

| 话题 | 含义 |
| --- | --- |
| `/reloc/debug/accumulated_raw` | 多帧累计后的原始局部点云，frame 为 `base_link` |
| `/reloc/debug/time_filtered` | 时间一致性过滤后的局部点云，frame 为 `base_link` |
| `/reloc/debug/coarse_aligned` | 粗配准后投到 `odom` 下的点云 |
| `/reloc/debug/map_consistent` | 粗配准后通过地图一致性过滤的点云 |
| `/reloc/debug/fine_aligned` | 精配准后最终投到 `odom` 下的点云 |

如果 `/reloc/debug/fine_aligned` 和先验地图明显错位，但仍然 ACCEPT，需要收紧验收参数。

## 6. 日志判断

成功时日志会出现：

```text
粗配准: ...
地图一致性过滤: ...
精配准: ...
ACCEPT: ...
重定位成功：...
已发到 /reloc/cloud_align
```

失败时不会发布 `/reloc/cloud_align`，会打印类似：

```text
REJECT: Coarse ICP 未收敛
REJECT: Coarse ICP 残差 ...
REJECT: 地图一致点 ... < ...
REJECT: Fine GICP 未收敛
REJECT: Fine GICP 残差 ...
REJECT: overlap ... 不足
REJECT: roll/pitch ...
REJECT: 相对初值平移 ...
REJECT: yaw 修正 ...
```

初值日志：

```text
本轮自动初值[FAST-LIO odom TF]: ...
请求初值: ...
```

如果看到 `YAML fallback`，说明触发时没有拿到 `odom -> base_link` TF，会退回 `initial_x/y/z/yaw`。

## 7. 常用参数

主要参数都在：

```text
src/fast_lio/config/reloc/relocalization.yaml
```

粗配准：

```yaml
coarse_map_voxel: 0.3
coarse_source_voxel: 0.3
coarse_max_correspondence_distance: 1.5
coarse_fitness_threshold: 0.7
```

精配准：

```yaml
fine_map_voxel: 0.1
fine_source_voxel: 0.1
fine_max_correspondence_distance: 0.5
fine_fitness_threshold: 0.25
gicp_correspondence_randomness: 20
```

多帧累计：

```yaml
accumulate_frames: 30
startup_delay: 5.0
```

动态点和地图一致性过滤：

```yaml
time_consistency_filter_en: true
time_consistency_voxel: 0.15
time_consistency_min_ratio: 0.35
time_consistency_min_frames: 3
map_consistency_distance: 0.5
```

最终验收：

```yaml
overlap_distance: 0.5
min_overlap_ratio: 0.35
min_correspondences: 300
max_roll_pitch_deg: 10.0
max_correction_dist: 2.0
max_yaw_correction_deg: 35.0
```

如果重定位经常误接受，优先收紧：

```yaml
min_overlap_ratio
min_correspondences
fine_fitness_threshold
max_yaw_correction_deg
max_correction_dist
```

如果重定位经常拒绝但 RViz 看起来匹配没问题，优先放宽：

```yaml
min_overlap_ratio
min_correspondences
fine_fitness_threshold
map_consistency_distance
```

## 8. 推荐验证流程

启动监听：

```bash
ros2 topic echo /reloc/cloud_align
```

启动系统：

```bash
ros2 launch fast_lio reloc_mid360.launch.py
```

手动触发：

```bash
ros2 topic pub --once /reloc/request std_msgs/msg/Empty "{}"
```

查看 FAST-LIO 输出：

```bash
ros2 topic echo /Odometry
ros2 run tf2_ros tf2_echo odom base_link
```

在 RViz 中确认：

```text
/map
/cloud_registered
/reloc/debug/fine_aligned
```

## 9. 注意事项

- 重定位期间 FAST-LIO 仍然正常运行，并持续发布 `/cloud_registered_body`。
- 当前 `/reloc/cloud_align` 仍然会触发 FAST-LIO reset，不是后端优化因子。
- 重定位时机器人最好保持静止，否则时间一致性过滤和多帧累计会变差。
- `map -> odom` 当前由 `icp_relocalizer` 发布为单位阵，设计含义是重定位后 `odom` 与先验 `map` 对齐。
- 不要同时启动 `mapping_mid360.launch.py` 和 `reloc_mid360.launch.py`，否则可能出现重复 FAST-LIO 节点、话题和 TF 冲突。

