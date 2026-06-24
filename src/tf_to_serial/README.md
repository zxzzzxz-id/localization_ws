# tf_to_serial 标定与使用说明

这个包把 FAST-LIO 输出的 TF 位姿转换成机器人中心位姿，并通过串口发送给下位机。标定建议按下面顺序做：

1. IMU 重力对齐标定
2. LiDAR 到机器人旋转中心的旋转标定
3. LiDAR/SLAM XY 到机器人 XY 的直线标定
4. 把结果写回 `src/tf_to_serial/src/tf_to_serial_node.cpp`

## 准备

在工作区根目录执行：

```bash
cd /home/pi/workspace/localization
colcon build --packages-select tf_to_serial
source install/setup.bash
```

标定前确认 FAST-LIO 或定位节点已经在发布：

```bash
ros2 run tf2_ros tf2_echo camera_init body_hf
```

如果 TF 不存在，先检查雷达驱动、FAST-LIO 和 frame 名称。默认标定节点使用：

- `source_frame=camera_init`
- `target_frame=body_hf`

## 1. IMU 重力对齐标定

用途：求出雷达/IMU 安装姿态的 pitch 补偿，让软件中的 XY 平面尽量水平。

操作：

1. 把设备放稳，保持静止。
2. 确认 IMU 话题 `/livox_mid360_imu` 正常发布。
3. 运行：

```bash
ros2 run tf_to_serial imu_gravity_calibration
```

输出里重点看：

```text
total_stddev = ...
pitch = ... rad
NEW: q_fix_.setRPY(0.0, <pitch_rad>, 0.0);
```

`total_stddev` 越小越好。如果提示 IMU 数据不稳定，重新放稳设备再跑。

把输出的 pitch 写回主节点：

```cpp
q_fix_.setRPY(0.0, <pitch_rad>, 0.0);
```

当前代码位置：

```cpp
// src/tf_to_serial/src/tf_to_serial_node.cpp
q_fix_.setRPY(0.0, -(180 - 55.1425) * M_PI / 180.0, 0.0);
```

旋转标定和直线标定也有同样的 `gravity_fix_pitch_deg` 参数。如果临时测试不同 pitch，可以不改代码，直接用参数传入：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p gravity_fix_pitch_deg:=-124.8575
```

## 2. 旋转标定

用途：求 LiDAR 到机器人旋转中心的距离 `lidar2robot_dis_`，以及给直线标定使用的 `observable_phase_deg`。

运行：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration
```

操作：

1. 启动后先保持静止，等待 warmup 完成。
2. 看到 `Waiting for rotation to start...` 或 `WAIT_ROTATION` 后再开始转。
3. 让机器人尽量绕自身中心原地旋转。默认一圈以上即可；实际标定建议转 2-3 圈。
4. 转动过程要慢、连续，少平移，尽量不要中途急停或反向。

如果想强制多转几圈再输出结果，推荐这样运行：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p gravity_fix_pitch_deg:=-124.8575 \
  -p rotate_min_yaw_span_deg:=720.0 \
  -p rotate_max_samples:=9000 \
  -p rotate_best_extra_samples:=800
```

其中 `rotate_min_yaw_span_deg=720.0` 表示至少两圈；三圈可以改成 `1080.0`。如果转得很慢，需要继续增大 `rotate_max_samples`。

注意：ROS2 参数每一项前面都要加 `-p`。例如 `rotate_min_yaw_span_deg:=1080` 前面不能省略 `-p`。

结果示例：

```text
samples=2193 path_span=578.11 deg yaw_span=581.13 deg
lidar2robot_dis=0.470864 m
observable_phase_deg=128.056504
quality: radius_rms=0.011361 m phase_rms=3.999998 deg
```

判断标准：

- `path_span` 默认至少大于 `360 deg`；如果按多圈标定，建议达到 `720 deg` 或 `1080 deg`。
- `yaw_span` 应该和 `path_span` 接近，差很多说明 TF yaw 或轨迹不可靠。
- `radius_rms` 默认要求小于 `0.025 m`。
- `phase_rms` 默认要求小于 `4 deg`。
- 新版旋转标定会做鲁棒裁剪，并继续采样一段时间保存最佳结果；日志里的 `raw_phase_rms` 是裁剪前的原始值，`phase_rms` 是用于判定和输出的鲁棒值。

上面这组数据可以先用：半径 RMS 约 `1.1 cm`，转动角度也够；但 `phase_rms=3.999998 deg` 刚好贴着阈值，属于可用但不算很宽裕。建议继续跑直线标定，如果直线四段的一致性好，就采用；如果直线结果发散，再重新做旋转标定。

记下：

```text
lidar2robot_dis=0.470864
observable_phase_deg=128.056504
```

## 3. 直线标定

用途：求 SLAM XY 坐标和机器人 XY 坐标之间的平面旋转 `xy_rotation_deg_`，并结合旋转标定输出最终的 `lidar2robot_ang_`。

把旋转标定得到的 `observable_phase_deg` 传进去：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504
```

操作按日志提示完成四段：

1. `+X`：机器人沿自身正 X 方向直走，超过默认 `0.35 m` 后停稳。
2. `-X`：沿自身负 X 方向直走，停稳。
3. `+Y`：沿自身正 Y 方向直走，停稳。
4. `-Y`：沿自身负 Y 方向直走，停稳。

每一段都要“直走，然后停住”。程序会等到最近一小段位移足够小后才结束该段。日志中常见状态：

- `CONTINUE`：距离还不够。
- `STOP_TO_COMPLETE`：距离够了，请停稳。
- `NOT_STRAIGHT`：轨迹不够直。
- `TOO_LEFT` / `TOO_RIGHT`：相对期望方向偏得太多。
- `REVERSED`：方向反了。

最终重点看：

```text
xy_rotation_deg=...
lidar2robot_ang=...
quality: line_angle_rms=... deg
```

四段结果里的 `accepted=true` 越多越好，`line_angle_rms` 越小越好。正常建议控制在几度以内；如果超过很多，重跑直线标定。

如果场地不平或轨迹有噪声，可以临时放宽直线度：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p line_max_lateral_rms_m:=0.06 \
  -p line_max_lateral_abs_m:=0.15
```

## 4. 写回主节点

标定完成后修改：

```cpp
// src/tf_to_serial/src/tf_to_serial_node.cpp
const double xy_rotation_deg_ = <直线标定输出的 xy_rotation_deg>;
const double lidar2robot_dis_ = <旋转标定输出的 lidar2robot_dis>;
const double lidar2robot_ang_ = <直线标定输出的 lidar2robot_ang>;
```

例如使用你这次旋转标定结果时，距离先写成：

```cpp
const double lidar2robot_dis_ = 0.470864;
```

`xy_rotation_deg_` 和 `lidar2robot_ang_` 等直线标定结束后再填。

修改后重新编译：

```bash
colcon build --packages-select tf_to_serial
source install/setup.bash
```

## 5. 运行主节点

确认串口设备存在并且没有被其他程序占用后运行：

```bash
ros2 run tf_to_serial tf_to_serial_node
```

主节点当前固定使用 `/dev/ttyUSB0` 和 `115200`。如果串口号变化，需要改 `src/tf_to_serial/src/tf_to_serial_node.cpp` 中的串口配置。

主节点会在发送前叠加基准偏移，默认 `yaw` 基础偏移为 `-90°`。如果主控会通过同一串口回传目标帧，帧格式与发送帧一致：

```text
FF FE 01 + 5 个 float + AA DD
```

这 5 个 `float` 按顺序对应 `x, y, z, yaw, pitch`。节点收到后，会把“收到那一刻的原始输出”作为参考点，之后按

```text
当前输出 - 参考输出 + 主控目标输出 + 基准偏移
```

来发送。

## 常用调参

旋转标定启动太慢：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p rotate_start_min_yaw_deg:=1.5 \
  -p rotate_start_min_motion_m:=0.01
```

旋转标定质量要求太严：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p rotate_max_radius_rms_m:=0.035 \
  -p rotate_max_phase_rms_deg:=5.0
```

旋转标定想多采样一段，避免刚过阈值就结束：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p rotate_best_extra_samples:=600 \
  -p rotate_target_phase_rms_deg:=2.0
```

旋转标定想强制多转两圈或三圈：

```bash
# 至少两圈
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p gravity_fix_pitch_deg:=-124.8575 \
  -p rotate_min_yaw_span_deg:=720.0 \
  -p rotate_max_samples:=9000 \
  -p rotate_best_extra_samples:=800

# 至少三圈
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p gravity_fix_pitch_deg:=-124.8575 \
  -p rotate_min_yaw_span_deg:=1080.0 \
  -p rotate_max_samples:=12000 \
  -p rotate_best_extra_samples:=1000
```

旋转标定中有少量跳点，可以调整鲁棒裁剪比例：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p rotate_radius_trim_ratio:=0.15 \
  -p rotate_phase_trim_ratio:=0.15
```

直线标定距离不够：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p line_min_distance_m:=0.25
```

直线标定停稳条件太严：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p line_stop_motion_m:=0.025
```

## 建议记录

每次标定建议记录下面几项，方便回溯：

```text
IMU:
  gravity_fix_pitch_deg =

Rotation:
  lidar2robot_dis =
  observable_phase_deg =
  radius_rms =
  phase_rms =

Line:
  xy_rotation_deg =
  lidar2robot_ang =
  line_angle_rms =
```
