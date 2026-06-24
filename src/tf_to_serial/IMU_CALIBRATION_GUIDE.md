# tf_to_serial 标定指南

## 概述

`tf_to_serial` 包含三类标定工具：

1. `imu_gravity_calibration`：通过 IMU 加速度标定重力对齐 pitch。
2. `lidar_robot_rotation_calibration`：通过原地旋转标定 LiDAR 到机器人旋转中心的距离。
3. `lidar_robot_line_calibration`：通过四段直线运动标定 SLAM XY 到机器人 XY 的平面旋转。

建议顺序是：先做 IMU 重力标定，再做旋转标定，最后做直线标定。

## 通用准备

在工作区根目录编译并加载环境：

```bash
cd /home/pi/workspace/localization
colcon build --packages-select tf_to_serial
source install/setup.bash
```

标定前确认 TF 正常：

```bash
ros2 run tf2_ros tf2_echo camera_init body_hf
```

默认 frame：

- `source_frame=camera_init`
- `target_frame=body_hf`

如果你的实际 frame 不同，运行标定节点时用参数覆盖。

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p source_frame:=camera_init \
  -p target_frame:=body_hf
```

## 1. IMU 重力标定

### 工作原理

1. 订阅 `/livox_mid360_imu` 话题接收 IMU 加速度数据
2. 累积约 500 个样本（~5 秒）
3. 计算加速度的平均值和方差
4. 验证数据稳定性（方差 < 0.15 m/s²）
5. 基于重力方向计算欧拉角（roll 和 pitch）
6. 输出推荐的代码修改

### 使用步骤

#### 1. 准备设备
- 将设备放在稳定、水平的平面上
- 确保设备完全静止（不晃动）
- 保持此状态至少 5-10 秒

#### 2. 运行标定工具
```bash
ros2 run tf_to_serial imu_gravity_calibration
```

#### 3. 查看输出结果
工具会输出以下信息：
- `ax, ay, az`：平均加速度（应接近 9.807 m/s²）
- `σ(ax), σ(ay), σ(az)`：各轴加速度的标准差
- `total_stddev`：总体标准差（< 0.15 表示稳定）
- **`roll` 和 `pitch` 角度**（单位：度和弧度）

#### 4. 更新主节点代码
根据输出的推荐代码更新 `tf_to_serial_node.cpp`：

**原始代码：**
```cpp
q_fix_.setRPY(0.0, -(180 - 56.39) * M_PI / 180.0, 0.0);
```

**更新为：**
```cpp
// 替换为标定工具输出的值
q_fix_.setRPY(roll_rad, pitch_rad, 0.0);
```

例如，如果标定结果是 `roll = -0.015234 rad, pitch = 0.123456 rad`，则：
```cpp
q_fix_.setRPY(-0.015234, 0.123456, 0.0);
```

当前主节点实际只使用 pitch 补偿时，可以按节点输出的推荐行写：

```cpp
q_fix_.setRPY(0.0, pitch_rad, 0.0);
```

旋转标定和直线标定节点也带有同样的重力补偿参数。如果不想先改代码，可以临时传入：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p gravity_fix_pitch_deg:=-124.8575
```

## 2. 旋转标定

### 作用

旋转标定用于求两个值：

- `lidar2robot_dis`：LiDAR 到机器人旋转中心的距离，写回主节点 `lidar2robot_dis_`。
- `observable_phase_deg`：旋转轨迹观察到的相位，作为直线标定的输入。

### 运行命令

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration
```

如果需要指定重力补偿：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p gravity_fix_pitch_deg:=-124.8575
```

### 操作步骤

1. 启动后保持机器人完全静止，等待 warmup 完成。
2. 日志出现 `Waiting for rotation to start...` 或 `WAIT_ROTATION` 后再开始转。
3. 让机器人尽量绕自身中心原地旋转。默认一圈以上即可；实际标定建议转 2-3 圈。
4. 旋转要慢、连续，尽量减少平移和急停，不要中途反向。

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

### 输出示例

```text
samples=2193 path_span=578.11 deg yaw_span=581.13 deg
center_x=0.222020 m center_y=-0.428434 m
lidar2robot_dis=0.470864 m
observable_phase_deg=128.056504
quality: radius_rms=0.011361 m phase_rms=3.999998 deg
Recommended code parameter:
  lidar2robot_dis_ = 0.470864;
```

### 质量判断

- `path_span` 默认至少大于 `360 deg`；如果按多圈标定，建议达到 `720 deg` 或 `1080 deg`。
- `yaw_span` 应和 `path_span` 接近，差很多说明 TF yaw 或运动轨迹有问题。
- `radius_rms` 默认阈值是 `0.025 m`，越小越好。
- `phase_rms` 默认阈值是 `4 deg`，越小越好。
- 新版算法会做鲁棒裁剪，并继续采样一段时间保存最佳结果；日志中的 `raw_phase_rms` 是裁剪前原始值，`phase_rms` 是裁剪后用于判定的值。

例如上面的结果可以先用：`radius_rms=0.011361 m` 较好，转动角度也足够；但 `phase_rms=3.999998 deg` 刚好压线。建议继续做直线标定，如果直线四段一致性好就采用；如果直线结果发散，再重跑旋转标定。

### 需要记录

```text
lidar2robot_dis=0.470864
observable_phase_deg=128.056504
```

## 3. 直线标定

### 作用

直线标定用于求：

- `xy_rotation_deg`：SLAM XY 坐标系到机器人 XY 坐标系的旋转角，写回主节点 `xy_rotation_deg_`。
- `lidar2robot_ang`：结合旋转标定相位后得到的 LiDAR 到机器人中心方向角，写回主节点 `lidar2robot_ang_`。

### 运行命令

把旋转标定得到的 `observable_phase_deg` 传进去：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504
```

如果需要指定重力补偿：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p gravity_fix_pitch_deg:=-124.8575
```

### 操作步骤

程序会依次提示四段直线：

1. `+X`：机器人沿自身正 X 方向直走。
2. `-X`：机器人沿自身负 X 方向直走。
3. `+Y`：机器人沿自身正 Y 方向直走。
4. `-Y`：机器人沿自身负 Y 方向直走。

每段建议：

- 走直线，不要转向。
- 运动距离超过默认 `0.35 m`。
- 到位后停稳，等程序进入下一段。

### 日志状态含义

- `CONTINUE`：距离还不够。
- `STOP_TO_COMPLETE`：距离够了，请停稳。
- `NOT_STRAIGHT`：轨迹不够直。
- `TOO_LEFT` / `TOO_RIGHT`：相对期望方向偏得太多。
- `REVERSED`：方向反了。

### 输出示例

```text
========== LINE CALIBRATION RESULTS ==========
xy_rotation_deg=...
quality: line_angle_rms=... deg
lidar2robot_ang=... deg
+X: distance=... heading=... yaw_span=... lateral_rms=... lateral_max=... accepted=true
Recommended code parameter:
  xy_rotation_deg_ = ...;
```

### 质量判断

- 四段尽量都是 `accepted=true`。
- `line_angle_rms` 越小越好，正常建议控制在几度以内。
- 每段 `yaw_span` 越小越好，直线运动时机器人不应明显旋转。
- `lateral_rms` 和 `lateral_max` 越小越好，代表轨迹更直。

如果 `line_angle_rms` 很大、某几段 `accepted=false`，建议重跑直线标定。优先保证 `+X/-X/+Y/-Y` 的物理方向确实是机器人坐标方向。

### 常用调参

直线距离太长，场地不够：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p line_min_distance_m:=0.25
```

直线轨迹噪声较大，适当放宽直线度：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p line_max_lateral_rms_m:=0.06 \
  -p line_max_lateral_abs_m:=0.15
```

停稳条件太严：

```bash
ros2 run tf_to_serial lidar_robot_line_calibration --ros-args \
  -p observable_phase_deg:=128.056504 \
  -p line_stop_motion_m:=0.025
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

旋转标定中有少量跳点，可以适当提高鲁棒裁剪比例：

```bash
ros2 run tf_to_serial lidar_robot_rotation_calibration --ros-args \
  -p rotate_radius_trim_ratio:=0.15 \
  -p rotate_phase_trim_ratio:=0.15
```

## 4. 写回主节点

旋转和直线标定完成后，修改 `src/tf_to_serial/src/tf_to_serial_node.cpp`：

```cpp
const double xy_rotation_deg_ = <直线标定输出的 xy_rotation_deg>;
const double lidar2robot_dis_ = <旋转标定输出的 lidar2robot_dis>;
const double lidar2robot_ang_ = <直线标定输出的 lidar2robot_ang>;
```

例如，你这次旋转标定的距离可以先写为：

```cpp
const double lidar2robot_dis_ = 0.470864;
```

`xy_rotation_deg_` 和 `lidar2robot_ang_` 等直线标定完成后再填。

写回后重新编译：

```bash
colcon build --packages-select tf_to_serial
source install/setup.bash
```

## 5. 启动主节点

确认串口没有被占用后运行：

```bash
ros2 run tf_to_serial tf_to_serial_node
```

当前主节点使用 `/dev/ttyUSB0` 和 `115200`。如果实际串口不同，需要修改 `tf_to_serial_node.cpp` 中的串口配置。

## 故障排除

### 问题 1: IMU 数据不稳定
**症状**：收到 `IMU data unstable (stddev=...)` 警告

**解决方案**：
- 确保设备完全静止
- 检查设备是否放在振动的平面上
- 尝试多次运行，等待更长时间

### 问题 2: 没有接收到 IMU 数据
**症状**：进度条一直停留在 0%

**解决方案**：
- 检查 IMU 话题是否正确：`ros2 topic list`
- 验证 IMU 节点是否正在运行：`ros2 node list`
- 检查话题名称，默认为 `/livox_mid360_imu`（如需修改，编辑代码第 8 行）

### 问题 3: 加速度范数不接近 9.807
**症状**：输出的 `norm` 与 9.807 相差很大（例如 8.5 或 11.2）

**解决方案**：
- IMU 可能需要重新校准（硬件级别）
- 检查坐标系是否正确
- 联系硬件供应商

## 技术细节

### 重力对齐计算
设备静止时，IMU 加速度计测得的值等于重力加速度的反向值。

根据加速度向量计算欧拉角：
```
pitch = atan2(ax, sqrt(ay² + az²))
roll = atan2(ay, az)
```

设置 `q_fix_` 为这些角度的负值，可以在软件中 level 设备的安装倾角。

### 验证方法
标准差 < 0.15 m/s² 表示设备处于相对稳定的静止状态，可以进行可靠的标定。

## 输出示例

```
[INFO] Raw Accelerations (avg):
  ax = 0.234567 m/s²
  ay = -0.123456 m/s²
  az = 9.756789 m/s²
  norm = 9.807134 m/s² (should be ~9.807)

[INFO] Stability Analysis:
  σ(ax) = 0.023456 m/s²
  σ(ay) = 0.034567 m/s²
  σ(az) = 0.045678 m/s²
  total_stddev = 0.061234 (threshold: 0.15)

[INFO] Derived Gravity Alignment Angles:
  roll  = -0.0126° (-0.000220 rad)
  pitch = 01.3712° (0.023939 rad)

[INFO] ========== RECOMMENDED CODE UPDATE ==========
[INFO] Replace the following line in tf_to_serial_node.cpp:
[INFO]   OLD: q_fix_.setRPY(0.0, -(180 - 56.39) * M_PI / 180.0, 0.0);
[INFO]   NEW: q_fix_.setRPY(-0.000220, 0.023939, 0.0);
```

## 注意事项

- ⚠️ 标定前必须关闭主节点 `tf_to_serial_node`，避免干扰
- ✓ 多次运行标定可以提高精度
- ✓ 设备的物理安装位置改变后需要重新标定
- ✓ 建议在室内、稳定环境下进行标定

## 相关文件

- **标定工具源码**：`src/tf_to_serial/src/imu_gravity_calibration.cpp`
- **主节点源码**：`src/tf_to_serial/src/tf_to_serial_node.cpp`
- **构建配置**：`src/tf_to_serial/CMakeLists.txt`
