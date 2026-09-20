# 双 HiPNUC IMU 录制

ROS 2 启动两个独立的 HiPNUC 串口节点，并将两路已解码的 ROS 消息录进同一个 rosbag。
不录制原始串口字节。两台设备需预先配置为 ENU 输出。

```bash
cd /home/zxz/workspace/localization_self_ws
colcon build --packages-up-to dual_imu_recorder
source install/setup.bash
ros2 launch dual_imu_recorder record.launch.py \
  port_1:=/dev/ttyUSB0 port_2:=/dev/ttyUSB1 \
  baudrate_1:=115200 baudrate_2:=115200
```

两个节点分别使用 `/imu1`、`/imu2` 命名空间和 `imu1_link`、`imu2_link` 机体坐标系。
每路录制 `imu/data`、`imu/mag`、`imu/temperature`、`hipnuc/imu`；
另录制全局 `/diagnostics`。诊断状态的名称和 `hardware_id` 可区分节点与串口。
默认输出目录是 `~/imu_bags/dual_imu_<时间戳>`，可用 `bag_dir:=/path/to/dir` 修改父目录。
按 Ctrl+C 结束录制，等待 rosbag 写完元数据后再查看：

```bash
ros2 topic hz /imu1/imu/data
ros2 topic hz /imu2/imu/data
ros2 topic echo /diagnostics
ros2 bag info "$(ls -td ~/imu_bags/dual_imu_* | head -n 1)"
```

可分别指定 `frame_id_1`、`frame_id_2`、`params_file_1`、`params_file_2`。
为保证录全数据，四类数据发布开关始终设为 `true`。`port_1` 和 `port_2` 必须指向不同设备。
多转接器场景优先使用稳定的 `/dev/serial/by-id/` 路径；无设备序列号时可查看
`/dev/serial/by-path/`。当前驱动的消息头采用主机发布前的 ROS 时间，性能分析时
还应检查产品消息的字段有效位、`imu/data` 中协方差首元素 `-1` 的缺失标记，
以及诊断中的接收速率和错误计数。串口不可用时驱动会重试并继续发布诊断。

## 静止零偏对比

录制完成后执行：

```bash
ros2 run dual_imu_recorder analyze_bag.py \
  ~/imu_bags/dual_imu_20260917_154143_664585
```

整段 bag 必须保持静止，建议录制至少 30 分钟。脚本会挑选角速度不超过
`0.02 rad/s`、加速度模长距 `9.80665 m/s²` 不超过 `0.2 m/s²` 的样本，
然后在 bag 的 `bias_analysis/` 中生成 `bias_summary.json` 和 `bias_comparison.png`。

陀螺三轴均值是零偏候选值，标准差反映静止噪声。加速度先由有效的 ENU 姿态旋转到
导航系，再减去 `[0, 0, 9.80665]`；得到的静止残差可比较两台设备，但其中同时包含
加速度计零偏和姿态误差，不能单独当作已标定的加速度计零偏。若静止判据过严，
可调整 `--max-gyro-rad-s` 或 `--gravity-tolerance-m-s2`。

`bias_summary.json` 还会包含 `yaw_static_jitter.variance_rad2`。脚本先对 yaw
解缠绕并拟合、移除线性漂移，再计算剩余抖动的方差和标准差；对应图表的第三列显示
标准差（度）。这衡量静止短期抖动，不等同于长时间航向漂移。
