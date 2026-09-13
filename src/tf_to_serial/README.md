# tf_to_serial

这个包负责两件事：

- 读取 FAST-LIO 已换算完成的 `odom -> base_link_hf` 高频机器人位姿，按既有协议发送；
- 接收重启魔术帧后被动执行已配置的重启命令。

## 配置

默认配置在 `config/tf_to_serial.yaml`，安装后路径为：

```bash
$(ros2 pkg prefix tf_to_serial)/share/tf_to_serial/config/tf_to_serial.yaml
```

YAML 中包含串口设备、波特率、超时/重连时间、TF frame、发送频率与告警阈值，以及固定协议的帧头、帧尾、坐标倍率和重启帧。所有协议字节以十六进制表示。

节点直接序列化 FAST-LIO 发布的 `odom -> base_link_hf`。

## 构建与运行

```bash
colcon build --packages-select tf_to_serial
source install/setup.bash

ros2 run tf_to_serial tf_to_serial_node --ros-args \
  --params-file "$(ros2 pkg prefix tf_to_serial)/share/tf_to_serial/config/tf_to_serial.yaml"
```

仓库根目录的 `postion_odom.sh` 负责启动节点并加载该 YAML；串口、TF、频率等参数统一在 YAML 中维护。

## 发送协议

每帧 25 字节，五个 `float` 为主机 IEEE-754 小端序：

```text
FF FE 01 | x_mm(float) | y_mm(float) | z_mm(float) |
yaw_deg(float) | pitch_deg(float) | AA DD
```

## 被动重启

默认重启帧：

```text
FF FE 01 78 13 AA DD
```

收到后执行 `restart_command`。默认命令来自环境变量 `POSTION_ODOM_RESTART_CMD`；`postion_odom.sh` 会自动设置它。若单独启动节点，可显式传入：

```bash
ros2 run tf_to_serial tf_to_serial_node --ros-args \
  --params-file "$(ros2 pkg prefix tf_to_serial)/share/tf_to_serial/config/tf_to_serial.yaml" \
  -p restart_command:='bash /path/to/postion_odom.sh'
```
