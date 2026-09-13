# localization

> Livox MID-360 激光雷达 + FAST-LIO 的 ROS 2 定位模块，为底盘提供实时、平滑的机器人中心位姿。

## 模块简介

- **硬件设备**：Livox MID-360 激光雷达；下位机通过串口（默认 `/dev/my_lidar`）接收位姿。
- **控制任务**：雷达驱动 → FAST-LIO 里程计（含自车点云过滤）→ TF 与高频里程计 → 串口回传；同时支持下位机发重启魔术帧被动重启整套程序。

**工程组成**

| 组成 | 说明 |
| --- | --- |
| `src/livox_ros_driver2` | Livox 官方 ROS 2 驱动，发布 `/livox/lidar`、`/livox/imu` |
| `src/fast_lio` | FAST-LIO 前端，输出 `odom -> base_link` / `base_link_hf`、`/Odometry`、`/OdometryHighFreq` |
| `src/tf_to_serial` | 把 `odom -> base_link_hf` 按固定协议经串口发给下位机，并处理被动重启 |
| `postion_odom.sh` | 一键启动脚本，导出 `POSTION_ODOM_RESTART_CMD`，启动参数为 `--team red\|blue` |
| `restart_postion_odom_helper.sh` | 被动重启前清理旧进程 |

**数据流**

```text
Livox MID-360
   │  /livox/lidar, /livox/imu
   ▼
livox_ros_driver2 ──▶ fast_lio（自车点过滤）──▶ TF: odom -> base_link / base_link_hf
                          │         /Odometry, /OdometryHighFreq
                          ▼
                     tf_to_serial ── /dev/my_lidar ──▶ 下位机
```

## 负责人

| 阶段 | 成员 |
| --- | --- |
| 一期 | 周天昊 [@RightTr](https://github.com/RightTr)、王圣乾 [@biliybiliy826](https://github.com/biliybiliy826) |
| 二期 | 栗振杰 [@L-Anjing](https://github.com/L-Anjing) |
| 三期 | 待续 |

## 环境

- Ubuntu 22.04 + ROS 2 Humble
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)：`livox_ros_driver2` 的编译依赖，需要单独编译安装并 source
- PCL、Eigen3、OpenMP、Boost、APR（`libapr1-dev`）
- ROS 2 依赖：`rclcpp`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`tf2`、`tf2_ros`、`pcl_conversions`、`urdf`、`serial`

```bash
source /opt/ros/humble/setup.bash
# 如果 Livox-SDK2 装在独立工作空间里：
# source <Livox-SDK2 工作空间>/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

## 文件说明

```text
localization/
├── README.md                        # 本文件
├── postion_odom.sh                  # 一键启动脚本（按脚本内启用的条目依次拉起节点）
├── restart_postion_odom_helper.sh   # 被动重启前清理旧进程
└── src/
    ├── fast_lio/                    # FAST-LIO 前端（含自车点云过滤、重定位）
    ├── livox_ros_driver2/           # Livox 官方 ROS 2 驱动
    └── tf_to_serial/                # 位姿串口回传 + 被动重启
```

一键启动：

```bash
./postion_odom.sh --team red     # 或 --team blue
```

各模块的参数配置、话题与调试方法见对应模块的 README：

- [`src/fast_lio/README.md`](src/fast_lio/README.md)
- [`src/tf_to_serial/README.md`](src/tf_to_serial/README.md)
- [`src/livox_ros_driver2/README.md`](src/livox_ros_driver2/README.md)

> `src/livox_ros_driver2` 采用 Livox 官方 ROS 2 驱动，`src/fast_lio` 为 MID-360 适配的 ROS 2 FAST-LIO 前端。
