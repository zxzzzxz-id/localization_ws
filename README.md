# localization_ws

用于独立调试 Livox MID-360 的 FAST-LIO 建图、定位和重定位。

- 工作空间仓库：https://github.com/zxzzzxz-id/localization_ws
- 算法子模块：`src/fast_lio`，来源 https://github.com/zxzzzxz-id/fast_lio
- 雷达驱动：`src/livox_ros_driver2`

## 获取与构建

```bash
git clone --recurse-submodules https://github.com/zxzzzxz-id/localization_ws.git
cd localization_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

已有检出在更新外层仓库后同步子模块：

```bash
git pull --ff-only
git submodule update --init --recursive
```

ROS 2 Humble 下需安装 PCL、Eigen、OpenMP 等依赖，详见子模块 `package.xml` 和 `CMakeLists.txt`。
先验 PCD、rosbag、日志和构建产物不纳入 Git；克隆后需要自行准备地图。

## 运行

在不同终端分别 source `install/setup.bash`，然后启动驱动与算法：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 建图：与定位模式二选一，退出时保存 src/fast_lio/PCD/scans.pcd
ros2 launch fast_lio mapping_mid360.launch.py

# 定位与重定位：指定已有地图，不会在退出时覆盖地图
ros2 launch fast_lio reloc_mid360.launch.py \
  map_pcd:=$HOME/workspace/localization_ws/src/fast_lio/PCD/scans.pcd
```

重定位启动会自动检测 `pcd_to_occupancy_map`：已安装并 source 时默认启动栅格节点；
缺少该包则提示并跳过。可用 `start_map:=false` 手动关闭栅格节点。
雷达 IP 与网络配置位于 `src/livox_ros_driver2/config/MID360_config.json`。

## 修改与发布子模块

子模块源码在其自己的仓库提交和推送；默认检出可能处于 detached HEAD，开发前先切换分支。

```bash
cd src/fast_lio
git switch main
# 修改并验证算法
git add <修改的文件>
git commit -m "Describe localization change"
git push origin main

cd ../..
git add src/fast_lio
git commit -m "Update FAST-LIO revision"
git push origin main
```

先推送算法提交，再推送外层子模块指针。导航集成仓库可引用同一算法仓库并固定自己的已验证版本。
