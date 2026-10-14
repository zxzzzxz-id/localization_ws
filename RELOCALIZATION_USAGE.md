# FAST-LIO 建图与重定位

## 1. 建图并保存先验地图

建图前，在 `src/fast_lio/config/mapping/mid360.yaml` 中启用保存：

```yaml
pcd_save:
    pcd_save_en: true
    interval: -1
```

重新编译并启动建图：

```bash
source install/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py
```

建图结束时按 `Ctrl+C`。终端出现 `saved accumulated global map` 表示保存完成。地图文件为：

```text
<工作区>/src/fast_lio/PCD/scans.pcd
```

保存日志会打印实际绝对路径和点数。当前配置中 `pcd_save_en` 是 `false`；需要建图保存时再设为 `true` 并重新编译。

## 2. 启动重定位和 FAST-LIO

`reloc_mid360.launch.py` 会同时启动 FAST-LIO、先验地图栅格节点、ICP 重定位节点和 RViz。这个 launch 默认自动尝试重定位，并关闭 FAST-LIO 的 PCD 保存。

直接使用默认配置启动，不需要额外参数：

```bash
source install/setup.bash
ros2 launch fast_lio reloc_mid360.launch.py
```

默认先验地图是安装目录下的：

```text
<工作区>/install/fast_lio/share/fast_lio/PCD/scans.pcd
```



## 3. 正常启动与重定位日志

启动时通常能看到以下信息：

```text
[fast_lio]: reloc input enabled: /reloc/cloud_align [odom]
[icp_relocalizer]: 读取先验地图 <地图路径> ...
[icp_relocalizer]: 先验地图 raw=... z_filtered=... coarse=... fine=...
[icp_relocalizer]: 等待 /cloud_registered_body，request=/reloc/request auto_start=true accumulate=30 ... 输出 /reloc/cloud_align
```

机器人静止、收到足够点云后，会显示累计帧数和配准结果。成功时重点看：

```text
ACCEPT: ...
重定位成功：... 已发到 /reloc/cloud_align
```

只有看到 `ACCEPT` 和 `重定位成功` 才表示重定位完成。若看到 `REJECT`，表示本轮匹配未通过验收。

