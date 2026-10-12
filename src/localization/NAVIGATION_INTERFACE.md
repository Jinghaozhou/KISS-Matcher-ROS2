# 导航定位接口

`navigation_localizer` 用 FastLIO 持续发布的局部地图与静态全局 PCD
配准。局部地图的每个点必须在 `odom` 坐标系中，消息的
`header.frame_id` 也必须为 `odom`；全局 PCD 的点坐标必须在 `map`
坐标系中。配准直接得到 `T_map_odom`。FastLIO 继续提供
`odom -> base_link`，导航系统使用 `map -> odom -> base_link`。

尚未接入 FastLIO 时，可先用独立的
[`pcd_cloud_publisher`](../pcd_cloud_publisher/README.md) 包发布
`/local_map`，检查输入话题和定位服务。该调试包不属于定位包的
构建依赖。
可用独立的
[`localization_visualizer`](../localization_visualizer/README.md) 包在
RViz 中叠加全局地图与定位后的局部地图；它也不属于定位包的构建依赖。

新节点与原有 `inter_frame_alignment` PCD 演示节点是两个入口。
导航时只启动 `navigation_localizer`，避免出现两个 TF 发布者。

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml \
  local_map_topic:=/local_map
```

默认地图为安装在功能包中的 `maps/3_part.pcd`（众创空间五号楼整体地图）。
也可通过 `map_pcd:=/absolute/path/to/global_map.pcd` 指定其他地图。
地图文件作为部署数据，需在目标机器上单独复制；详见
[`maps/README.md`](maps/README.md)。

全局地图启动时读取并体素化一次；局部地图订阅使用 sensor-data QoS，
持续接收并只保留最新的 `PointCloud2`。启动时获得有效局部地图后
自动定位；成功后按 `periodic_interval_sec` 周期再次定位。
将该参数设为 `0.0` 可关闭周期定位，保留启动定位和服务调用。
失败后按 `retry_interval_sec` 重试启动定位。

## 服务和状态

```bash
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize \
  "{apply_if_safe: true}"

ros2 topic echo /localization/status
```

`/localization/relocalize` 使用最新局部地图快照立即运行一次配准，
响应为 `LocalizationResult`：

| 字段 | 含义 |
| --- | --- |
| `matched` | KISS 内点和 VGICP 重叠率均满足阈值 |
| `applied` | 候选 `map -> odom` 已应用 |
| `reason` | 应用、预览、超出跳变阈值、输入问题或配准失败原因 |
| `candidate_map_to_odom` | 本次候选变换；仅 `matched=true` 时有效 |
| `has_previous` | 本次尝试前是否已有已应用的定位 |
| `delta_valid` | 是否成功利用 `odom -> base_link` 比较新旧机器人全局位姿 |
| `translation_delta_m` | 同一机器人里程计位姿下，新旧 `map -> base_link` 的平移差（m）；仅 `delta_valid=true` 时有效 |
| `rotation_delta_deg` | 同一机器人里程计位姿下，新旧 `map -> base_link` 的三维旋转差（°）；仅 `delta_valid=true` 时有效 |
| `inliers`、`overlap_percent` | 粗配准内点数（点对）和 VGICP 重叠率（%） |
| `registration_ms` | KISS + VGICP 耗时（ms），不含 ROS/PCL 转换和体素化 |
| `local_map_stamp` | 本次局部地图的采集时间 |
| `last_success_stamp` | 最近一次应用定位结果的时间；此前从未成功时为零 |
| `candidate_id` | 成功但未应用时的候选编号 |

首次定位若 `apply_if_safe=true`，通过质量检查即可应用。
已有定位时，只有 `delta_valid=true`，且机器人位姿差同时不超过
`max_auto_translation_m` 与 `max_auto_rotation_deg` 才自动应用。
`apply_if_safe=false` 始终只返回候选结果。

成功但未应用的结果可在 `candidate_timeout_sec` 内确认：

```bash
ros2 service call /localization/accept_candidate \
  kiss_matcher_localization/srv/AcceptCandidate \
  "{candidate_id: '返回的编号'}"
```

确认服务应用已有候选，不再重新计算配准。新定位尝试会取代旧候选。
服务、启动和周期定位使用同一个互斥执行组；局部地图接收与 TF
广播有各自的执行组。每次尝试及候选确认都会在
`/localization/status` 发布同一结构的结果；状态话题使用
reliable + transient-local QoS。

## TF 和失效处理

节点在第一次应用定位结果后以 `10 Hz` 发布 `map -> odom`。
失败或大幅变化暂缓应用时保留上一次变换，状态里的 `applied=false`
和 `last_success_stamp` 告诉导航系统结果的新鲜程度。发布 TF 时
使用当前 ROS 时间；状态保留局部地图的原始时间戳。导航系统应
依据状态自行决定多久以前的全局定位仍可用。

默认参数见 [`config/navigation_localization.yaml`](config/navigation_localization.yaml)。
`1.0 m` 与 `15.0°` 是初始自动应用阈值，需结合实测漂移与导航
允许的位姿跳变调整。局部地图过期、`frame_id` 不符、无点或配准
质量不足都会拒绝本次结果。
