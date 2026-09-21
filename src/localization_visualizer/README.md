# 独立定位可视化包

`localization_visualizer` 与定位算法、服务和自定义消息没有编译依赖。
它读取可选的全局 PCD、订阅 `/local_map`，从 TF 获取 `map -> odom`，
然后发布两路供 RViz 叠加显示的点云：

| 输出话题 | 坐标系 | 内容 |
| --- | --- | --- |
| `/localization_visualization/global_map` | `map` | 启动时加载并降采样的全局地图 |
| `/localization_visualization/aligned_local_map` | `map` | 按最新 `map -> odom` 对齐的局部地图 |

输入 `/local_map` 的 `header.frame_id` 必须是 `odom`。节点只读取 TF，
不会发布 TF，也不会影响定位结果。两路可视化输出使用 reliable +
transient-local QoS；RViz 配置的固定坐标系为 `map`，全局地图显示为灰色，
对齐后的局部地图显示为红色。地图和局部地图的可视化体素大小默认均为
`0.2 m`，不改变定位包自己的 `0.1 m` 处理参数。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select localization_visualizer
source install/setup.bash
```

## 运行

定位节点与局部地图发布节点已经运行后，在另一终端启动可视化：

```bash
source install/setup.bash
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:=/absolute/path/to/3_part.pcd
```

`start_rviz` 默认为 `true`。部署到另一个工作空间时，可以把该包与
`kiss_matcher_localization`、`pcd_cloud_publisher` 并列放在 `src/`。
可视化包不引用另两个包的源码或安装路径；只需要在启动时指定地图路径。
不指定 `map_pcd` 时，仍可显示对齐后的局部地图，但不会发布全局地图。

在当前工作空间根目录，可用
`map_pcd:="$PWD/src/localization/maps/3_part.pcd"` 指定已准备好的全局地图。

全局地图启动时只读取、体素化一次；随后发布一次，由
transient-local QoS 保留最后一条消息供稍后启动的 RViz 订阅。
局部地图只保留最新一份，并默认每 `1 s` 转换和发布一次。
只有收到 `map -> odom` 后才会发布对齐后的局部地图。
