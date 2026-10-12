# KISS Matcher 点云定位 ROS 2 包

`kiss_matcher_localization` 将 `odom` 坐标系中的局部点云与 `map` 坐标系中的
全局 PCD 地图配准，计算并发布 `map → odom`。配准先使用 KISS Matcher
寻找初始位姿，再使用 VGICP 精配准，并按内点数和重叠率检查结果。

包内有两个运行入口：`navigation_localizer` 用于持续定位和重定位服务；
`inter_frame_alignment` 用于两路点云之间的配准演示。导航定位时启动前者。

## 输入与输出

| 项目 | 默认值或行为 |
| --- | --- |
| 全局地图 | `maps/3_part.pcd`，点坐标位于 `map` 坐标系；启动时加载并体素化 |
| 局部地图 | `/local_map`，`sensor_msgs/msg/PointCloud2`；点坐标和 `header.frame_id` 均为 `odom` |
| 定位变换 | 首次成功应用后发布 `map → odom` TF |
| 定位状态 | `/localization/status`，消息类型为 `kiss_matcher_localization/msg/LocalizationResult` |
| 手动重定位 | `/localization/relocalize` 服务 |
| 候选确认 | `/localization/accept_candidate` 服务 |

默认地图文件放在 [`maps/`](maps/README.md)。PCD 数据不随 Git 仓库分发；
也可以在启动时用 `map_pcd` 指定其他地图。参数默认值见
[`config/navigation_localization.yaml`](config/navigation_localization.yaml)，
服务、状态字段和 TF 更新规则见[导航接口说明](NAVIGATION_INTERFACE.md)。

## 构建

需要 ROS 2 Humble、CMake ≥ 3.22、colcon、PCL、Eigen3、TBB、FLANN、LZ4，
以及 `robin`、`kiss_matcher`、`small_gicp` 算法库。当前工作空间包含算法
源码，在工作空间根目录运行：

```bash
bash src/localization/deployment/build_from_source.sh
```

脚本依次构建算法库和本包、`pcd_cloud_publisher`、
`localization_visualizer` 三个 ROS 包。算法库安装在 `external/`，
ROS 包安装在 `install/`。详细构建依赖和检查方法见
[从源码构建说明](deployment/README.md)。

## 运行导航定位

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml
```

节点收到有效的 `/local_map` 后自动尝试首次定位，成功后按配置周期重新定位。
使用其他地图时添加 `map_pcd:=/absolute/path/to/map.pcd`。局部地图可由实际
里程计系统提供；用 PCD 做离线测试时，可启动
[`pcd_cloud_publisher`](../pcd_cloud_publisher/README.md) 发布测试点云，并用
[`localization_visualizer`](../localization_visualizer/README.md) 查看对齐结果。
完整的三包启动命令见[工作空间 README](../../README.md#3-使用-pcd-调试输入)。

检查定位结果：

```bash
ros2 topic echo /localization/status
ros2 run tf2_ros tf2_echo map odom
```

手动请求一次重定位：

```bash
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize \
  "{apply_if_safe: true}"
```

首次匹配通过质量检查后可以直接应用。已有定位时，若缺少
`odom → base_link`，节点无法比较新旧机器人位姿，会保留匹配结果作为候选，
而不会自动更新 `map → odom`；候选可通过 `accept_candidate` 服务确认。

## 两路点云配准演示

`pcd_inter_frame_demo.launch.yaml` 使用包内的 PCD 发布器读取两份文件，
启动 `inter_frame_alignment`，默认打开 RViz：

```bash
ros2 launch kiss_matcher_localization pcd_inter_frame_demo.launch.yaml \
  source_pcd:=/absolute/path/to/local.pcd \
  target_pcd:=/absolute/path/to/map.pcd
```

已有点云话题可使用 `inter_frame_alignment.launch.yaml`，设置
`source_topic`、`target_topic`、`source_frame` 和 `target_frame`。
无图形界面时添加 `start_rviz:=false`。该演示入口与导航定位入口的 TF
用途不同；导航定位使用 `navigation_localization.launch.yaml`。
