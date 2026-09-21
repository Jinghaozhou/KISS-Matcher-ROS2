# KISS Matcher 点云定位 ROS 2 包

本目录是独立的 `kiss_matcher_localization` 功能包。它包含定位节点、
KISS Matcher 粗配准与 VGICP 精配准的封装、PCD 演示发布器、配置和 RViz
文件。源码没有引用上级工作空间的 `ros/` 或 `cpp/` 路径；复制本目录
到其他 ROS 2 工作空间的 `src/` 后仍可构建。
三个包一起迁移时的目录布局与构建命令见[独立部署说明](DEPLOYMENT.md)。

全局地图放在 [`maps/`](maps/README.md)。本机的 `3_part.pcd` 已放入该
目录，构建时会安装到功能包的 share 目录；地图文件较大，默认不纳入 Git。
模拟持续局部地图输入的节点放在独立的
[`pcd_cloud_publisher`](../pcd_cloud_publisher/README.md) 包中。
点云叠加和 RViz 放在独立的
[`localization_visualizer`](../localization_visualizer/README.md) 包中。

## 构建依赖

- ROS 2 Jazzy：`ament_cmake`、`rclcpp`、`sensor_msgs`、
  `geometry_msgs`、`tf2`、`tf2_ros`、`pcl_conversions`，以及用于
  导航服务和状态消息的 `rosidl_default_generators`。
- PCL（common、filters、io）以及 Eigen3、TBB、FLANN、LZ4。
- 已安装并可被 CMake 找到的 `kiss_matcher`、`robin` 和 `small_gicp`。
  这三者是外部 C++ 库，需在构建本包前安装；本包不会在构建过程中
  从网络下载源码。若装在非标准前缀，请将其前缀加入 `CMAKE_PREFIX_PATH`。

本包不依赖 SLAM 位姿图、GTSAM 或原来的 `kiss_matcher_ros` 包。
算法封装当前仍使用 ROS 日志类型；它是包内实现，并未承诺独立的
无 ROS C++ API。
演示时若设置 `start_rviz:=true`，还需安装 `rviz2`；无界面运行可设置
`start_rviz:=false`。

## 在现有工作空间构建

当前工作空间已经安装了上述 C++ 库，可直接执行：

```bash
cd /home/pc/KISS-Matcher
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select kiss_matcher_localization
source install/setup.bash
```

复制到其他工作空间后，应先 source ROS 2 与外部 C++ 库的安装前缀，
再执行 `colcon build --packages-select kiss_matcher_localization`。
不需要复制原工作空间的 SLAM 源码。
使用默认导航 launch 前，请同时复制所需的 `maps/3_part.pcd`；
也可通过 launch 的 `map_pcd` 参数指定包外地图。

默认构建类型为 `Release`，以启用定位计算所需的编译优化。若此前已
构建过旧版本，可重新执行上述 `colcon build`，CMake 会更新缓存中的
空构建类型；显式指定的 `Debug` 等构建类型仍会保留。

## 运行

演示入口默认打开 RViz：

```bash
ros2 launch kiss_matcher_localization pcd_inter_frame_demo.launch.yaml \
  source_pcd:=/path/to/local.pcd \
  target_pcd:=/path/to/map.pcd
```

接入已有点云话题时使用 `inter_frame_alignment.launch.yaml`，可设置
`source_topic`、`target_topic`、`source_frame`、`target_frame`、
`config_path` 和 `start_rviz`。两路输入为 `sensor_msgs/msg/PointCloud2`。
该演示入口仍沿用演示版的 TF 与 `reg/*` 可视化话题；导航系统所需的
`map -> odom`、位姿状态和失败有效期接口由独立的
[`navigation_localizer`](NAVIGATION_INTERFACE.md) 入口提供。

原 `kiss_matcher_ros` 包中的同名演示入口暂时保留，已有运行命令仍可用。
后续修改定位算法时应以本目录为准，待导航接口稳定后移除旧实现。
