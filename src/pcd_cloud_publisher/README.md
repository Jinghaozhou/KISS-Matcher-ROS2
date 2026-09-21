# 独立 PCD 点云话题发布包

`pcd_cloud_publisher` 只读取 PCD 并发布 `sensor_msgs/msg/PointCloud2`，
不链接 KISS Matcher、small_gicp 或定位功能包。它用于调试定位节点的
局部地图输入，也可以与 `kiss_matcher_localization` 并列部署到新的
ROS 2 工作空间。

默认 launch 读取安装后的 [`clouds/part_1_hard.pcd`](clouds/README.md)，
以 `odom` 为 `frame_id`、1 Hz 发布到 `/local_map`。每条消息使用当前
ROS 时间戳，适配定位节点的输入新鲜度检查。PCD 只有 XYZ 时，输出
的 intensity 填 `0`；节点不发布任何 TF。

## 构建和运行

```bash
cd /home/pc/kiss_localization_ws
bash src/localization/deployment/build_from_source.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pcd_cloud_publisher local_map_from_pcd.launch.yaml
```

可通过 `pcd_path`、`topic`、`frame_id`、`publish_rate` 参数更换输入
文件和发布方式。`publish_rate:=0.0` 只发布一次；调试持续输入时
保持正值。持续发布使用 sensor-data QoS（best-effort + volatile），
与定位节点的订阅一致；单次发布保留 reliable + transient-local QoS，
使稍后启动的订阅者仍能收到点云。PCD 只在启动时转换为 ROS 消息，
每次发布更新其时间戳。

另一个终端可启动 `kiss_matcher_localization` 的
`navigation_localization.launch.yaml`。定位包自行加载全局地图；
此发布包只提供 `/local_map`，不需要知道全局地图路径。

部署时将 `pcd_cloud_publisher/` 和 `localization/` 两个目录分别放入
目标工作空间的 `src/` 下，先安装定位包所需的外部 C++ 库，再一起
运行 `colcon build`。调试结束后，可以关闭此发布包并将 `/local_map`
切换到 FastLIO 的真实话题。
