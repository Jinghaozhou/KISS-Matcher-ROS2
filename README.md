# KISS Matcher 独立定位工作空间

本目录用于在不依赖 `/home/pc/KISS-Matcher/install` 的条件下，
从本地算法源码构建定位模块并验证部署。当前工作空间已完成源码构建
和 PCD 模拟输入下的定位服务验证；真实 FastLIO 与导航系统尚未联调。

定位实现参考 [MIT-SPARK/KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)。
项目自有代码采用 [MIT 许可](LICENSE)；随附算法及第三方源码的来源、
版权和许可见[第三方声明](THIRD_PARTY_NOTICES.md)。

## 目录

- `src/localization/`：定位节点、服务、全局地图和部署文档。
- `src/pcd_cloud_publisher/`：调试用 `/local_map` 发布节点。
- `src/localization_visualizer/`：独立可视化节点和 RViz 配置。
- `vendor/`：KISS Matcher、ROBIN、small_gicp、PMC、xenium 源码。
- `external/`：构建脚本将在此安装算法头文件和库。

## 1. 从本地源码构建

确认本机已安装 ROS 2 Jazzy、colcon、CMake、C++ 编译器、PCL、
Eigen3、TBB、FLANN、LZ4 和 OpenMP。ROBIN 的本地源码补丁已经应用，
无需再次运行 `patch`。

```bash
cd /home/pc/kiss_localization_ws
bash src/localization/deployment/build_from_source.sh
```

脚本自行加载 `/opt/ros/jazzy`，依次构建三个算法库和三个 ROS 包；
不要在构建前 source 旧 KISS-Matcher 工作空间。详细源码版本、
补丁用途和构建后检查命令见
[从源码构建说明](src/localization/deployment/README.md)。

## 2. 检查独立性

构建成功后：

```bash
cd /home/pc/kiss_localization_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix kiss_matcher_localization
ros2 pkg prefix pcd_cloud_publisher
ros2 pkg prefix localization_visualizer
grep -E '^(robin|kiss_matcher|small_gicp)_DIR:' \
  build/kiss_matcher_localization/CMakeCache.txt
```

前三项应都指向本工作空间的 `install/`，最后三项应指向本工作空间的
`external/lib/cmake/`。检查动态库的命令也见源码构建说明。

## 3. 使用 PCD 调试输入

在三个终端分别复制对应的完整命令：

```bash
cd /home/pc/kiss_localization_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch pcd_cloud_publisher local_map_from_pcd.launch.yaml
```

```bash
cd /home/pc/kiss_localization_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml
```

```bash
cd /home/pc/kiss_localization_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:="$(ros2 pkg prefix kiss_matcher_localization)/share/kiss_matcher_localization/maps/3_part.pcd"
```

当前工作空间已包含全局地图 `src/localization/maps/3_part.pcd` 和调试点云
`src/pcd_cloud_publisher/clouds/part_1_hard.pcd`。接入真实 FastLIO 时，
不运行调试发布器；由 FastLIO 持续发布 `/local_map`（`odom` 坐标系）
并提供 `odom → base_link`。完整导航接口见
[导航接口文档](src/localization/NAVIGATION_INTERFACE.md)。

上述两份 PCD 是本机验证数据，已被 Git 忽略；从 GitHub 克隆后需要自行
提供数据，或通过 launch 参数指定其他 PCD 路径。
