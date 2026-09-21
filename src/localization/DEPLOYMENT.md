# 独立工作空间部署

当前独立工作空间位于 `/home/pc/kiss_localization_ws`：

```text
kiss_localization_ws/
├── src/
│   ├── localization/             # kiss_matcher_localization
│   ├── pcd_cloud_publisher/      # PCD 模拟输入，调试时使用
│   └── localization_visualizer/  # 地图叠加与 RViz
├── vendor/                     # 五份算法源码
├── external/                   # 本机从源码生成的算法安装前缀
└── build_algorithms/           # 构建时生成
```

ROS 包之间没有编译依赖。定位包使用 `external/` 中由本地源码编译的
`kiss_matcher`、`robin` 和 `small_gicp` C++ 库。
不要把原工作空间的 `install/setup.bash` 当作目标工作空间的长期依赖。
目标机器还需 ROS 2 Jazzy、PCL、Eigen3、TBB、FLANN、LZ4 等
依赖；具体名称见各包的 `package.xml`。

跨电脑部署请携带并在目标电脑编译 `vendor/` 中的算法源码，而不是
复制另一台电脑编译好的 `.a` 或 `.so`。源码清单、离线构建脚本及
当前验证状态见 [从源码构建说明](deployment/README.md)。

## 准备数据

- 将全局地图复制到 `src/localization/maps/3_part.pcd`，或启动定位
  时用 `map_pcd` 参数给出绝对路径。
- 若使用模拟输入，将局部点云复制到
  `src/pcd_cloud_publisher/clouds/part_1_hard.pcd`，或启动发布器时
  用 `pcd_path` 参数给出绝对路径。
- 两个 PCD 都被各自目录的 `.gitignore` 排除；仅复制源码仓库不会
  自动带上点云文件。

## 从源码构建

当前工作空间的 ROBIN 本地源码补丁已应用，直接运行：

```bash
cd /home/pc/kiss_localization_ws
bash src/localization/deployment/build_from_source.sh
```

脚本会依次构建 ROBIN、KISS Matcher、small_gicp 和三个 ROS 2 包；
不需要先 source 旧工作空间。构建后的检查命令和源码版本见
[从源码构建说明](deployment/README.md)。

## 分别启动

调试 PCD 输入：

```bash
ros2 launch pcd_cloud_publisher local_map_from_pcd.launch.yaml
```

定位：

```bash
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml
```

可视化：

```bash
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:="$(ros2 pkg prefix kiss_matcher_localization)/share/kiss_matcher_localization/maps/3_part.pcd"
```

三个命令在各自终端运行，且每个终端都需先 source
`/opt/ros/jazzy/setup.bash` 与新工作空间的 `install/setup.bash`，
并设置 `LD_LIBRARY_PATH` 指向新工作空间的 `external/lib`；完整命令见
[运行终端](deployment/README.md#运行终端)。接入真实 FastLIO 后关闭模拟 PCD 发布器，
由 FastLIO 提供 `/local_map` 和 `odom -> base_link`。定位节点独占
`map -> odom` 的发布权。

## 验证状态

五份本地源码已经在 `/tmp/kiss-source-check` 完成算法和三个 ROS 包的
隔离构建，CMake 未引用旧工作空间安装目录。
`/home/pc/kiss_localization_ws` 的源码、补丁、脚本、地图和模拟输入
已就位，但该工作空间尚未运行构建脚本；真实 FastLIO 输入和导航
系统联调也尚未验证。
