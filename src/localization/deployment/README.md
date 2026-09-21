# 从源码构建独立工作空间

工作空间根目录为 `/home/pc/kiss_localization_ws`。`src/` 放三个 ROS 2 包，
`vendor/` 放算法源码，`external/` 是本机从源码编译后生成的安装前缀。
`vendor/` 和 `external/` 不作为 ROS 包交给 colcon 构建。

## 源码版本

| 源码目录 | 当前检验版本 |
| --- | --- |
| `vendor/kiss_matcher` | KISS-Matcher `8b58b118e00c7a5a22448741f8d956d638a44ae2` |
| `vendor/robin` | ROBIN `v1.2.7`（CMake 项目版本 `1.2.7`） |
| `vendor/small_gicp` | `2c6c7831bc1f5ada2ee5300027fe7d7aee66827d` |
| `vendor/pmc` | `4bbd40ababd8e925c4e1845c509173afe766b443` |
| `vendor/xenium` | `1c449ae953ce2a440b0d16c5ed1181d2754860ab` |

`vendor/robin` 在原项目中由 CMake 下载，当前副本没有 Git 元数据；
其版本根据下载配置和源码中的项目版本记录。
复制到另一台电脑时需要保留五个 `vendor/` 目录及源码自带的许可证文件。

## 构建

ROBIN 原版 CMake 在配置时联网下载 PMC 和 xenium。随附补丁让它使用
`vendor/` 中已经固定版本的源码，不改变配准算法。**当前
`/home/pc/kiss_localization_ws/vendor/robin` 已应用补丁**，直接运行：

```bash
cd /home/pc/kiss_localization_ws
bash src/localization/deployment/build_from_source.sh
```

复制到另一台电脑时，如果带过去的是当前整个工作空间的 `vendor/`
目录，补丁仍在，不要再应用一次。如果重新获取了未经修改的 ROBIN
源码，则先执行一次：

```bash
cd /path/to/kiss_localization_ws
patch -p1 -i src/localization/deployment/robin-local-sources.patch
```

构建脚本先清除可能继承的旧工作空间 ROS/CMake 环境，再只加载
`/opt/ros/jazzy`。它依次从源码构建 ROBIN、KISS Matcher、small_gicp，
安装到 `external/`，最后构建三个 ROS 2 包。系统必须事先安装
ROS 2 Jazzy、编译工具、PCL、Eigen3、TBB、FLANN、LZ4、OpenMP 等依赖。

同一台机器上已在 `/tmp/kiss-source-check` 和当前工作空间分别完成构建，
三个 ROS 包均编译通过。当前工作空间的 CMake 缓存指向本地 `external/`
中的算法安装前缀，未引用旧的 `KISS-Matcher/install`。

## 构建后检查

```bash
cd /home/pc/kiss_localization_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix kiss_matcher_localization
ros2 pkg prefix pcd_cloud_publisher
ros2 pkg prefix localization_visualizer
grep -E '^(robin|kiss_matcher|small_gicp)_DIR:' \
  build/kiss_matcher_localization/CMakeCache.txt
ldd install/kiss_matcher_localization/lib/kiss_matcher_localization/navigation_localizer \
  | grep 'not found' || true
```

三个包的前缀应都位于当前工作空间 `install/`；三个算法 CMake 包的路径
应都位于当前工作空间 `external/lib/cmake/`；最后一个命令不应输出
`not found`。检查后再按上一级 `DEPLOYMENT.md` 启动节点。

## 运行终端

```bash
cd /home/pc/kiss_localization_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
```

不要 source 旧工作空间的 `install/setup.bash`。各节点的启动命令和
真实 FastLIO 接入要求见上一级 `DEPLOYMENT.md`。
