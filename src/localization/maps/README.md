# 全局地图

此目录存放导航定位使用的全局点云地图。当前本机放有
`3_part.pcd`（众创空间五号楼整体地图，约 250 MB，21,841,914 个点）。
地图中的点坐标必须处于导航所用的 `map` 坐标系。

`*.pcd`、`*.ply`、`*.bin` 默认不纳入 Git。部署到另一台机器时，
请将所需地图复制到本目录后构建功能包；CMake 会把 PCD 文件安装到
`share/kiss_matcher_localization/maps/`。也可以在启动时通过
`map_pcd:=/absolute/path/to/map.pcd` 使用包外地图。

导航 launch 默认使用安装后的 `maps/3_part.pcd`。若该文件不存在，
请先放入地图并重新构建，或显式指定 `map_pcd`。
