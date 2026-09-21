# 调试点云

本机的 `part_1_hard.pcd` 是五号楼门口局部点云，用于模拟 FastLIO
持续发布的局部地图。其点坐标在测试中视作 `odom` 坐标系。

点云文件默认不纳入 Git。部署到另一台机器时，将 PCD 复制到本目录
后构建；安装目标为 `share/pcd_cloud_publisher/clouds/`。
也可以在启动时用 `pcd_path:=/absolute/path/to/cloud.pcd` 指定包外文件。
