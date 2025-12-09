airsim 简易全局规划示例

简介：
- 本示例基于预计算地图（point cloud + 元数据），演示在 AirSim 环境下的全局路径规划流程。此项目为教学/参考用途，面向同学分享，非正式的生产/发布仓库。

依赖：
- Python 及常用科学计算/优化库
- 必需（AirSim 远程控制）：
  - `msgpack-rpc-python`（安装：`pip3 install msgpack-rpc-python`）
  - `airsim`（安装：`pip3 install airsim`）
- 可能需要（取决于规划模块与约束求解器）：
  - `cvxpy`
  - `shapely`

运行说明：
- 离线/地图测试（无需启动 AirSim）：
  - `python simple_planner/test_on_maps.py /path/to/map_file.pkl`
- 在 AirSim 中运行仿真：
  - 启动 AirSim 环境后运行：
    `python simple_planner/test_on_airsim.py /path/to/map_file.pkl`

地图文件：
- 格式：`.pkl`，应包含点云与必要的地图元数据（例如分辨率、范围等）。脚本会读取这些信息用于路径搜索与碰撞检测。
- 推荐地图：`UrbanBuildings_res5_0_voxel.pkl`（5 米分辨率，作为默认推荐），另有 1 米分辨率的大文件可选。请根据需求选择合适分辨率以平衡细节与计算量。

配置注意事项：
- `test_on_airsim.py` 中假定的飞机/无人机名称为 `Drone1`（参见脚本内第 77 行），请确保 AirSim 的 `settings.json` 中实体名与之匹配。

资源与下载：
- 原始说明中包含地图与 AirSim 环境的下载链接（如需我可以把链接或文件位置再整理并发送给你）。

说明与建议：
- 本仓库为教学示例，若要在其它机器上复现，请先确认 Python 环境与所需包已安装，并确保提供的地图路径与 AirSim 设置正确。
- 如需我帮忙：整理依赖 `requirements.txt`、添加运行示例脚本或补充参数说明，可继续告诉我你的偏好。

作者：原始作者（见项目内文件）。
