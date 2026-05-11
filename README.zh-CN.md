# gcopter_ros2

[English](README.md) | 中文

`gcopter_ros2` 是原始 `gcopter` ROS1 demo 的独立 ROS2 迁移包。
包内包含 GCOPTER 核心算法头文件，编译时不依赖 `../gcopter` 目录。

原始 GCOPTER 仓库：https://github.com/ZJU-FAST-Lab/GCOPTER

## 依赖

完整 RViz 仿真使用 `mockamap` 在 `/mock_map` 上发布默认点云地图。

Mockamap 仓库：

```bash
git clone https://github.com/Atticlmr/mockamap.git
```

如果要运行本包提供的仿真 launch 文件，请把 `mockamap` 放在同一个 ROS2
工作空间的 `src` 目录下。

其它依赖包括：

- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `visualization_msgs`
- `px4_msgs`
- `rviz2`
- `rqt_plot`
- Eigen3
- OMPL

## 节点

### `gcopter_ros2_planner`

输入：

- `/mock_map` (`sensor_msgs/msg/PointCloud2`)：地图点云，默认 frame 为 `map`。
- `/move_base_simple/goal` (`geometry_msgs/msg/PoseStamped`)：RViz 2D Goal Pose。第一次点击作为 start，第二次点击作为 goal，之后重新点击会清空并重新开始。

输出：

- `/visualizer/route` (`visualization_msgs/msg/Marker`)
- `/visualizer/waypoints` (`visualization_msgs/msg/Marker`)
- `/visualizer/trajectory` (`visualization_msgs/msg/Marker`)
- `/visualizer/mesh` (`visualization_msgs/msg/Marker`)
- `/visualizer/edge` (`visualization_msgs/msg/Marker`)
- `/visualizer/spheres` (`visualization_msgs/msg/Marker`)
- `/visualizer/speed` (`std_msgs/msg/Float64`)
- `/visualizer/total_thrust` (`std_msgs/msg/Float64`)
- `/visualizer/tilt_angle` (`std_msgs/msg/Float64`)
- `/visualizer/body_rate` (`std_msgs/msg/Float64`)
- `/controller/position/output` (`px4_msgs/msg/TrajectorySetpoint`)

`/controller/position/output` 默认发布 NED 坐标系 setpoint，由
`enu_to_ned: true` 控制。节点不发布 PX4 offboard heartbeat，也不发送 vehicle command。

## 编译

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select mockamap gcopter_ros2
```

如果只需要编译 planner 包，并且依赖已经可用：

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select gcopter_ros2
```

## 启动

只启动 planner：

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 gcopter_ros2_planner.launch.py
```

启动包含 `mockamap`、planner、RViz 和 `rqt_plot` 的完整 RViz 仿真：

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 global_planning.launch.py
```

`gcopter_sim.launch.py` 也会启动 `mockamap`、planner、RViz 和 `rqt_plot`：

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 gcopter_sim.launch.py
```

如果没有启动 `mockamap`，planner 就收不到默认的 `/mock_map` 点云。此时需要
启动 `mockamap`，或者把 `config/gcopter_ros2.yaml` 里的 `map_topic` 改成其它
`sensor_msgs/msg/PointCloud2` 地图源。

## RViz 交互

1. Fixed Frame 保持 `map`。
2. 等待终端日志出现地图初始化成功。
3. 使用 RViz 的 `2D Goal Pose` 工具第一次点击 start。
4. 第二次点击 goal，节点会执行 RRT、FIRI 安全走廊生成和 GCOPTER 优化。
5. 第三次点击会清空上一组 start/goal，作为新的 start。

## 默认 mockamap 参数

`global_planning.launch.py` 和 `gcopter_sim.launch.py` 使用以下地图参数：

- `seed: 1024`
- `update_freq: 1.0`
- `resolution: 0.25`
- `x_length: 50`
- `y_length: 50`
- `z_length: 5`
- `type: 1`
- `complexity: 0.025`
- `fill: 0.3`
- `fractal: 1`
- `attenuation: 0.1`
