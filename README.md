# gcopter_ros2

这是 `gcopter` ROS1 demo 的独立 ROS2 迁移包。

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

`/controller/position/output` 默认发布 NED 坐标系 setpoint，由 `enu_to_ned: true` 控制。节点不发布 PX4 offboard heartbeat，不发送 vehicle command。

## 编译

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

启动 mockamap、planner、RViz 和 rqt_plot：

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 global_planning.launch.py
```

## RViz 交互

1. Fixed Frame 保持 `map`。
2. 等待终端日志出现地图初始化成功。
3. 使用 RViz 的 `2D Goal Pose` 工具第一次点击 start。
4. 第二次点击 goal，节点会执行 RRT、FIRI 安全走廊生成和 GCOPTER 优化。
5. 第三次点击会清空上一组 start/goal，作为新的 start。

## 默认 mockamap 参数

`global_planning.launch.py` 使用与原始 GCOPTER demo 一致的地图参数：

- `seed: 1024`
- `resolution: 0.25`
- `x_length: 50`
- `y_length: 50`
- `z_length: 5`
- `type: 1`
- `complexity: 0.025`
- `fill: 0.3`
- `fractal: 1`
- `attenuation: 0.1`

## 迁移说明

详细迁移记录见 [docs/GCOPTER_ROS2_ADAPTATION.md](docs/GCOPTER_ROS2_ADAPTATION.md)。
