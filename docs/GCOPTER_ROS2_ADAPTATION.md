# GCOPTER ROS2 迁移记录

本文记录从原始 `gcopter` ROS1 demo 到 `gcopter_ros2` ROS2 包的迁移过程。目标包位于 `gcopter_ros2`，不会修改原始 `gcopter` 目录。

## 1. 包结构与构建系统

修改模块：

- `package.xml`
- `CMakeLists.txt`
- `include/gcopter/*`

模块作用：

- `package.xml` 声明 ROS2 包名 `gcopter_ros2`、ament 构建工具和运行依赖。
- `CMakeLists.txt` 构建 `gcopter_ros2_planner` 可执行文件，链接 Eigen3、OMPL 和 ROS2 消息依赖。
- `include/gcopter/*` 是从原始 `gcopter/include/gcopter/*` 复制的核心算法头文件。

为什么这样改：

- ROS1 使用 `catkin`、`roscpp` 和 `catkin_package()`；ROS2 使用 `ament_cmake`、`rclcpp` 和 `ament_package()`。
- 目标包必须独立编译，因此复制算法头文件到目标包内，不通过 `../gcopter` include 路径参与编译。

和原 ROS1 逻辑的对应关系：

- 原始包 `gcopter` 中的 `global_planning` 可执行文件对应 ROS2 中的 `gcopter_ros2_planner`。
- 原始算法头文件保持在 `gcopter` 命名空间和 `voxel_map`、`sfc_gen`、`flatness` 命名空间下，调用接口不改。

ROS2/C++17 兼容修补：

- `include/gcopter/gcopter.hpp` 显式包含 `gcopter/geo_utils.hpp`，因为该头文件内部调用 `geo_utils::enumerateVs()`。
- `include/gcopter/geo_utils.hpp` 中 `filterLess::operator()` 增加 `const`，满足 C++17 `std::set` 对比较器 const 可调用的要求。
- 这两处只修补目标包内复制的算法头文件，原始 `gcopter` 目录未修改，算法公式和调用关系未改变。

## 2. 参数系统

修改模块：

- `config/gcopter_ros2.yaml`
- `src/gcopter_ros2_planner.cpp`

模块作用：

- 使用 ROS2 parameter server 管理 planner、地图、优化器和 PX4 setpoint 发布参数。

为什么这样改：

- ROS1 `ros::NodeHandle("~").getParam()` 已替换为 ROS2 `declare_parameter()` 和 `get_parameter()`。
- 参数名改为 snake_case，便于 ROS2 配置风格统一。

和原 ROS1 逻辑的对应关系：

| ROS2 参数 | ROS1 参数 | 含义 |
| --- | --- | --- |
| `map_topic` | `MapTopic` | 地图点云输入话题 |
| `target_topic` | `TargetTopic` | RViz 目标点输入话题 |
| `dilate_radius` | `DilateRadius` | 障碍物膨胀半径 |
| `voxel_width` | `VoxelWidth` | 体素分辨率 |
| `map_bound` | `MapBound` | 地图边界 `[xmin, xmax, ymin, ymax, zmin, zmax]` |
| `timeout_rrt` | `TimeoutRRT` | RRT 求解时间 |
| `max_vel_mag` | `MaxVelMag` | 最大速度约束 |
| `max_bdr_mag` | `MaxBdrMag` | 最大 body rate 约束 |
| `max_tilt_angle` | `MaxTiltAngle` | 最大倾角约束 |
| `min_thrust` | `MinThrust` | 最小总推力 |
| `max_thrust` | `MaxThrust` | 最大总推力 |
| `vehicle_mass` | `VehicleMass` | 飞行器质量 |
| `grav_acc` | `GravAcc` | 重力加速度 |
| `horiz_drag` | `HorizDrag` | 水平阻力系数 |
| `vert_drag` | `VertDrag` | 垂直阻力系数 |
| `paras_drag` | `ParasDrag` | 寄生阻力系数 |
| `speed_eps` | `SpeedEps` | 速度平滑项 |
| `weight_t` | `WeightT` | 时间权重 |
| `chi_vec` | `ChiVec` | 约束惩罚权重 |
| `smoothing_eps` | `SmoothingEps` | 平滑 epsilon |
| `integral_intervs` | `IntegralIntervs` | 积分区间数量 |
| `rel_cost_tol` | `RelCostTol` | 优化相对代价收敛阈值 |

新增 ROS2 参数：

- `marker_frame`：RViz Marker frame，默认 `map`。
- `controller_topic`：PX4 上层控制 setpoint 输出，默认 `/controller/position/output`。
- `publish_rate_hz`：轨迹采样发布频率，默认 50 Hz。
- `enu_to_ned`：是否将 GCOPTER ENU 轨迹转为 PX4 NED setpoint，默认 true。
- `setpoint_yaw_enu`：轨迹采样 setpoint 的 ENU yaw。

## 3. 地图点云输入与 VoxelMap

修改模块：

- `src/gcopter_ros2_planner.cpp`
- `include/gcopter/voxel_map.hpp`

模块作用：

- 订阅 `sensor_msgs/msg/PointCloud2` 地图点云。
- 将有效点写入 `voxel_map::VoxelMap`，并按 `dilate_radius / voxel_width` 做膨胀。

为什么这样改：

- ROS1 demo 将 `msg->data` 直接按 float 数组读取，隐含假设 xyz 位于每个 point 的前 12 字节。
- ROS2 版本按 `msg->fields` 查找 `x/y/z` offset，并检查 datatype 必须是 `FLOAT32`，避免不同 PointCloud2 layout 下解析错误。
- 地图仍然只初始化一次，保持原始 demo 的一次性地图逻辑。

和原 ROS1 逻辑的对应关系：

- 原 ROS1 `mapCallBack()` 中 `voxelMap.setOccupied()` 和 `voxelMap.dilate()` 保持对应。
- ROS2 版本增加日志：接收点数量、解析有效点数量、地图边界、膨胀体素数。

## 4. start/goal 回调

修改模块：

- `src/gcopter_ros2_planner.cpp`
- `config/gcopter_rviz.rviz`

模块作用：

- 订阅 `/move_base_simple/goal`。
- 第一次点击作为 start，第二次点击作为 goal，之后点击会清空并重新开始。

为什么这样改：

- 保持原始 demo 的交互习惯，方便 RViz 直接使用 `2D Goal Pose`。
- 未初始化地图时拒绝目标点，避免在空地图上规划。

和原 ROS1 逻辑的对应关系：

- `z_goal` 计算保持原式：

```cpp
map_min_z + dilate_radius + abs(orientation.z) * (map_z_range - 2 * dilate_radius)
```

- `voxel_map.query(goal) == 0` 时才接受目标点；不可行点输出 warning。

## 5. 规划主流程

修改模块：

- `src/gcopter_ros2_planner.cpp`
- `include/gcopter/sfc_gen.hpp`
- `include/gcopter/gcopter.hpp`
- `include/gcopter/flatness.hpp`

模块作用：

- 前端 RRT 搜索 route。
- 从体素地图提取膨胀表面点。
- FIRI 生成凸安全飞行走廊。
- GCOPTER 在多面体安全走廊内优化五次多项式轨迹。

为什么这样改：

- 迁移目标要求保留 GCOPTER/FIRI 核心算法逻辑，不做薄封装。
- ROS2 节点只替换通讯、参数、可视化和控制输出，不改变算法调用链。

和原 ROS1 逻辑的对应关系：

ROS2 `plan()` 保持原始顺序：

1. `sfc_gen::planPath<voxel_map::VoxelMap>()`
2. `voxel_map.getSurf(pc)`
3. `sfc_gen::convexCover(route, pc, origin, corner, 7.0, 3.0, h_polys)`
4. `sfc_gen::shortCut(h_polys)`
5. `gcopter::GCOPTER_PolytopeSFC::setup()`
6. `gcopter::GCOPTER_PolytopeSFC::optimize()`
7. 保存 `Trajectory<5>` 并开始按 timer 采样发布。

## 6. ROS2 可视化

修改模块：

- `src/gcopter_ros2_planner.cpp`
- `config/gcopter_rviz.rviz`

模块作用：

- 发布 route、waypoints、trajectory、mesh、edge、spheres 六类 RViz Marker。
- 发布 speed、total_thrust、tilt_angle、body_rate 四个指标话题。

为什么这样改：

- 原始 `include/misc/visualizer.hpp` 依赖 ROS1 `ros::Publisher` 和 ROS1 消息类型。
- ROS2 版本重写 visualizer，使用 `rclcpp::Publisher`、`visualization_msgs/msg/Marker` 和 `std_msgs/msg/Float64`。
- Marker frame 默认 `map`，与 mockamap 点云和 RViz Fixed Frame 一致。

和原 ROS1 逻辑的对应关系：

- route、waypoints、trajectory 的 Marker 类型、颜色和采样方式对应原 `Visualizer::visualize()`。
- polytope mesh 仍然使用 `geo_utils::enumerateVs()` 和 `quickhull::QuickHull` 枚举三角面。
- 指标计算仍然使用 `flatness::FlatnessMap::forward()`。

可视化修复：

- RViz2 的 Marker display 使用 `Topic` 字段配置话题，不再使用 ROS1 RViz 配置里的 `Marker Topic` 字段。
- RRT route 在 `planPath()` 成功后立即发布，不再等待 GCOPTER 优化完成，便于区分前端路径和后端轨迹问题。
- route 改为 `LINE_STRIP`，更适合显示连续路径；trajectory 仍保持采样线段。
- 飞行走廊发布时增加日志，输出 `h_polys`、跳过的 polytope 数量、mesh triangle 数量和 edge segment 数量。

## 7. PX4 位置控制输出

修改模块：

- `src/gcopter_ros2_planner.cpp`
- `config/gcopter_ros2.yaml`

模块作用：

- 按 `publish_rate_hz` 采样 GCOPTER 轨迹。
- 发布 `/controller/position/output`，类型为 `px4_msgs/msg/TrajectorySetpoint`。

为什么这样改：

- planner 只负责规划轨迹和发布统一位置控制输出，不接管 PX4 offboard 模式。
- `px4_ros2_ctrl` 的 FSM 和 `Px4OutputAdapter` 负责 heartbeat、vehicle command 和 `/fmu/in/*` 输出。

和原 ROS1 逻辑的对应关系：

- 原 ROS1 `process()` 每 1 kHz 根据当前轨迹时间发布可视化指标。
- ROS2 版本把该逻辑放入 timer：发布指标、当前位置 sphere 和 PX4 setpoint。
- 轨迹超出总时长后停止发布 setpoint。

坐标转换：

- GCOPTER 内部按 ENU 使用：`x_e, y_e, z_u`。
- 默认 `enu_to_ned: true`，发布到 PX4 setpoint 时转换为：`x_n = y_e, y_east = x_e, z_down = -z_u`。
- yaw 转换为 `yaw_ned = pi/2 - yaw_enu`。

## 8. launch 与仿真

修改模块：

- `launch/gcopter_ros2_planner.launch.py`
- `launch/global_planning.launch.py`

模块作用：

- `gcopter_ros2_planner.launch.py` 只启动 planner。
- `global_planning.launch.py` 启动 mockamap、planner、RViz 和 rqt_plot。

为什么这样改：

- 将 planner 单独启动与完整仿真启动分开，便于接真实地图或外部控制系统。
- mockamap 参数按原始 GCOPTER demo 固定，保证地图默认行为一致。

和原 ROS1 逻辑的对应关系：

- 原 `global_planning.launch` 同时启动 RViz、rqt_plot、mockamap 和 `global_planning`。
- ROS2 版 `global_planning.launch.py` 提供等价组合，并把地图话题保持为 `/mock_map`。

## 9. 验证命令

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select gcopter_ros2
source install/setup.bash
ros2 launch gcopter_ros2 global_planning.launch.py
ros2 topic list | grep visualizer
ros2 topic echo /controller/position/output --once
```

期望日志：

- 地图初始化成功，显示点数、有效点数和边界。
- 第一次 RViz 点击后显示 `Start received`。
- 第二次 RViz 点击后显示 `Goal received`。
- RRT route 点数正常。
- h_polys 数量正常。
- GCOPTER trajectory pieces 和 duration 正常。
