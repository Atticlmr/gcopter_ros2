# gcopter_ros2

English | [中文](README.zh-CN.md)

`gcopter_ros2` is an independent ROS 2 port of the original `gcopter` ROS 1 demo.
The GCOPTER core algorithm headers are included in this package, so it does not
depend on `../gcopter` at build time.

Original GCOPTER repository: https://github.com/ZJU-FAST-Lab/GCOPTER

## Dependencies

The full RViz simulation uses `mockamap` to publish the default point cloud map on
`/mock_map`.

Mockamap repository:

```bash
git clone https://github.com/Atticlmr/mockamap.git
```

Place `mockamap` in the same ROS 2 workspace `src` directory as this package when
you want to run the provided simulation launch files.

Other expected dependencies include:

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

## Node

### `gcopter_ros2_planner`

Inputs:

- `/mock_map` (`sensor_msgs/msg/PointCloud2`): point cloud map, using `map` as the default frame.
- `/move_base_simple/goal` (`geometry_msgs/msg/PoseStamped`): RViz 2D Goal Pose input. The first click is used as the start, the second click is used as the goal, and the next click resets the pair.

Outputs:

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

`/controller/position/output` publishes NED setpoints by default, controlled by
`enu_to_ned: true`. This node does not publish PX4 offboard heartbeat messages and
does not send vehicle commands.

## Build

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select mockamap gcopter_ros2
```

If you only need to build the planner package and already have all dependencies
available:

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select gcopter_ros2
```

## Launch

Launch only the planner:

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 gcopter_ros2_planner.launch.py
```

Launch the complete RViz simulation with `mockamap`, planner, RViz, and `rqt_plot`:

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 global_planning.launch.py
```

`gcopter_sim.launch.py` also starts `mockamap`, the planner, RViz, and `rqt_plot`:

```bash
source /home/li/Desktop/ws_ros2/install/setup.bash
ros2 launch gcopter_ros2 gcopter_sim.launch.py
```

If `mockamap` is not running, the planner will not receive the default `/mock_map`
point cloud. In that case, either start `mockamap` or change `map_topic` in
`config/gcopter_ros2.yaml` to a different `sensor_msgs/msg/PointCloud2` source.

## RViz Workflow

1. Keep the RViz Fixed Frame as `map`.
2. Wait until the terminal reports that the map has been initialized.
3. Use RViz `2D Goal Pose` for the first click to set the start.
4. Click a second time to set the goal. The node then runs RRT, FIRI safe corridor generation, and GCOPTER optimization.
5. Click again to clear the previous start/goal pair and set a new start.

## Default mockamap Parameters

`global_planning.launch.py` and `gcopter_sim.launch.py` use the following map
parameters:

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
