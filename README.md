# UGP: Resilient Multi-Robot Warehouse Logistics

ROS 2 Jazzy + Gazebo Harmonic + Nav2 simulation of differential-drive robots in a warehouse.
Current state: one robot (`robot_1`) navigating with Nav2 and AMCL. A*, MAPF/CBS, task assignment and failure recovery are added in later phases.

## Requirements

- Ubuntu 24.04 (native, or WSL2 with GUI support)
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup python3-colcon-common-extensions python3-numpy
```

## Get and build

```bash
mkdir -p ~/ugp_ws/src && cd ~/ugp_ws/src
git clone https://github.com/Gayathri452/Multi-Agent-Warehouse-Robotics.git ugp_warehouse
cd ~/ugp_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs both of these first:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ugp_ws/install/setup.bash
```

If other people share your network, pick your own number so you do not see their robots:
```bash
export ROS_DOMAIN_ID=42
```

## Run (4 terminals, in this order)

| # | Command | Wait for |
|---|---|---|
| 1 | `ros2 launch ugp_warehouse warehouse.launch.py` | Gazebo opens and the orange robot appears (about 10 s) |
| 2 | `ros2 launch ugp_warehouse map.launch.py` | map server active |
| 3 | `ros2 launch ugp_warehouse amcl.launch.py` | "Received a 400 X 280 map" |
| 4 | `ros2 launch ugp_warehouse nav2.launch.py` | all Nav2 nodes active (about 20 s) |

Starting Nav2 before AMCL fails, because Nav2 needs the `map` frame that AMCL publishes.

Check it is ready (terminal 5):
```bash
ros2 lifecycle get /robot_1/bt_navigator     # expect: active [3]
```

## Test

Send the robot to delivery zone D1 (east side):
```bash
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 8.2, y: 2.7}, orientation: {w: 1.0}}}}" --feedback
```
Expect `Goal finished with status: SUCCEEDED` (about 1 to 1.5 minutes).

Pickup zones: P1 (-8.2, 2.7), P2 (-8.2, -2.7). Delivery zones: D1 (8.2, 2.7), D2 (8.2, -2.7).

Unreachable goal (inside a shelf) should end `ABORTED` with error code 208:
```bash
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -4.5, y: 4.2}, orientation: {w: 1.0}}}}" --feedback
```

## Visualise (optional)

```bash
rviz2 -d ~/ugp_ws/src/ugp_warehouse/rviz/warehouse.rviz --ros-args -p use_sim_time:=true \
  -r /tf:=/robot_1/tf -r /tf_static:=/robot_1/tf_static -r /goal_pose:=/robot_1/goal_pose
```
Use the **2D Goal Pose** button to click a goal on the map.
In RViz, set the Map display's Durability Policy to **Transient Local** if the map does not show.

## Stop everything

```bash
pkill -f "ros2 launch"; pkill -f -- "--ros-args"; pkill -f ruby; pkill -f parameter_bridge
```
Do this before restarting, or old nodes stay alive and cause strange errors.

## Troubleshooting

| Problem | Fix |
|---|---|
| `Package 'ugp_warehouse' not found` | You did not source `install/setup.bash` in this terminal |
| Goal rejected: "Action server is inactive" | Nav2 did not finish starting. Stop everything and start again in order |
| `Timed out waiting for transform ... map` | AMCL is not running yet. Start terminal 3 before terminal 4 |
| Robot or Gazebo is very slow | Close other apps. Lidar rendering is CPU-heavy without a GPU |
| Map missing in RViz | Map display: Durability Policy `Transient Local` |

## Layout

| Path | Purpose |
|---|---|
| `launch/` | `warehouse` (sim + robot), `map`, `amcl`, `nav2` |
| `worlds/`, `models/` | Warehouse world and robot model |
| `config/` | Occupancy map and Nav2 parameters |
| `make_world.py`, `make_map.py`, `make_nav2_params.py` | Generators for the files above |# Multi-Agent-Warehouse-Robotics
