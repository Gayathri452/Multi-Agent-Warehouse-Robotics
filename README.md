# UGP: Resilient Multi-Robot Warehouse Logistics

ROS 2 Jazzy + Gazebo Harmonic + Nav2 simulation of differential-drive robots in a warehouse.
Current state: two robots with Nav2 and AMCL, A* planning, task assignment, and
prioritized space-time coordination. CBS and failure recovery are later phases.

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

## Run

Start the complete two-robot system:
```bash
ros2 launch ugp_warehouse bringup.launch.py
```
Wait for Gazebo, both robots, AMCL, and Nav2 to start. In another sourced
terminal, check that both navigators are active:
```bash
ros2 lifecycle get /robot_1/bt_navigator     # expect: active [3]
ros2 lifecycle get /robot_2/bt_navigator     # expect: active [3]
```

Start a coordinated pickup-and-delivery run in another sourced terminal:
```bash
ros2 run ugp_warehouse task_manager --ros-args -p use_sim_time:=true \
  -p "tasks:=['P1:D1','P2:D2']"
```

## Test

Run the Phase 5 space-time coordinator tests (no ROS or simulator required):
```bash
python3 ugp_warehouse/test_space_time.py
```
The tests cover crossing routes with a scheduled wait, shared-goal exclusion,
opposing routes in a narrow corridor, and swap-conflict detection. When the task
manager assigns multiple tasks at once, it coordinates their pickup routes
before sending them to Nav2. A route that cannot be scheduled is reported as
failed rather than sent through a known conflict. Delivery legs still use the
existing per-robot A* waypoint planner; dynamic replanning and failure recovery
are not part of this phase.

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
