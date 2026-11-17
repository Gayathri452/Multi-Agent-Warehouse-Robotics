# Multi-Agent Warehouse Robotics

A ROS 2 Jazzy and Gazebo Harmonic simulation of three differential-drive robots
performing coordinated warehouse pickup-and-delivery tasks. Nav2 handles motion;
the project adds task assignment, joint multi-robot route planning, collision
avoidance, and robot failure recovery.

## What the project demonstrates

- **Three simulated robots** (`robot_1`, `robot_2`, `robot_3`) navigating a shared
  warehouse with AMCL localization and Nav2.
- **Pickup-and-delivery task assignment** using A* route costs and robot workload.
- **Coordinated route planning** using Conflict-Based Search (CBS). Robots are
  paced by observed progress, wait for one another when necessary, and queue
  rather than driving to the same pickup or delivery zone at once.
- **Failure detection and recovery**. A robot that loses odometry or navigation
  service is taken out of service; its task can be reassigned and the fleet
  replans around it.

The standard demo below runs **eight tasks** with all three robots. Tasks are
assigned dynamically, so which robot performs each task can vary.

## Requirements

- Ubuntu 24.04 (native or WSL2 with a working graphical desktop)
- ROS 2 Jazzy
- Gazebo Harmonic

Install ROS 2 Jazzy first using the
[official Ubuntu installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Then install the simulation and build dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-rosdep
```

For screen recording, install OBS Studio too:

```bash
sudo apt install -y obs-studio
```

## Get the source and build

For a new checkout:

```bash
mkdir -p ~/ugp_ws/src
cd ~/ugp_ws/src
git clone -b main --single-branch \
  https://github.com/Gayathri452/Multi-Agent-Warehouse-Robotics.git \
  ugp_warehouse
```

If `~/ugp_ws/src/ugp_warehouse` already exists, update it instead of cloning
again:

```bash
cd ~/ugp_ws/src/ugp_warehouse
git switch main
git pull origin main
```

Install any remaining ROS package dependencies and build:

```bash
sudo rosdep init  # Run once per Ubuntu installation; skip if already initialized.
rosdep update
cd ~/ugp_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Every new terminal needs the ROS and workspace environments. Use this at the
start of **each** terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ugp_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

The domain ID keeps this demo's ROS topics separate from other ROS systems on
the same network. Use the same ID in every terminal.

## Run the three-robot, eight-task demo

### 1. Start Gazebo and the robots

In Terminal 1, source the environment as above, then run:

```bash
ros2 launch ugp_warehouse bringup.launch.py
```

Keep this terminal open. Gazebo, the warehouse, and the three robots will start;
AMCL and Nav2 launch in stages. On a first launch, allow about a minute for all
three robots' navigation systems to become active.

### 2. (Optional) Set Gazebo to a top-down camera view

The camera helper requires Gazebo to have created its GUI configuration at
least once. If this is the first launch, wait for Gazebo to open, press **Ctrl+C**
in Terminal 1, and run the helper from a sourced terminal:

```bash
cd ~/ugp_ws/src/ugp_warehouse
python3 scripts/set_gazebo_top_view.py
```

The helper saves a backup at `~/.gz/sim/8/gui.config.bak`. Restart the launch
command from Step 1; the top-down view is applied when Gazebo opens. This changes
only the camera view, not the world or robot behavior.

### 3. Verify that navigation is ready

In Terminal 2, source the environment as above, then check the lifecycle state
of each robot:

```bash
for robot in robot_1 robot_2 robot_3; do
  for node in amcl planner_server controller_server bt_navigator; do
    printf '%s/%s: ' "$robot" "$node"
    ros2 lifecycle get "/$robot/$node"
  done
done
```

Each node should report `active [3]`. If any are still starting, wait a little
longer and run the check again.

### 4. Start all eight tasks

In Terminal 3, source the environment as above, then run:

```bash
ros2 run ugp_warehouse task_manager --ros-args \
  -p use_sim_time:=true \
  -p "tasks:=['P1:D1','P2:D2','P1:D2','P2:D1','D1:P1','D2:P2','D1:P2','D2:P1']"
```

Each entry is `pickup:delivery`. The task manager assigns work to available
robots and logs assignments, CBS plans, waits, and task progress. Run duration
depends on computer performance and simulation speed. A successful run ends
with:

```text
All tasks finished: 8 done, 0 failed
```

To watch the live task-status message in another terminal, source the environment
and run:

```bash
ros2 topic echo /task_status std_msgs/msg/String
```

## Tests

The planner, assignment, fleet coordination, and recovery tests can be run
without launching ROS or Gazebo. From the repository root:

```bash
python3 ugp_warehouse/test_astar.py
python3 ugp_warehouse/test_tasks.py
python3 ugp_warehouse/test_space_time.py
python3 ugp_warehouse/test_cbs.py
python3 ugp_warehouse/test_health.py
python3 ugp_warehouse/test_fleet.py
```

`test_fleet.py` runs multiple simulated fleet scenarios and may take about a
minute. Each test script prints its checks and reports failures.

## Optional: demonstrate failure recovery

While the task manager is running, use another sourced terminal from the
repository root to stop a robot's navigation:

```bash
bash scripts/fail_robot.sh robot_2
```

The task manager should report the robot out of service and reassign its task
when possible. Restore the robot with:

```bash
bash scripts/revive_robot.sh robot_2
```

The revive script must remain running in its terminal while Nav2 restarts.
Other supported demonstrations are `scripts/block_route.sh` and
`scripts/unblock_route.sh`.

## Warehouse locations

| Location | Map coordinates (m) | Use |
|---|---:|---|
| P1 | (-7.0, -4.8) | Pickup |
| P2 | (-7.0, 4.8) | Pickup |
| D1 | (7.0, 4.8) | Delivery |
| D2 | (7.0, -4.8) | Delivery |

## Project structure

| Path | Responsibility |
|---|---|
| `launch/` | Starts Gazebo, map, AMCL, TF relay, and Nav2 |
| `worlds/`, `models/` | Warehouse world and robot model |
| `config/` | Map and per-robot Nav2 parameters |
| `ugp_warehouse/task_manager.py` | ROS task execution, status, and recovery coordination |
| `ugp_warehouse/tasks.py` | Task state and cost-based robot assignment |
| `ugp_warehouse/astar.py` | Grid-based route planning |
| `ugp_warehouse/cbs.py`, `fleet.py` | Joint conflict resolution and progress-based fleet pacing |
| `ugp_warehouse/health.py` | Robot health monitoring and recovery |
| `scripts/` | Camera setup and fault-injection helpers |
| `make_world.py`, `make_map.py` | Regenerate world/map from the shared layout |

## Troubleshooting

| Symptom | What to check |
|---|---|
| `Package 'ugp_warehouse' not found` | Source both `/opt/ros/jazzy/setup.bash` and `~/ugp_ws/install/setup.bash` in that terminal. |
| A lifecycle node is not `active [3]` | Keep the launch terminal open and wait for startup; check again after a short delay. |
| A robot is missing in Gazebo | Check the launch terminal for spawn errors and verify its odometry, e.g. `ros2 topic echo /robot_2/odom --once`. |
| Gazebo is slow | Close other applications; lidar rendering is CPU/GPU intensive. |
| Map is missing in RViz | Set the Map display's durability policy to **Transient Local**. |
| Robots/topics are not visible across terminals | Confirm every terminal uses the same `ROS_DOMAIN_ID`. |
