#!/bin/bash
# Phase 7: bring a robot back after fail_robot.sh, by restarting its Nav2.
#   bash scripts/revive_robot.sh robot_2        (leave it running in its own terminal)
# After Nav2 is up and healthy for revive_seconds (10 s), the task manager logs
# BACK IN SERVICE and gives the robot tasks again.
ROBOT=${1:-robot_2}
echo "Restarting Nav2 for $ROBOT (takes about 20 s). Leave this terminal open."
exec ros2 launch ugp_warehouse nav2.launch.py robot:=$ROBOT
