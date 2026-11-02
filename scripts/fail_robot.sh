#!/bin/bash
# Phase 7 fault injection: make one robot break down in the middle of the mission.
#   bash scripts/fail_robot.sh robot_2
# Kills that robot's Nav2 (its navigation software crashes) and stops its wheels.
# Localization (AMCL) keeps running, so revive_robot.sh can bring it back later.
ROBOT=${1:-robot_2}
n=0
for pid in $(pgrep -f -- "__ns:=/$ROBOT( |$)"); do
  cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null)
  case "$cmd" in *amcl*|*lifecycle_manager_localization*) continue ;; esac
  kill "$pid" 2>/dev/null && n=$((n + 1))
done
if [ "$n" -eq 0 ]; then
  echo "No Nav2 processes found for $ROBOT (is the simulation running? already failed?)"
  exit 1
fi
echo "$ROBOT: navigation crashed ($n processes stopped). Stopping its wheels..."
# Gazebo keeps executing the last velocity command, so send zero a few times.
timeout 10 ros2 topic pub --times 5 --rate 5 /$ROBOT/cmd_vel geometry_msgs/msg/Twist "{}" \
  > /dev/null 2>&1
echo "$ROBOT is down. Watch the task manager: OUT OF SERVICE within a few seconds,"
echo "then its task goes back in the queue and another robot takes it."
