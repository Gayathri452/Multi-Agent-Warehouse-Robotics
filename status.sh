#!/bin/bash
# Health check for the whole UGP system. Run while the bringup is running.
source /opt/ros/jazzy/setup.bash
source ~/ugp_ws/install/setup.bash

state() {   # lifecycle state with retries: discovery can be slow under load
  local out
  for i in 1 2 3 4; do
    out=$(ros2 lifecycle get "$1" 2>&1 | tail -1)
    case "$out" in *"not found"*) sleep 1 ;; *) break ;; esac
  done
  echo "$out"
}

echo "=== lifecycle states (want: active [3]) ==="
for r in robot_1 robot_2; do
  for n in amcl planner_server controller_server bt_navigator; do
    printf "%-30s" "/$r/$n"
    state "/$r/$n"
  done
done

echo; echo "=== map -> odom, published by AMCL (want: a Translation line) ==="
for r in robot_1 robot_2; do
  printf "%-10s" "$r"
  timeout 10 ros2 run tf2_ros tf2_echo map $r/odom --ros-args -p use_sim_time:=true \
    -r /tf:=/$r/tf -r /tf_static:=/$r/tf_static 2>&1 | grep -m1 -E "Translation|does not exist" || echo "no output"
done

echo; echo "=== simulation speed (want > 0.5) ==="
timeout 5 gz topic -e -t /stats -n 1 2>/dev/null | grep real_time_factor
