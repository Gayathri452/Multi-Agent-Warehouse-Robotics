#!/bin/bash
# Stop every UGP process: simulation, bridge, Nav2, AMCL, map server, relays.
pkill -f "ros2 launch"
pkill -f -- "--ros-args"
pkill -f "gz sim"
pkill -f ruby
pkill -f parameter_bridge
pkill -f static_transform_publisher
sleep 2
ros2 daemon stop > /dev/null 2>&1
echo "all UGP processes stopped"
