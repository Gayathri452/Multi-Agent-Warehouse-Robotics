#!/bin/bash
# Phase 7 fault injection: drop a box into an aisle so a route is blocked.
#   bash scripts/block_route.sh              (default: north aisle, between P1 and the centre)
#   bash scripts/block_route.sh X Y [name]   (any spot, in metres)
# The box is 0.6 m x 2.2 m, wide enough to close a 2 m aisle.
X=${1:--4.5}; Y=${2:-2.7}; NAME=${3:-blocker}
SDF="<sdf version='1.9'><model name='$NAME'><static>true</static><link name='link'>\
<collision name='c'><geometry><box><size>0.6 2.2 1.0</size></box></geometry></collision>\
<visual name='v'><geometry><box><size>0.6 2.2 1.0</size></box></geometry>\
<material><diffuse>0.9 0.1 0.1 1</diffuse><ambient>0.9 0.1 0.1 1</ambient></material>\
</visual></link></model></sdf>"
if ! command -v gz > /dev/null; then
  echo "The 'gz' command is not available. Instead, in the Gazebo window click the box"
  echo "icon in the top toolbar and click in the aisle at ($X, $Y) to drop a box there."
  exit 1
fi
gz service -s /world/warehouse/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf: \"$SDF\" pose: {position: {x: $X, y: $Y, z: 0.5}} name: \"$NAME\"" \
  && echo "Dropped '$NAME' at ($X, $Y). Remove it with: bash scripts/unblock_route.sh $NAME"
