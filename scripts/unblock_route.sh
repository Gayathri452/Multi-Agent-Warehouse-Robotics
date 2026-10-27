#!/bin/bash
# Remove a box dropped by block_route.sh:   bash scripts/unblock_route.sh [name]
NAME=${1:-blocker}
gz service -s /world/warehouse/remove --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"$NAME\" type: MODEL" \
  && echo "Removed '$NAME'. The task manager stops avoiding that area after block_seconds."
