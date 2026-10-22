"""Generate one Nav2 parameter file per robot from Nav2's default file.

Run:  python3 make_nav2_params.py
Writes config/nav2_params_<robot>.yaml for every robot in ugp_warehouse/robots.py.

What changes for robot N (everything else stays at Nav2's defaults):
  * frame names   base_link / base_footprint -> <N>/base_link,  odom -> <N>/odom
  * topic names   /scan -> scan, /odom -> odom (relative, so they live under /<N>/)
  * footprint topic for the collision monitor -> relative
  * static_layer map topic -> /map (all robots share one map server)
  * robot_radius 0.38 m, inflation_radius 0.55 m
"""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ugp_warehouse.robots import ROBOTS  # noqa: E402

DEFAULT = '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
OUT_DIR = os.path.join(HERE, 'config')
TOPIC_KEYS = ('topic', 'scan_topic', 'odom_topic', 'footprint_topic')
TOPICS = {
    '/scan': 'scan',
    '/odom': 'odom',
    '/local_costmap/published_footprint': 'local_costmap/published_footprint',
}
ROBOT_RADIUS = 0.38
INFLATION_RADIUS = 0.55


def rewrite(node, name, changes, path=''):
    """Walk the YAML tree and rewrite the values listed above, recording each change."""
    if isinstance(node, list):
        for item in node:
            rewrite(item, name, changes, path)
        return
    if not isinstance(node, dict):
        return
    for key in list(node.keys()):
        value = node[key]
        here = '%s/%s' % (path, key)
        if isinstance(value, (dict, list)):
            rewrite(value, name, changes, here)
            if key == 'static_layer' and isinstance(value, dict) and value.get('map_topic') != '/map':
                changes.append('%s/map_topic: %s -> /map' % (here, value.get('map_topic')))
                value['map_topic'] = '/map'
            continue
        new = value
        if 'frame' in str(key) and value in ('base_link', 'base_footprint'):
            new = name + '/base_link'
        elif 'frame' in str(key) and value == 'odom':
            new = name + '/odom'
        elif key in TOPIC_KEYS and value in TOPICS:
            new = TOPICS[value]
        elif key == 'robot_radius':
            new = ROBOT_RADIUS
        elif key == 'inflation_radius':
            new = INFLATION_RADIUS
        if new != value:
            node[key] = new
            changes.append('%s: %s -> %s' % (here, value, new))


def main():
    for name, _, _, _ in ROBOTS:
        with open(DEFAULT) as f:
            params = yaml.safe_load(f)
        changes = []
        rewrite(params, name, changes)
        out = os.path.join(OUT_DIR, 'nav2_params_%s.yaml' % name)
        with open(out, 'w') as f:
            yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
        print('%s: %d changes -> %s' % (name, len(changes), out))
        if name == ROBOTS[0][0]:
            for line in changes:
                print('   ' + line)
        ctrl = params.get('controller_server', {}).get('ros__parameters', {})
        print('   controller plugin:', (ctrl.get('FollowPath') or {}).get('plugin'))


if __name__ == '__main__':
    main()
