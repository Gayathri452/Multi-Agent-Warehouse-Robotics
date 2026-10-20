import os

import yaml

SRC = '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
DST = os.path.expanduser('~/ugp_ws/src/ugp_warehouse/config/nav2_params.yaml')
ROBOT = 'robot_1'

FRAMES = {
    'base_link': ROBOT + '/base_link',
    'base_footprint': ROBOT + '/base_link',
    'odom': ROBOT + '/odom',
}
TOPICS = {'/scan': 'scan', '/odom': 'odom', '/local_costmap/published_footprint': 'local_costmap/published_footprint'}
TOPIC_KEYS = ('topic', 'scan_topic', 'odom_topic', 'footprint_topic')
changes = []


def note(path, old, new):
    changes.append(f'{path}: {old} -> {new}')


def walk(node, path=''):
    if isinstance(node, list):
        for i, item in enumerate(node):
            walk(item, f'{path}[{i}]')
        return
    if not isinstance(node, dict):
        return
    for key in list(node):
        value = node[key]
        here = f'{path}/{key}'
        if isinstance(value, (dict, list)):
            if key == 'static_layer' and isinstance(value, dict):
                value['map_topic'] = '/map'
                note(here + '/map_topic', 'default', '/map')
            walk(value, here)
        elif isinstance(value, str):
            if 'frame' in key and value in FRAMES:
                node[key] = FRAMES[value]
                note(here, value, FRAMES[value])
            elif key in TOPIC_KEYS and value in TOPICS:
                node[key] = TOPICS[value]
                note(here, value, TOPICS[value])
        elif key == 'robot_radius':
            note(here, value, 0.38)
            node[key] = 0.38
        elif key == 'inflation_radius':
            note(here, value, 0.55)
            node[key] = 0.55


with open(SRC) as f:
    params = yaml.safe_load(f)

walk(params)

with open(DST, 'w') as f:
    yaml.safe_dump(params, f, sort_keys=False)

for line in changes:
    print(line)
print(len(changes), 'changes written to', DST)
cs = params.get('controller_server', {}).get('ros__parameters', {})
print('controller plugin:', cs.get('FollowPath', {}).get('plugin'))
