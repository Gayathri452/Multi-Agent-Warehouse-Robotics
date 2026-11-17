import os

from ugp_warehouse.layout import WALLS
from ugp_warehouse.locations import LOCATIONS


HERE = os.path.dirname(os.path.abspath(__file__))
WORLD_PATH = os.path.join(HERE, 'worlds', 'warehouse.sdf')

HEAD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="warehouse">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.94 0.94 0.98 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
"""
TAIL = "  </world>\n</sdf>\n"

FLOOR = (0.82, 0.82, 0.85)
WALL = (0.18, 0.18, 0.2)
GREEN = (0.08, 0.75, 0.2)
BLUE = (0.08, 0.3, 0.9)


def box(name, x, y, z, sx, sy, sz, rgb, collide=True):
    size = f'{sx} {sy} {sz}'
    r, g, b = rgb
    collision = ''
    if collide:
        collision = (
            f"<collision name='collision'><geometry><box><size>{size}</size>"
            "</box></geometry></collision>"
        )
    return (
        f"    <model name='{name}'><static>true</static>"
        f"<pose>{x} {y} {z} 0 0 0</pose><link name='link'>{collision}"
        f"<visual name='visual'><geometry><box><size>{size}</size></box></geometry>"
        f"<material><ambient>{r} {g} {b} 1</ambient>"
        f"<diffuse>{r} {g} {b} 1</diffuse></material></visual>"
        "</link></model>\n"
    )


world = HEAD
world += box('floor', 0, 0, -0.05, 30, 30, 0.1, FLOOR)
for name, x, y, sx, sy in WALLS:
    world += box(name, x, y, 0.3, sx, sy, 0.6, WALL)

for index, key in enumerate(('P1', 'P2'), 1):
    x, y = LOCATIONS[key]
    world += box(f'pickup_{index}', x, y, 0.01, 0.8, 0.8, 0.02, GREEN, False)
for index, key in enumerate(('D1', 'D2'), 1):
    x, y = LOCATIONS[key]
    world += box(f'delivery_{index}', x, y, 0.01, 0.8, 0.8, 0.02, BLUE, False)
world += TAIL

with open(WORLD_PATH, 'w', encoding='utf-8') as sdf_file:
    sdf_file.write(world)
print('wrote', WORLD_PATH)
