import os

HEAD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="warehouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.8 0.85 0.9 1</background>
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


def box(name, x, y, z, sx, sy, sz, rgb, collide=True):
    size = f"{sx} {sy} {sz}"
    r, g, b = rgb
    col = ""
    if collide:
        col = (f"<collision name='c'><geometry><box><size>{size}</size>"
               f"</box></geometry></collision>")
    return (f"    <model name='{name}'><static>true</static>"
            f"<pose>{x} {y} {z} 0 0 0</pose><link name='l'>{col}"
            f"<visual name='v'><geometry><box><size>{size}</size></box></geometry>"
            f"<material><ambient>{r} {g} {b} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse></material></visual>"
            f"</link></model>\n")


GRAY, WALL, BROWN = (0.75, 0.75, 0.75), (0.5, 0.5, 0.55), (0.6, 0.4, 0.2)
GREEN, BLUE = (0.1, 0.7, 0.2), (0.1, 0.3, 0.9)

out = HEAD
out += box("floor", 0, 0, -0.05, 30, 30, 0.1, GRAY)
out += box("wall_n", 0, 6.1, 1, 18.4, 0.2, 2, WALL)
out += box("wall_s", 0, -6.1, 1, 18.4, 0.2, 2, WALL)
out += box("wall_e", 9.1, 0, 1, 0.2, 12, 2, WALL)
out += box("wall_w", -9.1, 0, 1, 0.2, 12, 2, WALL)

for row, y in zip("abcd", (4.2, 1.2, -1.2, -4.2)):
    out += box(f"shelf_{row}1", -4.5, y, 0.75, 6, 0.6, 1.5, BROWN)
    out += box(f"shelf_{row}2", 4.5, y, 0.75, 6, 0.6, 1.5, BROWN)

out += box("pickup_1", -8.2, 2.7, 0.01, 1, 1, 0.02, GREEN, False)
out += box("pickup_2", -8.2, -2.7, 0.01, 1, 1, 0.02, GREEN, False)
out += box("delivery_1", 8.2, 2.7, 0.01, 1, 1, 0.02, BLUE, False)
out += box("delivery_2", 8.2, -2.7, 0.01, 1, 1, 0.02, BLUE, False)
out += TAIL

path = os.path.expanduser("~/ugp_ws/src/ugp_warehouse/worlds/warehouse.sdf")
with open(path, "w") as f:
    f.write(out)
print("wrote", path)
