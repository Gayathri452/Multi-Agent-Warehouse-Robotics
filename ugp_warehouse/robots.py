"""Single source of truth for the robots: (name, spawn x, spawn y, spawn yaw).

Used by warehouse.launch.py, bringup.launch.py and make_nav2_params.py.
Add a robot here and re-run make_nav2_params.py + ugp-build.
"""
ROBOTS = [
    ('robot_1', -8.0, 0.0, 0.0),
    ('robot_2', -8.3, -1.0, -1.5708),
]
