"""Shared warehouse geometry used by the Gazebo world and Nav2 map."""

WALLS = (
    # name, x, y, width, depth
    ('wall_n', 0.0, 6.1, 18.4, 0.2),
    ('wall_s', 0.0, -6.1, 18.4, 0.2),
    ('wall_e', 9.1, 0.0, 0.2, 12.0),
    ('wall_w', -9.1, 0.0, 0.2, 12.0),
    ('partition_nw', -6.75, 2.0, 4.5, 0.2),
    ('partition_nc', 0.0, 2.0, 5.0, 0.2),
    ('partition_ne', 6.75, 2.0, 4.5, 0.2),
    ('partition_sw', -6.75, -2.0, 4.5, 0.2),
    ('partition_sc', 0.0, -2.0, 5.0, 0.2),
    ('partition_se', 6.75, -2.0, 4.5, 0.2),
    ('partition_nw_end', -4.5, 3.0, 0.2, 2.0),
    ('partition_nc_end', -2.5, 3.0, 0.2, 2.0),
    ('partition_ne_end', 2.5, 4.0, 0.2, 4.0),
    ('partition_ne_inner', 4.5, 3.0, 0.2, 2.0),
    ('partition_sw_end', -4.5, -3.0, 0.2, 2.0),
    ('partition_sc_end', -2.5, -3.0, 0.2, 2.0),
    ('partition_se_end', 2.5, -4.0, 0.2, 4.0),
    ('partition_se_inner', 4.5, -3.0, 0.2, 2.0),
)
