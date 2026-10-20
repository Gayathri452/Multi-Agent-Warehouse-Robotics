"""Warehouse occupancy map -> coarse planning grid.

Pure Python + numpy (no ROS needed), so A*, MAPF and CBS can be tested quickly.

Pipeline: PGM/YAML map -> inflate obstacles by the robot size -> shrink to a
coarse grid (default 0.25 m cells). A coarse cell is free only if every fine
cell inside it is free, so a robot standing in a free cell cannot touch a wall.

Cells are (row, col). Row 0 is the TOP of the map (largest y), like the image.
"""
import math
import os

import numpy as np
import yaml


def read_pgm(path):
    """Read a binary (P5) PGM file into a uint8 array of shape (height, width)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # P5, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':  # comment line
            while data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1  # single whitespace after maxval
    if tokens[0] != b'P5':
        raise ValueError('only binary PGM (P5) is supported')
    width, height = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, np.uint8, width * height, pos).reshape(height, width)


def dilate(mask, radius):
    """Grow True cells of a boolean array by `radius` cells (disc shaped)."""
    out = mask.copy()
    h, w = mask.shape
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if (dr or dc) and dr * dr + dc * dc <= radius * radius:
                dst_r = slice(max(dr, 0), h + min(dr, 0))
                src_r = slice(max(-dr, 0), h + min(-dr, 0))
                dst_c = slice(max(dc, 0), w + min(dc, 0))
                src_c = slice(max(-dc, 0), w + min(-dc, 0))
                out[dst_r, dst_c] |= mask[src_r, src_c]
    return out


class GridMap:
    """A boolean grid: free[r, c] is True where a robot CENTER may be."""

    def __init__(self, free, cell_size, origin_x, origin_y):
        self.free = free
        self.cell = cell_size
        self.ox = origin_x
        self.oy = origin_y
        self.rows, self.cols = free.shape

    @classmethod
    def from_yaml(cls, yaml_path, robot_radius=0.38, margin=0.07, cell_size=0.25):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
        img = read_pgm(os.path.join(os.path.dirname(yaml_path), meta['image']))
        fine_free = img >= 250          # trinary map: 254 free, 0 wall, 128 unknown
        inflated = dilate(~fine_free, int(math.ceil((robot_radius + margin) / res)))
        f = int(round(cell_size / res))
        h, w = inflated.shape
        if h % f or w % f:
            raise ValueError('map size %dx%d is not divisible by %d' % (w, h, f))
        coarse_blocked = inflated.reshape(h // f, f, w // f, f).any(axis=(1, 3))
        return cls(~coarse_blocked, f * res, ox, oy)

    # ---- coordinate conversion -------------------------------------------
    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.ox) / self.cell))
        row = self.rows - 1 - int(math.floor((y - self.oy) / self.cell))
        return (row, col)

    def cell_to_world(self, cell):
        row, col = cell
        return (self.ox + (col + 0.5) * self.cell,
                self.oy + (self.rows - row - 0.5) * self.cell)

    # ---- queries ---------------------------------------------------------
    def in_bounds(self, cell):
        return 0 <= cell[0] < self.rows and 0 <= cell[1] < self.cols

    def is_free(self, cell):
        return self.in_bounds(cell) and bool(self.free[cell])

    def nearest_free(self, cell, max_radius=6):
        """Closest free cell to `cell` (itself if free). None if none nearby."""
        if self.is_free(cell):
            return cell
        best, best_d = None, None
        for dr in range(-max_radius, max_radius + 1):
            for dc in range(-max_radius, max_radius + 1):
                c = (cell[0] + dr, cell[1] + dc)
                d = dr * dr + dc * dc
                if self.is_free(c) and (best_d is None or d < best_d):
                    best, best_d = c, d
        return best
