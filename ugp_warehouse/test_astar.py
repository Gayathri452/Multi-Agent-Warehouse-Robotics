"""Tests for A* (no ROS, no simulator).  Run:  python3 test_astar.py"""
import os
import random
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from astar import astar, simplify          # noqa: E402
from grid_map import GridMap               # noqa: E402

MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'warehouse_map.yaml')


def make_grid(rows):
    """Tiny test grid from text rows: '#' = blocked, anything else free."""
    free = np.array([[ch != '#' for ch in row] for row in rows])
    return GridMap(free, 1.0, 0.0, 0.0)


def bfs_length(grid, start, goal):
    """Reference shortest path length (number of cells) to cross-check A*."""
    dist = {start: 1}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        if cell == goal:
            return dist[cell]
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nxt = (cell[0] + dr, cell[1] + dc)
            if grid.is_free(nxt) and nxt not in dist:
                dist[nxt] = dist[cell] + 1
                queue.append(nxt)
    return None


def check(name, ok):
    print(('PASS  ' if ok else 'FAIL  ') + name)
    if not ok:
        check.failed += 1


check.failed = 0


def draw(grid, path, step=1):
    """ASCII picture: # blocked, . free, * path, S start, G goal."""
    cells = set(path or [])
    for r in range(0, grid.rows, step):
        line = ''
        for c in range(grid.cols):
            cell = (r, c)
            if path and cell == path[0]:
                line += 'S'
            elif path and cell == path[-1]:
                line += 'G'
            elif cell in cells:
                line += '*'
            else:
                line += '.' if grid.free[cell] else '#'
        print(line)


def small_tests():
    print('--- small maps ---')
    g = make_grid(['.....', '.....', '.....'])
    p = astar(g, (0, 0), (2, 4))
    check('open map: shortest length', p is not None and len(p) == 7)

    g = make_grid(['..#..', '..#..', '.....'])
    p = astar(g, (0, 0), (0, 4))
    check('wall with a gap: goes through the gap', p is not None and (2, 2) in p)

    g = make_grid(['#######', '.......', '#######'])
    p = astar(g, (1, 0), (1, 6))
    check('one-cell-wide corridor', p is not None and len(p) == 7)

    g = make_grid(['..#..', '..#..', '..#..'])
    check('no path: returns None', astar(g, (0, 0), (0, 4)) is None)

    g = make_grid(['.#.', '...'])
    check('goal inside an obstacle: None', astar(g, (1, 0), (0, 1)) is None)
    check('start == goal', astar(g, (1, 0), (1, 0)) == [(1, 0)])

    g = make_grid(['.....'])
    p = astar(g, (0, 0), (0, 4))
    check('simplify keeps only the ends on a straight line',
          simplify(p) == [(0, 0), (0, 4)])


def warehouse_tests():
    print('--- warehouse map ---')
    grid = GridMap.from_yaml(MAP_YAML)
    print('planning grid: %d rows x %d cols, cell %.2f m, free cells %d'
          % (grid.rows, grid.cols, grid.cell, int(grid.free.sum())))

    spots = {'P1': (-8.2, 2.7), 'P2': (-8.2, -2.7), 'D1': (8.2, 2.7),
             'D2': (8.2, -2.7), 'robot_1 start': (-8.0, 0.0)}
    cells = {n: grid.world_to_cell(*xy) for n, xy in spots.items()}
    for n, c in cells.items():
        check('%s cell %s is free' % (n, c), grid.is_free(c))

    for a, b in (('P1', 'D1'), ('P2', 'D2'), ('robot_1 start', 'P1'), ('P1', 'D2')):
        t0 = time.time()
        p = astar(grid, cells[a], cells[b])
        ms = (time.time() - t0) * 1000
        ok = p is not None and len(p) == bfs_length(grid, cells[a], cells[b])
        wp = simplify(p) if p else []
        print('      %s -> %s: %d cells (%.1f m), %d waypoints, %.0f ms'
              % (a, b, len(p or []), (len(p or []) - 1) * grid.cell, len(wp), ms))
        check('%s -> %s is a shortest path (matches BFS)' % (a, b), ok)

    inside_shelf = grid.world_to_cell(-4.5, 4.2)
    check('goal inside a shelf: no path', astar(grid, cells['P1'], inside_shelf) is None)

    random.seed(1)
    free_cells = [tuple(c) for c in np.argwhere(grid.free)]
    bad = 0
    for _ in range(30):
        a, b = random.sample(free_cells, 2)
        p = astar(grid, a, b)
        ref = bfs_length(grid, a, b)
        if (p is None) != (ref is None) or (p and len(p) != ref):
            bad += 1
    check('30 random pairs match BFS (%d mismatches)' % bad, bad == 0)

    print('\nP1 -> D1  (# blocked, . free, * path, S start, G goal)')
    draw(grid, astar(grid, cells['P1'], cells['D1']))


if __name__ == '__main__':
    small_tests()
    warehouse_tests()
    print('\n%s' % ('ALL TESTS PASSED' if check.failed == 0
                    else '%d TEST(S) FAILED' % check.failed))
    sys.exit(1 if check.failed else 0)
