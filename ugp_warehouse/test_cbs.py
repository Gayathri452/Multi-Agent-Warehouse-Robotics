"""Tests for Phase 6: prioritized planning and CBS (no ROS).  Run:  python3 test_cbs.py"""
import itertools
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cbs import (at, cbs, first_conflict, plan_one, prioritized,  # noqa: E402
                 recommended_min_sep, resolve, sum_of_costs, wait_for_rules)
from grid_map import GridMap                                      # noqa: E402
from locations import LOCATIONS                                   # noqa: E402

MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'warehouse_map.yaml')
failed = 0


def check(name, ok):
    global failed
    print(('PASS  ' if ok else 'FAIL  ') + name)
    failed += 0 if ok else 1


def make_grid(rows):
    """Tiny test grid from text rows: '#' = blocked, anything else free."""
    return GridMap(np.array([[ch != '#' for ch in row] for row in rows]), 1.0, 0.0, 0.0)


def draw(grid, paths, rows=None):
    """ASCII picture: # blocked, . free, 1/2/3 = robot paths, X = both."""
    marks = {}
    for i, p in enumerate(paths):
        for cell in p:
            marks[cell] = 'X' if marks.get(cell, str(i + 1)) != str(i + 1) else str(i + 1)
    for r in rows or range(grid.rows):
        line = [marks.get((r, c), '.' if grid.free[r, c] else '#') for c in range(grid.cols)]
        print('      ' + ''.join(line))


# ---- 1. low-level planner -------------------------------------------------------
def low_level_tests():
    print('--- space-time A* with constraints ---')
    g = make_grid(['.....'])
    straight = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]
    check('no constraints: shortest path', plan_one(g, (0, 0), (0, 4)) == straight)
    p = plan_one(g, (0, 0), (0, 4), [('v', 2, {(0, 2)})])
    check('cell forbidden at t=2: waits one step', p is not None and len(p) == 6 and
          at(p, 2) != (0, 2))
    p = plan_one(g, (0, 0), (0, 2), [('v', 7, {(0, 2)})])
    check('goal busy at t=7: parks only after that', p is not None and len(p) - 1 >= 8)
    check('goal blocked forever: None',
          plan_one(g, (0, 0), (0, 4), [('from', 3, {(0, 4)})]) is None)
    p = plan_one(g, (0, 0), (0, 4), [('e', 0, (0, 0), (0, 1))])
    check('move forbidden at t=0: starts one step later', p is not None and len(p) == 6)


# ---- 2. small maps, classic MAPF (min_sep 1) ------------------------------------
def small_map_tests():
    print('--- small maps (classic, same-cell conflicts) ---')
    # crossing at a junction: both arrive at the centre at t=2
    g = make_grid(['##.##', '##.##', '.....', '##.##', '##.##'])
    starts, goals = [(2, 0), (0, 2)], [(2, 4), (4, 2)]
    alone = [plan_one(g, s, t) for s, t in zip(starts, goals)]
    c = first_conflict(alone)
    check('crossing: independent paths clash at the centre at t=2',
          c is not None and c[0] == 'vertex' and c[3] == 2 and c[4] == (2, 2))
    p = cbs(g, starts, goals)
    check('crossing: CBS is conflict-free and costs one wait (8 + 1)',
          p is not None and first_conflict(p) is None and sum_of_costs(p) == 9)
    pr = prioritized(g, starts, goals)
    check('crossing: prioritized also solves it', pr is not None and first_conflict(pr) is None)

    # head-on swap in a corridor with one side pocket
    g = make_grid(['#.###', '.....'])
    starts, goals = [(1, 0), (1, 4)], [(1, 4), (1, 0)]
    c = first_conflict([plan_one(g, s, t) for s, t in zip(starts, goals)])
    check('swap: independent paths clash (vertex or swap)', c is not None)
    p = cbs(g, starts, goals)
    check('swap: CBS uses the pocket, conflict-free',
          p is not None and first_conflict(p) is None and (0, 1) in p[0] + p[1])
    print('      CBS cost %d (each robot alone: 4 + 4)' % sum_of_costs(p))

    # prioritized planning depends on the order, CBS does not
    #   robot 0 (pocket -> corridor cell) parks on robot 1's only route
    g = make_grid(['###.#', '.....'])
    starts, goals = [(0, 3), (1, 0)], [(1, 2), (1, 4)]
    check('order [0, 1]: prioritized FAILS (robot 0 parks in the way)',
          prioritized(g, starts, goals, order=[0, 1]) is None)
    pr = prioritized(g, starts, goals, order=[1, 0])
    check('order [1, 0]: prioritized works', pr is not None and first_conflict(pr) is None)
    p = cbs(g, starts, goals)
    check('CBS solves it without being told an order',
          p is not None and first_conflict(p) is None and sum_of_costs(p) == sum_of_costs(pr))

    # shared goal: impossible when robots park on their goals
    st = {}
    check('shared goal: rejected at once',
          cbs(g, [(1, 0), (1, 4)], [(1, 2), (1, 2)], stats=st) is None and
          'same goal' in st['reason'])

    # one-lane corridor, no pocket: CBS cannot prove "impossible", it gives up
    g = make_grid(['.....'])
    st = {}
    t0 = time.time()
    p = cbs(g, [(0, 0), (0, 4)], [(0, 4), (0, 0)], max_nodes=200, stats=st)
    check('no room to pass: CBS gives up cleanly (%s, %.1f s)' % (st['reason'], time.time() - t0),
          p is None and 'gave up' in st['reason'])


# ---- 3. random instances ----------------------------------------------------------
def random_tests():
    print('--- 40 random 7x7 maps, 3 robots ---')
    rng = random.Random(3)
    bad = worse = solved = 0
    for _ in range(40):
        free = np.array([[rng.random() > 0.2 for _ in range(7)] for _ in range(7)])
        g = GridMap(free, 1.0, 0.0, 0.0)
        cells = [tuple(c) for c in np.argwhere(free)]
        starts, goals = rng.sample(cells, 3), rng.sample(cells, 3)
        opt = cbs(g, starts, goals, max_nodes=400)
        if opt is None:
            continue
        solved += 1
        fast = cbs(g, starts, goals, w=1.5, max_nodes=400)
        pr = prioritized(g, starts, goals)
        bad += first_conflict(opt) is not None or (fast is not None and
                                                   first_conflict(fast) is not None)
        worse += pr is not None and sum_of_costs(pr) < sum_of_costs(opt)
        worse += fast is not None and sum_of_costs(fast) > 1.5 * sum_of_costs(opt)
    check('%d solved instances: every CBS plan is conflict-free' % solved, bad == 0)
    check('CBS (w=1) never costs more than prioritized; w=1.5 stays within 1.5x', worse == 0)


# ---- 4. warehouse map, real robot size ---------------------------------------------
def warehouse_tests():
    print('--- warehouse map, robots 0.76 m wide ---')
    grid = GridMap.from_yaml(MAP_YAML)
    sep = recommended_min_sep(grid.cell)
    check('recommended separation is 4 cells (1.0 m)', sep == 4)
    cell = {k: grid.world_to_cell(*xy) for k, xy in LOCATIONS.items()}

    # the deliberate conflict: two robots drive the same aisle in opposite directions
    starts, goals = [cell['P1'], cell['D1']], [cell['D1'], cell['P1']]
    alone = [plan_one(grid, s, t) for s, t in zip(starts, goals)]
    c = first_conflict(alone, sep)
    if c:
        x, y = grid.cell_to_world(c[4])
        print('      planned alone: robots meet at step %d near (%.1f, %.1f) m' % (c[3], x, y))
    check('deliberate conflict: P1->D1 and D1->P1 clash head-on', c is not None)
    check('same-cell check alone would MISS how close they get',
          first_conflict(alone, 1) is None or first_conflict(alone, 1)[3] > c[3])
    st = {}
    t0 = time.time()
    paths = cbs(grid, starts, goals, min_sep=sep, w=1.1, stats=st)
    ok = paths is not None and first_conflict(paths, sep) is None
    check('CBS resolves it automatically (%d nodes, %.2f s)' % (st['expanded'], time.time() - t0),
          ok)
    if ok:
        for i, (p, a) in enumerate(zip(paths, alone)):
            print('      robot %d: %d steps planned alone, %d with CBS'
                  % (i, len(a) - 1, len(p) - 1))
        print('      (1 = robot 0, 2 = robot 1, X = both at different times)')
        draw(grid, paths, range(13, 22))
        min_d = min(np.hypot(*np.subtract(at(paths[0], t), at(paths[1], t)))
                    for t in range(max(map(len, paths))))
        check('closest approach %.2f m >= 1.0 m' % (min_d * grid.cell), min_d >= sep)

    # three robots, one of them crossing the others
    starts = [cell['P1'], cell['D1'], cell['P2']]
    goals = [cell['D1'], cell['P1'], cell['D2']]
    paths, how = resolve(grid, starts, goals, min_sep=sep)
    check('3 robots: conflict-free plan via %s' % how,
          paths is not None and first_conflict(paths, sep) is None)

    # head-on in the one-lane west corridor: CBS gives up, fallback still answers
    t0 = time.time()
    paths, how = resolve(grid, [cell['P1'], cell['P2']], [cell['P2'], cell['P1']], min_sep=sep)
    check('P1 <-> P2 in the one-lane corridor: answered via %s (%.1f s)'
          % (how, time.time() - t0), paths is not None and first_conflict(paths, sep) is None)
    return grid, sep


# ---- 5. executing the plan when Nav2 is faster or slower than planned --------------
def execute(paths, rules, rng, sep):
    """Random-speed execution. Each tick every robot advances one step with
    probability 0.3-0.9 (its own speed), but only if its wait-for rules allow.
    Returns the closest distance seen (cells) or None on deadlock."""
    speed = [rng.uniform(0.3, 0.9) for _ in paths]
    need = {}
    for b, k, a, j in rules:
        need.setdefault((b, k), []).append((a, j))
    prog = [0] * len(paths)
    closest = float('inf')
    for _ in range(5000):
        if all(prog[i] == len(p) - 1 for i, p in enumerate(paths)):
            return closest
        before = list(prog)
        for i, p in enumerate(paths):
            k = prog[i] + 1
            if k < len(p) and rng.random() < speed[i] and \
                    all(before[a] >= j for a, j in need.get((i, k), ())):
                prog[i] = k
        for a, b in itertools.combinations(range(len(paths)), 2):
            gap = np.subtract(paths[a][prog[a]], paths[b][prog[b]])
            closest = min(closest, float(np.hypot(*gap)))
    return None


def timing_tests(grid, sep):
    print('--- option 2 execution with random speed differences ---')
    cell = {k: grid.world_to_cell(*xy) for k, xy in LOCATIONS.items()}
    paths = cbs(grid, [cell['P1'], cell['D1']], [cell['D1'], cell['P1']], min_sep=sep, w=1.1)
    rules = wait_for_rules(paths, sep)
    print('      %d wait-for rules, e.g. %s' % (len(rules), rules[:2]))
    rng = random.Random(7)
    blind = [execute(paths, [], rng, sep) for _ in range(30)]
    safe = [execute(paths, rules, rng, sep) for _ in range(30)]
    hits = sum(d is not None and d < sep for d in blind)
    print('      without rules, %d of 30 runs got closer than 1.0 m' % hits)
    check('with wait-for rules: 30 random-speed runs, no deadlock',
          all(d is not None for d in safe))
    check('with wait-for rules: never closer than 1.0 m',
          all(d is not None and d >= sep for d in safe))


if __name__ == '__main__':
    low_level_tests()
    small_map_tests()
    random_tests()
    timing_tests(*warehouse_tests())
    print('\n%s' % ('ALL TESTS PASSED' if failed == 0 else '%d TEST(S) FAILED' % failed))
    sys.exit(1 if failed else 0)
