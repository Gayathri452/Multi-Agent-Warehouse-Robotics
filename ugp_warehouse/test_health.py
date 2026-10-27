"""Tests for Phase 7: failure detection and recovery (no ROS).  Run:  python3 test_health.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cbs import at                                         # noqa: E402
from fleet import HOLD, LEG, Fleet, default_min_sep        # noqa: E402
from grid_map import GridMap                               # noqa: E402
from health import DOWN, Supervisor                        # noqa: E402
from locations import LOCATIONS                            # noqa: E402

MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'warehouse_map.yaml')
failed = 0


def check(name, ok):
    global failed
    print(('PASS  ' if ok else 'FAIL  ') + name)
    failed += 0 if ok else 1


def run(sup, start, end, step=0.2, odom=('a', 'b'), nav=('a', 'b')):
    """Advance time; robots in `odom` send odometry, robots in `nav` have Nav2 up."""
    events, t = [], start
    while t < end - 1e-9:
        t = round(t + step, 6)
        for n in sup.robots:
            if n in odom:
                sup.odom(n, t)
            sup.nav(n, n in nav, t)
        events += sup.check(t)
    return events, t


def heartbeat_tests():
    print('--- heartbeats and timeouts ---')
    sup = Supervisor(['a', 'b'], heartbeat_timeout=3.0, nav_timeout=3.0, revive_after=10.0)
    events, t = run(sup, 0.0, 1.0)
    check('both robots ready once odometry and Nav2 are seen',
          ('ready', 'a') in events and ('ready', 'b') in events and sup.is_up('a'))

    events, t = run(sup, t, t + 2.0, odom=('a',))
    check('2 s without odometry: still in service', sup.is_up('b') and not events)
    events, t = run(sup, t, t + 2.0, odom=('a',))
    check('over 3 s without odometry: OUT OF SERVICE (heartbeat lost)',
          any(e[0] == 'down' and e[1] == 'b' and 'heartbeat' in e[2] for e in events)
          and sup.robots['b'].state == DOWN and sup.down() == {'b'})

    events, t = run(sup, t, t + 2.0, odom=('a',), nav=('b',))
    check('Nav2 gone 2 s: still in service (short blips are tolerated)', sup.is_up('a'))
    events, t = run(sup, t, t + 2.0, odom=('a',), nav=('b',))
    check('Nav2 gone over 3 s: OUT OF SERVICE (navigation not responding)',
          any(e[0] == 'down' and e[1] == 'a' and 'navigation' in e[2] for e in events))

    events, t = run(sup, t, t + 9.0)
    check('healthy again for 9 s: not yet back', not sup.is_up('a') and not sup.is_up('b'))
    events, t = run(sup, t, t + 2.0)
    check('healthy for 10 s: BACK IN SERVICE',
          ('back', 'a') in events and ('back', 'b') in events and sup.down() == set())

    sup = Supervisor(['a'])
    events, t = run(sup, 0.0, 16.0, odom=())
    check('no odometry ever: falls back to the Nav2 heartbeat',
          ('nav_only', 'a') in events and sup.is_up('a'))


def stuck_tests():
    print('--- stuck detection ---')
    sup = Supervisor(['a'], stuck_timeout=25.0)
    _, t = run(sup, 0.0, 1.0, odom=('a',), nav=('a',))
    events = []
    for i in range(130):                       # driving, progress frozen for 26 s
        t = round(t + 0.2, 6)
        sup.odom('a', t)
        sup.nav('a', True, t)
        sup.progress('a', (1, 7), True, t)
        events += sup.check(t)
    check('driving without progress for 25 s: stuck (reported once)',
          [e[0] for e in events].count('stuck') == 1)
    sup = Supervisor(['a'], stuck_timeout=25.0)
    _, t = run(sup, 0.0, 1.0, odom=('a',), nav=('a',))
    events = []
    for i in range(200):                       # waiting at a hold point: not driving
        t = round(t + 0.2, 6)
        sup.odom('a', t)
        sup.nav('a', True, t)
        sup.progress('a', (1, 7), False, t)
        events += sup.check(t)
    check('waiting for another robot is not being stuck', not events)


def bookkeeping_tests():
    print('--- blocked areas and task attempts ---')
    sup = Supervisor(['a'], block_seconds=120.0, max_attempts=3)
    sup.block({(1, 1), (1, 2)}, 10.0)
    check('a blocked area is avoided...', sup.blocked(100.0) == {(1, 1), (1, 2)})
    check('...and forgotten after block_seconds', sup.blocked(131.0) == set())
    check('task given back once: another robot may try (attempt 2 of 3)',
          sup.give_back(7) and sup.attempt(7) == 2)
    check('given back twice: attempt 3 of 3', sup.give_back(7) and sup.attempt(7) == 3)
    check('given back a third time: out of attempts -> FAILED', not sup.give_back(7))


def planning_tests():
    print('--- planning around failures (warehouse map) ---')
    grid = GridMap.from_yaml(MAP_YAML)
    sep = default_min_sep(grid.cell)
    p1, d1 = LOCATIONS['P1'], LOCATIONS['D1']

    # robot_2 broke down in the north aisle, on robot_1's way from P1 to D1
    fleet = Fleet(grid, ['robot_1', 'robot_2'], min_sep=sep)
    fleet.on_pose('robot_1', p1)
    fleet.on_pose('robot_2', (0.0, 2.7))
    result = fleet.plan({'robot_1': d1, 'robot_2': None}, down={'robot_2'})
    path = fleet.timed['robot_1']
    dead = grid.world_to_cell(0.0, 2.7)
    closest = min(math.dist(at(path, t), dead) for t in range(len(path)))
    check('robot out of service is not planned (stays where it stopped)',
          result['down'] == ['robot_2'] and fleet.legs['robot_2'].kind == HOLD and
          len(fleet.legs['robot_2'].cells) == 1)
    check('the other robot keeps %.2f m from it (closest %.2f m)'
          % (sep * grid.cell, closest * grid.cell), closest >= sep)
    check('and still reaches D1', fleet.legs['robot_1'].kind == LEG and
          path[-1] == grid.world_to_cell(*d1))

    # a box closes the north aisle between P1 and the centre
    fleet = Fleet(grid, ['robot_1', 'robot_2'], min_sep=sep)
    fleet.on_pose('robot_1', p1)
    fleet.on_pose('robot_2', LOCATIONS['P2'])
    fleet.plan({'robot_1': d1, 'robot_2': None})
    open_route = len(fleet.timed['robot_1'])
    fleet.on_pose('robot_1', p1)
    fleet.legs['robot_1'].progress = 8            # it got this far, then was stopped
    blocked = fleet.area_ahead('robot_1')
    check('area ahead of a stopped robot is a disc on its route',
          len(blocked) > 20 and fleet.legs['robot_1'].cells[11] in blocked)
    fleet.plan({'robot_1': d1, 'robot_2': None}, obstacles=blocked)
    detour = fleet.timed['robot_1']
    check('replanned route avoids the blocked area',
          not any(c in blocked for c in detour))
    check('and is a real detour through another aisle (%d -> %d steps)'
          % (open_route - 1, len(detour) - 1), len(detour) > open_route + 10)


if __name__ == '__main__':
    heartbeat_tests()
    stuck_tests()
    bookkeeping_tests()
    planning_tests()
    print('\n%s' % ('ALL TESTS PASSED' if failed == 0 else '%d TEST(S) FAILED' % failed))
    sys.exit(1 if failed else 0)
