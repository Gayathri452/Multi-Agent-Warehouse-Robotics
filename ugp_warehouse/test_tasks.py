"""Tests for task assignment (no ROS).  Run:  python3 test_tasks.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_map import GridMap                  # noqa: E402
from locations import LOCATIONS               # noqa: E402
from tasks import Task, assign                # noqa: E402

MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'warehouse_map.yaml')
failed = 0


def check(name, ok):
    global failed
    print(('PASS  ' if ok else 'FAIL  ') + name)
    failed += 0 if ok else 1


grid = GridMap.from_yaml(MAP_YAML)
R1, R2 = (-8.0, 0.0), (-8.3, -1.0)


def robots(c1=0, c2=0):
    return {'robot_1': {'xy': R1, 'completed': c1}, 'robot_2': {'xy': R2, 'completed': c2}}


# 1. two tasks, two idle robots: each robot gets the task nearest to it
tasks = [Task(1, 'P1', 'D1'), Task(2, 'P2', 'D2')]
got, bad = assign(grid, tasks, robots(), LOCATIONS)
who = {t.id: r for t, r, _ in got}
for t, r, c in got:
    print('      task %d (%s->%s) -> %s, cost %.1f m' % (t.id, t.pickup, t.delivery, r, c))
check('P1->D1 goes to robot_1, P2->D2 goes to robot_2', who == {1: 'robot_1', 2: 'robot_2'})
check('nothing infeasible', bad == [])

# 2. one task near robot_2: robot_2 wins
got, _ = assign(grid, [Task(3, 'P2', 'D2')], robots(), LOCATIONS)
check('single task P2->D2 goes to the nearer robot_2', got[0][1] == 'robot_2')

# 3. load balancing: robot_2 already did 10 tasks, so robot_1 gets it despite being farther
got, _ = assign(grid, [Task(4, 'P2', 'D2')], robots(c2=10), LOCATIONS)
check('load penalty moves the task to robot_1', got[0][1] == 'robot_1')

# 4. only one idle robot, two tasks: one assignment, the oldest task first
got, _ = assign(grid, [Task(5, 'P1', 'D1'), Task(6, 'P2', 'D2')],
                {'robot_2': {'xy': R2, 'completed': 0}}, LOCATIONS)
check('one idle robot -> one assignment, oldest task (5)', len(got) == 1 and got[0][0].id == 5)

# 5. unreachable pickup (inside shelf) is reported infeasible
places = dict(LOCATIONS, X=(-4.5, 4.2))
got, bad = assign(grid, [Task(7, 'X', 'D1')], robots(), places)
check('pickup inside a shelf is infeasible', got == [] and [t.id for t in bad] == [7])

print('\n%s' % ('ALL TESTS PASSED' if failed == 0 else '%d TEST(S) FAILED' % failed))
sys.exit(1 if failed else 0)
