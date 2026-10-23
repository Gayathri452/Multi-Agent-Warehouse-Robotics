"""Test task assignment without ROS. Run: python3 test_tasks.py."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_map import GridMap                  # noqa: E402
from locations import LOCATIONS               # noqa: E402
from tasks import assign, Task                # noqa: E402

MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'warehouse_map.yaml')


R1, R2 = (-8.0, 0.0), (-8.3, -1.0)


def robots(c1=0, c2=0):
    return {'robot_1': {'xy': R1, 'completed': c1},
            'robot_2': {'xy': R2, 'completed': c2}}


def test_task_assignment_cases():
    grid = GridMap.from_yaml(MAP_YAML)

    tasks = [Task(1, 'P1', 'D1'), Task(2, 'P2', 'D2')]
    got, bad = assign(grid, tasks, robots(), LOCATIONS)
    who = {task.id: robot for task, robot, _ in got}
    assert who == {1: 'robot_1', 2: 'robot_2'}
    assert bad == []

    got, _ = assign(grid, [Task(3, 'P2', 'D2')], robots(), LOCATIONS)
    assert got[0][1] == 'robot_2'

    got, _ = assign(grid, [Task(4, 'P2', 'D2')], robots(c2=10), LOCATIONS)
    assert got[0][1] == 'robot_1'

    got, _ = assign(
        grid, [Task(5, 'P1', 'D1'), Task(6, 'P2', 'D2')],
        {'robot_2': {'xy': R2, 'completed': 0}}, LOCATIONS)
    assert len(got) == 1
    assert got[0][0].id == 5

    places = dict(LOCATIONS, X=(-4.5, 4.2))
    got, bad = assign(grid, [Task(7, 'X', 'D1')], robots(), places)
    assert got == []
    assert [task.id for task in bad] == [7]


if __name__ == '__main__':
    test_task_assignment_cases()
    print('ALL TESTS PASSED')
