"""Tests for space-time A* and prioritized multi-robot coordination."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_map import GridMap  # noqa: E402
from space_time import MultiRobotCoordinator, ReservationTable, space_time_astar  # noqa: E402


def make_grid(rows):
    free = np.array([[ch != '#' for ch in row] for row in rows])
    return GridMap(free, 1.0, 0.0, 0.0)


def assert_conflict_free(paths):
    scheduled = {}
    edges = {}
    for name, path in paths.items():
        for time, cell in enumerate(path):
            key = (time, cell)
            assert key not in scheduled, '%s conflicts with %s at %s' % (
                name, scheduled.get(key), key)
            scheduled[key] = name
            if time:
                edge = (time, path[time - 1], cell)
                reverse = (time, cell, path[time - 1])
                assert reverse not in edges, '%s swaps with %s at time %d' % (
                    name, edges.get(reverse), time)
                edges[edge] = name


def test_wait_action_is_returned_for_crossing_paths():
    grid = make_grid(['...', '...', '...'])
    coordinator = MultiRobotCoordinator(grid)
    paths, unplanned = coordinator.plan({
        'horizontal': ((1, 0), (1, 2)),
        'vertical': ((0, 1), (2, 1)),
    })
    assert not unplanned
    assert_conflict_free(paths)
    assert any(a == b for a, b in zip(paths['vertical'], paths['vertical'][1:]))


def test_shared_goal_is_not_assigned_to_both_robots():
    grid = make_grid(['...', '...', '...'])
    paths, unplanned = MultiRobotCoordinator(grid).plan({
        'first': ((1, 0), (1, 2)),
        'second': ((0, 2), (1, 2)),
    })
    assert paths['first'][-1] == (1, 2)
    assert unplanned == ['second']


def test_narrow_corridor_swap_is_reported_unplannable():
    grid = make_grid(['.....'])
    paths, unplanned = MultiRobotCoordinator(grid).plan({
        'left_to_right': ((0, 0), (0, 4)),
        'right_to_left': ((0, 4), (0, 0)),
    })
    assert paths == {}
    assert unplanned == ['left_to_right', 'right_to_left']


def test_reverse_edge_conflict_is_rejected():
    grid = make_grid(['..'])
    reservations = ReservationTable()
    reservations.reserve_path([(0, 1), (0, 0)])
    assert space_time_astar(
        grid, (0, 0), (0, 1), reservations, max_time=4) is None


if __name__ == '__main__':
    test_wait_action_is_returned_for_crossing_paths()
    test_shared_goal_is_not_assigned_to_both_robots()
    test_narrow_corridor_swap_is_reported_unplannable()
    test_reverse_edge_conflict_is_rejected()
    print('ALL SPACE-TIME TESTS PASSED')
