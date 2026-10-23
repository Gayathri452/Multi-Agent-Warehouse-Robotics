"""Tests for space-time A* and prioritized multi-robot coordination."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from ugp_warehouse.grid_map import GridMap
    from ugp_warehouse.space_time import (
        CBSCoordinator, find_first_conflict, MultiRobotCoordinator,
        ReservationTable, space_time_astar)
except ImportError:
    from grid_map import GridMap  # noqa: E402
    from space_time import (CBSCoordinator, find_first_conflict,
                            MultiRobotCoordinator, ReservationTable,
                            space_time_astar)  # noqa: E402


def make_grid(rows):
    free = np.array([[ch != '#' for ch in row] for row in rows])
    return GridMap(free, 1.0, 0.0, 0.0)


def assert_conflict_free(paths):
    scheduled = {}
    edges = {}
    horizon = max((len(path) for path in paths.values()), default=0)
    for name, path in paths.items():
        for time in range(horizon):
            cell = path[min(time, len(path) - 1)]
            key = (time, cell)
            assert key not in scheduled, '%s conflicts with %s at %s' % (
                name, scheduled.get(key), key)
            scheduled[key] = name
            if time:
                previous = path[min(time - 1, len(path) - 1)]
                edge = (time, previous, cell)
                reverse = (time, cell, previous)
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


def test_conflict_detector_finds_vertex_and_swap_conflicts():
    assert find_first_conflict({
        'a': [(0, 0), (0, 1)],
        'b': [(1, 0), (0, 1)],
    }).kind == 'vertex'
    conflict = find_first_conflict({
        'a': [(0, 0), (0, 1)],
        'b': [(0, 1), (0, 0)],
    })
    assert conflict.kind == 'edge'
    assert conflict.time == 1


def test_cbs_resolves_crossing_paths_without_conflicts():
    grid = make_grid(['...', '...', '...'])
    paths, unplanned = CBSCoordinator(grid).plan({
        'horizontal': ((1, 0), (1, 2)),
        'vertical': ((0, 1), (2, 1)),
    })
    assert not unplanned
    assert_conflict_free(paths)
    assert find_first_conflict(paths) is None


def test_cbs_rejects_shared_goal_and_narrow_corridor_deadlock():
    grid = make_grid(['...', '...', '...'])
    paths, unplanned = CBSCoordinator(grid, max_nodes=100).plan({
        'first': ((1, 0), (1, 2)),
        'second': ((0, 2), (1, 2)),
    })
    assert not paths
    assert set(unplanned) == {'first', 'second'}

    corridor = make_grid(['.....'])
    paths, unplanned = CBSCoordinator(corridor, max_nodes=100).plan({
        'left_to_right': ((0, 0), (0, 4)),
        'right_to_left': ((0, 4), (0, 0)),
    })
    assert not paths
    assert set(unplanned) == {'left_to_right', 'right_to_left'}


if __name__ == '__main__':
    test_wait_action_is_returned_for_crossing_paths()
    test_shared_goal_is_not_assigned_to_both_robots()
    test_narrow_corridor_swap_is_reported_unplannable()
    test_reverse_edge_conflict_is_rejected()
    test_conflict_detector_finds_vertex_and_swap_conflicts()
    test_cbs_resolves_crossing_paths_without_conflicts()
    test_cbs_rejects_shared_goal_and_narrow_corridor_deadlock()
    print('ALL SPACE-TIME AND CBS TESTS PASSED')
