"""Tasks and cost-based task assignment (pure Python, no ROS, so it is easy to test).

Task assignment answers "WHO should do this task?"  (A* then answers "HOW does
that robot get there?"). Cost of giving task T to robot R:

    cost = A*(robot -> pickup)  +  A*(pickup -> delivery)  +  LOAD_PENALTY * tasks R already finished

The load term stops one robot from being given everything when two are about
equally close. Lengths are A* path lengths on the inflated grid, in metres.
"""
from dataclasses import dataclass

try:
    from ugp_warehouse.astar import astar
except ImportError:           # running a test from inside the package folder
    from astar import astar

PENDING, TO_PICKUP, TO_DELIVERY, DONE, FAILED = (
    'PENDING', 'TO_PICKUP', 'TO_DELIVERY', 'DONE', 'FAILED')
LOAD_PENALTY = 2.0            # metres of "extra distance" per task already completed


@dataclass
class Task:
    id: int
    pickup: str               # a key of LOCATIONS, e.g. 'P1'
    delivery: str             # e.g. 'D2'
    state: str = PENDING
    robot: str = ''


def path_length(grid, a_xy, b_xy):
    """A* length in metres between two world points, or None if there is no path."""
    start = grid.nearest_free(grid.world_to_cell(*a_xy))
    goal = grid.world_to_cell(*b_xy)
    if start is None:
        return None
    path = astar(grid, start, goal)
    return None if path is None else (len(path) - 1) * grid.cell


def assignment_cost(grid, robot_xy, pickup_xy, delivery_xy, completed):
    to_pickup = path_length(grid, robot_xy, pickup_xy)
    to_delivery = path_length(grid, pickup_xy, delivery_xy)
    if to_pickup is None or to_delivery is None:
        return None
    return to_pickup + to_delivery + LOAD_PENALTY * completed


def assign(grid, tasks, idle_robots, locations):
    """Greedy assignment, oldest task first, cheapest idle robot for each.

    idle_robots: {name: {'xy': (x, y), 'completed': n}}  (only robots with no task)
    Returns (assigned, infeasible):
      assigned   = [(task, robot_name, cost), ...]   at most one task per robot
      infeasible = [task, ...]   no robot has a path (e.g. pickup inside a shelf)
    """
    free = dict(idle_robots)
    assigned, infeasible = [], []
    for task in tasks:
        if task.state != PENDING:
            continue
        if not free:
            break
        best = None
        for name, r in free.items():
            cost = assignment_cost(grid, r['xy'], locations[task.pickup],
                                   locations[task.delivery], r['completed'])
            if cost is not None and (best is None or cost < best[1]):
                best = (name, cost)
        if best is None:
            infeasible.append(task)
        else:
            assigned.append((task, best[0], best[1]))
            del free[best[0]]
    return assigned, infeasible
