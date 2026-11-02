"""Space-time A* and prioritized multi-robot route coordination."""
import heapq

try:
    from ugp_warehouse.astar import MOVES, manhattan, reconstruct
except ImportError:
    from astar import MOVES, manhattan, reconstruct


class ReservationTable:
    """Vertex and directed-edge reservations indexed by discrete time step."""

    def __init__(self):
        self.vertices = {}
        self.edges = {}
        self.goal_holds = {}
        self.start_holds = {}

    def reserve_start(self, cell, start_time, hold_until):
        self.start_holds[cell] = (start_time, hold_until)

    def release_start(self, cell):
        self.start_holds.pop(cell, None)

    def vertex_conflict(self, cell, time):
        hold = self.start_holds.get(cell)
        return (cell in self.vertices.get(time, ()) or
                bool(hold and hold[0] <= time <= hold[1]))

    def edge_conflict(self, source, target, time):
        return (target, source) in self.edges.get(time, ())

    def reserve_path(self, path, start_time=0, hold_until=None):
        if not path:
            return
        for offset, cell in enumerate(path):
            time = start_time + offset
            self.vertices.setdefault(time, set()).add(cell)
            if offset:
                previous = path[offset - 1]
                self.edges.setdefault(time, set()).add((previous, cell))
        if hold_until is not None:
            self.goal_holds[path[-1]] = (
                start_time + len(path) - 1, hold_until)
            for time in range(start_time + len(path), hold_until + 1):
                self.vertices.setdefault(time, set()).add(path[-1])


def space_time_astar(grid, start, goal, reservations=None, start_time=0,
                     max_time=None):
    """Plan a shortest route with wait actions and vertex/swap avoidance.

    The returned list contains one cell per time step, including the start.
    Repeated cells represent waiting. Returns None if no route exists within
    the search horizon.
    """
    if not grid.is_free(start) or not grid.is_free(goal):
        return None
    reservations = reservations or ReservationTable()
    if max_time is None:
        max_time = start_time + max(
            20, manhattan(start, goal) + grid.rows + grid.cols)
    if (start_time > max_time or
            reservations.vertex_conflict(start, start_time)):
        return None

    initial = (start, start_time)
    counter = 0
    queue = [(manhattan(start, goal), counter, initial)]
    costs = {initial: 0}
    parents = {initial: None}
    closed = set()

    while queue:
        _, _, state = heapq.heappop(queue)
        if state in closed:
            continue
        cell, time = state
        held = reservations.goal_holds.get(goal)
        if cell == goal and not (held and held[0] <= max_time):
            if not any(
                reservations.vertex_conflict(goal, future_time)
                    for future_time in range(time, max_time + 1)):
                path = reconstruct(parents, state)
                return [position for position, _ in path]
        closed.add(state)
        if time >= max_time:
            continue

        next_time = time + 1
        for dr, dc in ((0, 0),) + MOVES:
            nxt = (cell[0] + dr, cell[1] + dc)
            if nxt != cell and not grid.is_free(nxt):
                continue
            if reservations.vertex_conflict(nxt, next_time):
                continue
            if reservations.edge_conflict(cell, nxt, next_time):
                continue
            next_state = (nxt, next_time)
            new_cost = costs[state] + 1
            if new_cost >= costs.get(next_state, float('inf')):
                continue
            costs[next_state] = new_cost
            parents[next_state] = state
            counter += 1
            priority = new_cost + manhattan(nxt, goal)
            heapq.heappush(queue, (priority, counter, next_state))
    return None


class MultiRobotCoordinator:
    """Prioritized planner that reserves each accepted route for later robots."""

    def __init__(self, grid, max_time=None):
        self.grid = grid
        self.max_time = max_time

    def plan(self, agents, priority=None, start_time=0):
        """Return ``(paths, unplanned)`` for name -> (start_cell, goal_cell).

        Paths are planned in priority order. The destination remains reserved
        through the planning horizon, preventing two robots sharing a goal.
        """
        order = list(priority) if priority is not None else list(agents)
        order.extend(name for name in agents if name not in order)
        horizon = self.max_time
        if horizon is None:
            distance = max(
                (manhattan(start, goal) for start, goal in agents.values()),
                default=0)
            horizon = start_time + max(
                20, distance + self.grid.rows + self.grid.cols)
        reservations = ReservationTable()
        starts = [start for start, _ in agents.values()]
        duplicate_starts = {cell for cell in starts if starts.count(cell) > 1}
        for start in starts:
            reservations.reserve_start(start, start_time, horizon)

        paths = {}
        unplanned = [
            name for name, (start, _) in agents.items()
            if start in duplicate_starts]
        for name in order:
            if name in unplanned:
                continue
            start, goal = agents[name]
            reservations.release_start(start)
            path = space_time_astar(
                self.grid, start, goal, reservations, start_time, horizon)
            if path is None:
                unplanned.append(name)
                reservations.reserve_start(start, start_time, horizon)
                continue
            paths[name] = path
            reservations.reserve_path(path, start_time, horizon)
        return paths, unplanned
