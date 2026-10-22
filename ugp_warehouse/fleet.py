"""Phase 6 execution layer: plan every robot's current leg together and pace the
robots by progress instead of by the clock (pure Python, no ROS, easy to test).

The task manager tells the fleet where each robot is going, the fleet answers
with Nav2 waypoint segments, and robots report back where they are:

    plan(goals, priority)   joint CBS plan for all robots from where they are now
    next_segment(name)      ('drive', waypoints) | ('wait', [robots])
                            | ('done', leg kind) | ('idle', None)
    on_pose(name, xy)       AMCL pose -> progress along the plan
    on_segment_done(name)   Nav2 reached the end of the segment

Pacing. CBS returns timed paths. wait_for_rules turns them into "robot b may not
enter cell k until robot a has reached cell j". A robot drives freely up to the
last cell it is allowed to enter and stops there. Planned waits are dropped,
because the rules already keep the passing order, at any Nav2 speed.

Progress is only claimed once a robot has passed a cell's centre (from AMCL), or
when Nav2 reports the segment done. Nav2 may stop up to `stop_tolerance` short,
so that slack is added to the robots' separation (see default_min_sep).

Goal clashes. Robots park on their goals, so two robots cannot head for the same
zone at once. The robot with the older task goes first and the other holds
('deferred') until the next plan. An idle or deferred robot standing too close to
someone's goal is first moved out of the way ('make_way').
"""
import math
from collections import deque

try:
    from ugp_warehouse.astar import simplify
    from ugp_warehouse.cbs import recommended_min_sep, resolve, wait_for_rules
except ImportError:           # running a test from inside the package folder
    from astar import simplify
    from cbs import recommended_min_sep, resolve, wait_for_rules

LEG, HOLD, MAKE_WAY, STAGE = 'leg', 'hold', 'make_way', 'stage'


def default_min_sep(cell_size=0.25, robot_radius=0.38, stop_tolerance=0.25):
    """Separation in cells for live runs: robot size + 0.1 m + Nav2's xy_goal_tolerance."""
    return recommended_min_sep(cell_size, robot_radius, 0.1 + stop_tolerance)


class Leg:
    """One robot's share of the current joint plan."""

    def __init__(self, cells, holds, target, kind):
        self.cells = cells          # distinct consecutive cells, cells[0] = where it starts
        self.holds = holds          # index -> [(robot, index it must have reached), ...]
        self.target = target        # exact (x, y) for the last waypoint, or None
        self.kind = kind            # LEG (task leg), HOLD (stay) or MAKE_WAY
        self.progress = 0           # last cell index the robot has certainly reached
        self.seg_end = None         # index the Nav2 segment in flight ends at
        self.finished = False


class Fleet:
    def __init__(self, grid, names, min_sep=None, w=1.1, max_nodes=1500):
        self.grid = grid
        self.names = list(names)
        self.sep = min_sep or default_min_sep(grid.cell)
        self.w = w
        self.max_nodes = max_nodes
        self.xy = {n: None for n in self.names}
        self.legs = {}
        self.timed = {}             # name -> timed cell path of the current plan (for RViz)

    # ---- inputs ------------------------------------------------------------------
    def on_pose(self, name, xy):
        self.xy[name] = tuple(xy)
        leg = self.legs.get(name)
        if leg is not None and leg.seg_end is not None:
            self._track(leg, xy)

    def on_segment_done(self, name):
        leg = self.legs[name]
        leg.progress = max(leg.progress, leg.seg_end)
        leg.seg_end = None

    def stop_all(self):
        """Robots were stopped (replanning): forget segments in flight."""
        for leg in self.legs.values():
            leg.seg_end = None

    # ---- planning ------------------------------------------------------------------
    def plan(self, goals, priority=()):
        """goals: {name: (x, y) or None}. priority: names, most urgent first.

        Returns {'planned': [...], 'deferred': [...], 'make_way': [...], 'method': str}.
        Always produces a plan: if the robots cannot all reach their goals, the
        least urgent ones are deferred (held) until a later plan.
        """
        starts = self._start_cells()
        order = [n for n in priority if goals.get(n) is not None]
        order += [n for n in self.names if goals.get(n) is not None and n not in order]
        goal_cell = {n: self.grid.nearest_free(self.grid.world_to_cell(*goals[n]))
                     for n in order}
        active, deferred = [], []
        for n in order:              # one robot per zone at a time, oldest task first
            if goal_cell[n] is None:
                deferred.append(n)
            elif any(self._close(goal_cell[n], goal_cell[m]) for m in active):
                deferred.append(n)
            else:
                active.append(n)
        while True:
            for staging in (True, False):          # queue near the zone, else hold still
                targets = {n: goal_cell[n] for n in active}
                others = [n for n in self.names if n not in active]
                for i, n in enumerate(others):
                    avoid = list(targets.values()) + [starts[m] for m in others[i + 1:]]
                    if staging and n in deferred and goal_cell[n] is not None:
                        targets[n] = self._near(goal_cell[n], starts[n], avoid)
                    else:
                        targets[n] = self._near(starts[n], starts[n], avoid)
                paths, method = resolve(self.grid, [starts[n] for n in self.names],
                                        [targets[n] for n in self.names], self.sep,
                                        self.w, self.max_nodes)
                if paths is not None:
                    break
            if paths is not None or not active:
                break
            deferred.append(active.pop())          # least urgent robot waits for now
        if paths is None:                          # should not happen: everyone stays put
            paths = [[starts[n]] for n in self.names]
            method = 'everyone holds (%s)' % method
        self._install(paths, goals, active, deferred, starts)
        kinds = {n: self.legs[n].kind for n in self.names}
        return {'planned': active, 'deferred': deferred, 'method': method,
                'staging': [n for n in deferred if kinds[n] == STAGE],
                'make_way': [n for n in self.names if kinds[n] == MAKE_WAY]}

    def _install(self, paths, goals, active, deferred, starts):
        index = {n: i for i, n in enumerate(self.names)}
        rules = wait_for_rules(paths, self.sep)
        compressed = {}
        for n, path in zip(self.names, paths):
            cells, step_to_idx = [path[0]], [0]
            for cell in path[1:]:
                if cell != cells[-1]:
                    cells.append(cell)
                step_to_idx.append(len(cells) - 1)
            compressed[n] = (cells, step_to_idx)
        holds = {n: {} for n in self.names}
        for b, k, a, j in rules:
            nb, na = self.names[b], self.names[a]
            at_b = compressed[nb][1][k - 1] + 1    # first move after b's step k-1
            need = compressed[na][1][min(j, len(paths[a]) - 1)]
            entry = holds[nb].setdefault(at_b, {})
            entry[na] = max(entry.get(na, 0), need)
        self.legs, self.timed = {}, {}
        for n in self.names:
            cells = compressed[n][0]
            if n in active:
                kind, target = LEG, goals[n]
            elif cells[-1] == starts[n]:
                kind, target = HOLD, None
            else:
                kind, target = (STAGE if n in deferred else MAKE_WAY), None
            hold = {i: sorted(d.items()) for i, d in holds[n].items() if i < len(cells)}
            self.legs[n] = Leg(cells, hold, target, kind)
            self.timed[n] = paths[index[n]]

    # ---- pacing ----------------------------------------------------------------------
    def next_segment(self, name):
        leg = self.legs.get(name)
        if leg is None or leg.seg_end is not None or leg.finished:
            return ('idle', None)
        last = len(leg.cells) - 1
        if leg.progress >= last:
            leg.finished = True
            return ('done', leg.kind)
        end = leg.progress
        for i in range(leg.progress + 1, last + 1):
            if self.blockers(name, i):
                break
            end = i
        if end == leg.progress:
            return ('wait', self.blockers(name, end + 1))
        leg.seg_end = end
        return ('drive', self._waypoints(leg, leg.progress, end))

    def blockers(self, name, index):
        """Robots that must move on before `name` may enter cell `index`."""
        return [a for a, need in self.legs[name].holds.get(index, ())
                if self.legs[a].progress < need]

    def progress_signature(self):
        """Changes whenever any robot gets further. Unchanged for long = stalled."""
        return tuple((n, self.legs[n].progress) for n in self.names if n in self.legs)

    def waiting_for(self, name):
        leg = self.legs.get(name)
        if leg is None or leg.seg_end is not None or leg.progress >= len(leg.cells) - 1:
            return []
        return self.blockers(name, leg.progress + 1)

    # ---- helpers ------------------------------------------------------------------
    def _close(self, a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 < self.sep ** 2

    def _start_cells(self):
        starts, used = {}, set()
        for n in self.names:
            cell = self.grid.nearest_free(self.grid.world_to_cell(*self.xy[n]))
            if cell is None or cell in used:
                cell = self._bfs(cell or self.grid.world_to_cell(*self.xy[n]),
                                 lambda c: c not in used)
            starts[n] = cell
            used.add(cell)
        return starts

    def _near(self, origin, start, avoid):
        """Free cell fewest steps from origin that keeps clear of every cell in avoid.
        Among those (give or take one step), the one closest to the robot's start."""
        found, best_depth = [], None
        for cell, depth in self._bfs_layers(origin):
            if best_depth is not None and depth > best_depth + 1:
                break
            if not any(self._close(cell, a) for a in avoid):
                best_depth = depth if best_depth is None else best_depth
                found.append(cell)
        if not found:
            return start
        return min(found, key=lambda c: (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2)

    def _bfs_layers(self, origin):
        """Free cells in order of steps from origin, as (cell, steps)."""
        if not self.grid.is_free(origin):
            origin = self.grid.nearest_free(origin)
            if origin is None:
                return
        seen, queue = {origin}, deque([(origin, 0)])
        while queue:
            cell, depth = queue.popleft()
            yield cell, depth
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nxt = (cell[0] + dr, cell[1] + dc)
                if nxt not in seen and self.grid.is_free(nxt):
                    seen.add(nxt)
                    queue.append((nxt, depth + 1))

    def _bfs(self, start, good):
        if start is None:
            return None
        seen, queue = {start}, deque([start])
        while queue:
            cell = queue.popleft()
            if self.grid.is_free(cell) and good(cell):
                return cell
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nxt = (cell[0] + dr, cell[1] + dc)
                if nxt not in seen and self.grid.in_bounds(nxt) and \
                        (self.grid.is_free(nxt) or not self.grid.is_free(cell)):
                    seen.add(nxt)
                    queue.append(nxt)
        return None

    def _track(self, leg, xy):
        """Advance progress to the furthest cell whose centre the robot has passed."""
        x, y = xy
        best = None
        seen = {leg.cells[leg.progress]}
        for i in range(leg.progress + 1, min(leg.seg_end, leg.progress + 6) + 1):
            if leg.cells[i] in seen:
                break                               # path doubles back: stop looking
            seen.add(leg.cells[i])
            cx, cy = self.grid.cell_to_world(leg.cells[i])
            px, py = self.grid.cell_to_world(leg.cells[i - 1])
            along = ((x - cx) * (cx - px) + (y - cy) * (cy - py)) / self.grid.cell
            if along >= -0.02 and math.hypot(x - cx, y - cy) <= 1.5 * self.grid.cell:
                best = i
        if best is not None:
            leg.progress = best

    def _waypoints(self, leg, first, last):
        cells = leg.cells[first:last + 1]
        corners = simplify(cells)[1:]
        points = [self.grid.cell_to_world(c) for c in corners]
        if last == len(leg.cells) - 1 and leg.target is not None:
            points[-1] = tuple(leg.target)
        if last < len(leg.cells) - 1:              # face the next move while waiting
            ahead = self.grid.cell_to_world(leg.cells[last + 1])
        else:
            ahead = None
        out = []
        for i, (x, y) in enumerate(points):
            if i + 1 < len(points):
                nx, ny = points[i + 1]
            elif ahead is not None:
                nx, ny = ahead
            else:                                  # end of leg: keep the last heading
                bx, by = points[i - 1] if i else self.grid.cell_to_world(cells[0])
                nx, ny = 2 * x - bx, 2 * y - by
            out.append((x, y, math.atan2(ny - y, nx - x)))
        return out
