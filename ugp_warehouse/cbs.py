"""Phase 6: conflict resolution for several robots (pure Python, no ROS).

Phase 5 gives each robot a space-time path and detects where two paths clash.
Phase 6 removes the clashes, in two ways:

  * Prioritized planning: plan robots one at a time in a fixed order. Each robot
    treats the robots planned before it as moving obstacles. Fast, but the order
    matters and it can fail even when a plan exists.
  * CBS (conflict-based search): plan every robot on its own, find the first
    conflict, and split into two branches ("robot A may not be there then" /
    "robot B may not be there then"). Always expand the cheapest branch. Returns
    the plan with the lowest total time (sum of costs) when one exists. With
    w > 1 it trades a bounded amount of optimality for a lot of speed.

Time is discrete. One step = move one cell (up/down/left/right) or wait.
A path is a list of cells with path[t] = the robot's cell at step t. After its
last cell the robot stays there (parked on its goal).

Robot size: planning cells are 0.25 m but the robots are 0.76 m wide, so two
robots in neighbouring cells would touch. `min_sep` is the smallest allowed
distance between two robot centres, in cells:
    min_sep = 1 -> classic MAPF: same-cell (vertex) and swap (edge) conflicts only
    min_sep = 4 -> about 1 m, safe for these robots (see recommended_min_sep)

Constraints understood by the low-level planner (plan_one):
    ('v', t, cells)     may not be on any of `cells` at step t
    ('e', t, u, v)      may not move u -> v between steps t and t+1 (u == v: may not wait)
    ('from', t, cells)  may not be on any of `cells` at any step >= t (a parked robot)
"""
import heapq
import itertools
import math
from collections import deque

MOVES = ((-1, 0), (1, 0), (0, -1), (0, 1), (0, 0))     # 4 moves, then wait


# ---- small helpers -----------------------------------------------------------
def recommended_min_sep(cell_size=0.25, robot_radius=0.38, margin=0.1):
    """Safe centre-to-centre distance between two robots, in cells.

    Distances are only checked at whole steps. Between two steps two robots that
    are both >= s cells apart can get as close as sqrt(s*s - 1) cells, so s is
    chosen to keep even that in-between distance above 2 * radius + margin.
    For 0.38 m robots on 0.25 m cells this gives 4 (worst case 0.97 m apart).
    """
    need = (2 * robot_radius + margin) / cell_size
    return max(1, math.ceil(math.sqrt(need * need + 1)))


def at(path, t):
    """Cell of a robot at step t (it stays on its last cell after arriving)."""
    return path[t] if t < len(path) else path[-1]


def sum_of_costs(paths):
    """Total arrival time of all robots (the number CBS minimises)."""
    return sum(len(p) - 1 for p in paths)


def _d2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _need2(start_a, start_b, min_sep):
    """Squared separation two robots must keep.

    If they START closer than min_sep (both parked near the same zone), they only
    have to never get closer than they started; otherwise there is no solution.
    """
    return min(min_sep * min_sep, _d2(start_a, start_b))


def _disc(center, need2):
    """All cells strictly closer than sqrt(need2) to center."""
    r = int(math.isqrt(max(need2 - 1, 0)))
    return frozenset((center[0] + dr, center[1] + dc)
                     for dr in range(-r, r + 1) for dc in range(-r, r + 1)
                     if dr * dr + dc * dc < need2)


def distance_map(grid, goal):
    """BFS steps from every free cell to goal (walls only). Used as the A* heuristic."""
    if not grid.is_free(goal):
        return {}
    dist = {goal: 0}
    queue = deque([goal])
    while queue:
        cell = queue.popleft()
        for dr, dc in MOVES[:4]:
            nxt = (cell[0] + dr, cell[1] + dc)
            if nxt not in dist and grid.is_free(nxt):
                dist[nxt] = dist[cell] + 1
                queue.append(nxt)
    return dist


# ---- conflict detection --------------------------------------------------------
def conflicts(paths, min_sep=1):
    """Yield every conflict, earliest first.

    ('vertex', a, b, t, cell_a, cell_b)          robots a and b too close at step t
    ('edge',   a, b, t, (u_a, v_a), (u_b, v_b))  a and b swap cells between t and t+1

    Edge (swap) conflicts only exist for min_sep 1. With min_sep >= 2 a swap is
    already a vertex conflict, and recommended_min_sep covers the in-between motion.
    """
    n = len(paths)
    pairs = list(itertools.combinations(range(n), 2))
    need = {(a, b): _need2(paths[a][0], paths[b][0], min_sep) for a, b in pairs}
    horizon = max(len(p) for p in paths)
    for t in range(horizon):
        for a, b in pairs:
            ca, cb = at(paths[a], t), at(paths[b], t)
            if _d2(ca, cb) < need[a, b]:
                yield ('vertex', a, b, t, ca, cb)
        if min_sep > 1 or t + 1 >= horizon:
            continue
        for a, b in pairs:
            ua, va = at(paths[a], t), at(paths[a], t + 1)
            ub, vb = at(paths[b], t), at(paths[b], t + 1)
            if ua == vb and ub == va and ua != va:
                yield ('edge', a, b, t, (ua, va), (ub, vb))


def first_conflict(paths, min_sep=1):
    """The earliest conflict, or None if the plan is conflict-free."""
    return next(conflicts(paths, min_sep), None)


# ---- low level: space-time A* with constraints ---------------------------------
def plan_one(grid, start, goal, constraints=(), dist=None, others=(), min_sep=1):
    """Shortest timed path start -> goal that respects the constraints.

    Returns [cell at t=0, cell at t=1, ..., goal] or None if impossible.
    The robot is assumed to stay on the goal forever after the last step.

    others: paths of the other robots. They are NOT obstacles; among equally
    short paths the one that comes too close to them least often is preferred
    (a 'conflict avoidance table'). This keeps CBS from branching on clashes
    that a same-length detour avoids anyway.
    """
    if dist is None:
        dist = distance_map(grid, goal)
    if start not in dist:
        return None
    vertex, edges, blocked_from, last = {}, set(), {}, 0
    for con in constraints:
        if con[0] == 'v':
            vertex.setdefault(con[1], set()).update(con[2])
            last = max(last, con[1])
        elif con[0] == 'e':
            edges.add((con[1], con[2], con[3]))
            last = max(last, con[1] + 1)
        else:
            for cell in con[2]:
                blocked_from[cell] = min(con[1], blocked_from.get(cell, con[1]))
            last = max(last, con[1])
    if goal in blocked_from:
        return None                      # someone parks on our goal forever
    park = 0                             # earliest step we may stop on the goal for good
    for t, cells in vertex.items():
        if goal in cells:
            park = max(park, t + 1)
    for t, u, v in edges:
        if u == v == goal:
            park = max(park, t + 1)
    others = [(p, _need2(p[0], start, min_sep)) for p in others]
    # from step tcap on, nothing depends on time any more (constraints and other
    # robots are all settled), so (cell, t) and (cell, tcap) are the same state
    tcap = max([last + 1] + [len(p) for p, _ in others])

    def allowed(cell, t):
        if cell in vertex.get(t, ()):
            return False
        b = blocked_from.get(cell)
        return b is None or t < b

    def clashes(cell, t):
        return sum(_d2(at(p, t), cell) < n2 for p, n2 in others)

    if not allowed(start, 0):
        return None
    h = max(dist[start], park)
    open_heap = [(h, 0, h, 0, start)]    # (f, clashes so far, h, t, cell)
    parent = {(start, 0): None}
    closed = set()
    while open_heap:
        _, k, _, t, cell = heapq.heappop(open_heap)
        key = (cell, min(t, tcap))
        if key in closed:
            continue
        closed.add(key)
        if cell == goal and t >= park:
            path, node = [], (cell, t)
            while node is not None:
                path.append(node[0])
                node = parent[node]
            return path[::-1]
        nt = t + 1
        for dr, dc in MOVES:
            nxt = (cell[0] + dr, cell[1] + dc)
            if nxt not in dist or (nxt, min(nt, tcap)) in closed:
                continue
            if not allowed(nxt, nt) or (t, cell, nxt) in edges:
                continue
            if (nxt, nt) not in parent:
                parent[(nxt, nt)] = (cell, t)
                h = max(dist[nxt], park - nt)
                nk = k + clashes(nxt, nt) if others else 0
                heapq.heappush(open_heap, (nt + h, nk, h, nt, nxt))
    return None


def _check_inputs(grid, starts, goals, min_sep):
    """None if the problem is worth searching, else a short reason."""
    if len(set(starts)) != len(starts):
        return 'two robots start on the same cell'
    for i, (s, g) in enumerate(zip(starts, goals)):
        if not grid.is_free(s) or not grid.is_free(g):
            return 'robot %d: start or goal is not a free cell' % i
    for a, b in itertools.combinations(range(len(starts)), 2):
        if _d2(goals[a], goals[b]) < _need2(starts[a], starts[b], min_sep):
            return 'robots %d and %d have (nearly) the same goal' % (a, b)
    return None


# ---- prioritized planning ------------------------------------------------------
def _avoid(other, my_start, min_sep):
    """Constraints that keep a robot clear of another robot's fixed timed path."""
    need2 = _need2(other[0], my_start, min_sep)
    end = len(other) - 1
    cons = [('v', t, _disc(other[t], need2)) for t in range(1, end)]
    cons.append(('from', end, _disc(other[end], need2)))     # parked on its goal
    if min_sep <= 1:                                          # classic: also no swapping
        cons += [('e', t, other[t + 1], other[t]) for t in range(end)
                 if other[t] != other[t + 1]]
    return cons


def prioritized(grid, starts, goals, order=None, min_sep=1):
    """Plan robots one by one; earlier robots in `order` have right of way.

    Returns the list of paths (same indexing as starts) or None if a robot
    cannot be planned around the ones before it.
    """
    if _check_inputs(grid, starts, goals, min_sep):
        return None
    order = list(range(len(starts))) if order is None else list(order)
    paths = [None] * len(starts)
    for i in order:
        cons = []
        for j, other in enumerate(paths):
            if other is not None:
                cons += _avoid(other, starts[i], min_sep)
        paths[i] = plan_one(grid, starts[i], goals[i], cons)
        if paths[i] is None:
            return None
    return paths


# ---- CBS -------------------------------------------------------------------------
def _branches(conflict, starts, min_sep):
    """Two constraints such that every valid plan satisfies at least one of them."""
    kind, a, b, t, x, y = conflict
    if kind == 'edge':
        return [(a, ('e', t, x[0], x[1])), (b, ('e', t, y[0], y[1]))]
    if min_sep <= 1:
        return [(a, ('v', t, frozenset([x]))), (b, ('v', t, frozenset([y])))]
    # Big robots. Forbidding one cell barely helps (the robot steps one cell over
    # and clashes again), so forbid a REGION for each robot: Sa around x and Sb
    # around y, chosen so that every cell of Sa is too close to every cell of Sb.
    # A valid plan can then never have a in Sa and b in Sb at step t, so
    # "a not in Sa" OR "b not in Sb" still covers every valid plan (no plan lost).
    need2 = _need2(starts[a], starts[b], min_sep)
    r = int(math.isqrt(need2)) + 1
    box = [(x[0] + i, x[1] + j) for i in range(-r, r + 1) for j in range(-r, r + 1)]
    best = None
    for ra2 in (0, 1, 2, 4, 5, 8):                     # grow Sa ring by ring around x
        ra = int(math.isqrt(ra2))
        sa = [(x[0] + i, x[1] + j) for i in range(-ra, ra + 1) for j in range(-ra, ra + 1)
              if i * i + j * j <= ra2]
        if max(_d2(p, y) for p in sa) >= need2:
            break                                      # y would no longer be in Sb
        sb = [q for q in box if all(_d2(p, q) < need2 for p in sa)]
        if best is None or min(len(sa), len(sb)) > best[0]:
            best = (min(len(sa), len(sb)), frozenset(sa), frozenset(sb))
    return [(a, ('v', t, best[1])), (b, ('v', t, best[2]))]


def cbs(grid, starts, goals, min_sep=1, w=1.0, max_nodes=5000, stats=None):
    """Conflict-based search.

    w = 1.0 : classic CBS, returns the plan with the lowest sum of costs.
    w > 1.0 : bounded-suboptimal CBS. Among all search nodes whose cost is within
              w times the best known lower bound, expand the one with the FEWEST
              conflicts. The returned plan costs at most w times the optimum, and
              the search is often 100x faster. With robot-size separation
              (min_sep 4) on the warehouse map use w = 1.1.

    Returns a list of paths, or None (no plan, or gave up after max_nodes).
    Pass a dict as `stats` to see how much work was done and why it stopped.
    """
    stats = {} if stats is None else stats
    stats.update(expanded=0, generated=1, reason='', cost=None)
    reason = _check_inputs(grid, starts, goals, min_sep)
    if reason:
        stats['reason'] = reason
        return None
    dists = {g: distance_map(grid, g) for g in set(goals)}
    paths = []
    for s, g in zip(starts, goals):
        paths.append(plan_one(grid, s, g, (), dists[g], others=paths, min_sep=min_sep))
        if paths[-1] is None:
            stats['reason'] = 'robot %d has no path even when alone' % (len(paths) - 1)
            return None

    nodes = {}                    # id -> (cost, n_conflicts, constraints, paths, first conflict)
    open_heap, focal, waiting = [], [], []    # (cost, id) / (n_conflicts, cost, id) / (cost, id)
    done = set()
    ids = itertools.count()

    def add(cons, paths, lb):
        found = list(conflicts(paths, min_sep))
        cost, nid = sum_of_costs(paths), next(ids)
        nodes[nid] = (cost, len(found), cons, paths, found[:1])
        heapq.heappush(open_heap, (cost, nid))
        if cost <= w * lb:
            heapq.heappush(focal, (len(found), cost, nid))
        else:
            heapq.heappush(waiting, (cost, nid))

    root_cost = sum_of_costs(paths)
    add(tuple(() for _ in starts), paths, root_cost)
    while True:
        while open_heap and open_heap[0][1] in done:
            heapq.heappop(open_heap)
        if not open_heap:
            stats['reason'] = 'no conflict-free plan exists'
            return None
        lb = open_heap[0][0]                           # nothing cheaper than this exists
        while waiting and waiting[0][0] <= w * lb:     # lower bound rose: admit more nodes
            cost, nid = heapq.heappop(waiting)
            heapq.heappush(focal, (nodes[nid][1], cost, nid))
        _, _, nid = heapq.heappop(focal)
        done.add(nid)
        cost, _, cons, paths, first = nodes.pop(nid)
        if not first:
            stats['cost'] = cost
            return paths
        stats['expanded'] += 1
        if stats['expanded'] > max_nodes:
            stats['reason'] = 'gave up after %d nodes' % max_nodes
            return None
        for agent, con in _branches(first[0], starts, min_sep):
            new_cons = list(cons)
            new_cons[agent] = cons[agent] + (con,)
            rest = paths[:agent] + paths[agent + 1:]
            path = plan_one(grid, starts[agent], goals[agent], new_cons[agent],
                            dists[goals[agent]], others=rest, min_sep=min_sep)
            if path is None:
                continue
            new_paths = list(paths)
            new_paths[agent] = path
            stats['generated'] += 1
            add(tuple(new_cons), new_paths, lb)


def resolve(grid, starts, goals, min_sep=1, w=1.1, max_nodes=1500, stats=None):
    """What the coordinator should call: CBS first, prioritized planning as backup.

    CBS can give up on a head-on clash inside a one-lane corridor (it has to rule
    out a huge number of almost-equal waiting patterns). Prioritized planning
    then tries every robot order (fine for 2-4 robots) and keeps the cheapest.
    Returns (paths, method) with method 'cbs' or 'prioritized', or (None, reason).
    """
    stats = {} if stats is None else stats
    paths = cbs(grid, starts, goals, min_sep, w, max_nodes, stats)
    if paths is not None:
        return paths, 'cbs'
    if stats['reason'].startswith(('two robots', 'robot', 'robots')):
        return None, stats['reason']             # input problem, an order will not help
    best = None
    for order in itertools.permutations(range(len(starts))):
        p = prioritized(grid, starts, goals, order, min_sep)
        if p is not None and (best is None or sum_of_costs(p) < sum_of_costs(best)):
            best = p
    if best is None:
        return None, stats['reason'] + '; prioritized planning failed in every order'
    return best, 'prioritized'


# ---- execution helper for Phase 5 option 2 (concurrent schedules with waits) --------
def wait_for_rules(paths, min_sep=1):
    """Turn a timed plan into 'wait for' rules that stay safe when Nav2 is faster
    or slower than planned.

    A rule (b, k, a, j) means: robot b must not start moving to its step k until
    robot a has reached its step j. Following the rules keeps the same passing
    order as the plan whatever the real speeds are, so planned timing is only a
    guide and drift cannot create a collision.
    """
    rules = []
    for a, b in itertools.permutations(range(len(paths)), 2):
        need2 = _need2(paths[a][0], paths[b][0], min_sep)
        done = 0                                  # a's step already required by an earlier rule
        for k in range(1, len(paths[b])):
            cell = paths[b][k]
            last = None                           # a's last step before k that is too close
            for i in range(k - 1, -1, -1):
                if _d2(at(paths[a], i), cell) < need2:
                    last = i
                    break
            if last is not None and last + 1 > done:
                done = last + 1
                rules.append((b, k, a, done))
    return rules
