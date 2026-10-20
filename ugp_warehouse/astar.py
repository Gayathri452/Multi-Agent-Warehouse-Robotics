"""A* path planning on a GridMap (single robot).

State      : a grid cell (row, col)
Neighbors  : up / down / left / right (4-connected), only if free
Cost g     : number of steps from the start (every step costs 1)
Heuristic h: Manhattan distance to the goal (never overestimates on a
             4-connected grid, so A* returns a shortest path)
f = g + h  : the cell with the smallest f is expanded next
"""
import heapq

MOVES = ((-1, 0), (1, 0), (0, -1), (0, 1))


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    """Return the list of cells from start to goal (inclusive), or None if no path."""
    if not grid.is_free(start) or not grid.is_free(goal):
        return None
    if start == goal:
        return [start]

    counter = 0                                   # tie-breaker so heap never compares cells
    open_heap = [(manhattan(start, goal), counter, start)]
    g_cost = {start: 0}
    parent = {start: None}
    closed = set()

    while open_heap:
        _, _, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue
        if cell == goal:
            return reconstruct(parent, cell)
        closed.add(cell)
        for dr, dc in MOVES:
            nxt = (cell[0] + dr, cell[1] + dc)
            if not grid.is_free(nxt) or nxt in closed:
                continue
            new_g = g_cost[cell] + 1
            if new_g < g_cost.get(nxt, float('inf')):
                g_cost[nxt] = new_g
                parent[nxt] = cell
                counter += 1
                heapq.heappush(open_heap, (new_g + manhattan(nxt, goal), counter, nxt))
    return None                                   # open list empty: goal unreachable


def reconstruct(parent, cell):
    path = []
    while cell is not None:
        path.append(cell)
        cell = parent[cell]
    path.reverse()
    return path


def simplify(path):
    """Keep only the start, the end and the cells where the direction changes."""
    if path is None or len(path) <= 2:
        return path
    out = [path[0]]
    for prev, cur, nxt in zip(path, path[1:], path[2:]):
        if (cur[0] - prev[0], cur[1] - prev[1]) != (nxt[0] - cur[0], nxt[1] - cur[1]):
            out.append(cur)
    out.append(path[-1])
    return out
