"""Tests for the Phase 6 execution layer (no ROS).  Run:  python3 test_fleet.py

A tiny simulator stands in for Gazebo + Nav2 + AMCL, and is deliberately unfair:
  * every segment is driven at a random speed (0.2 to 0.5 m/s), so robots never
    keep the planned timing,
  * a segment counts as done anywhere within 0.25 m of its last waypoint (like
    Nav2's xy_goal_tolerance), so robots often stop short,
  * poses arrive only after the robot moved 0.25 m (like AMCL), with 2 cm noise.
The dispatcher loop below does what task_manager does: assign tasks, replan on
every leg start/end, send segments, report progress.
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fleet import LEG, Fleet, default_min_sep           # noqa: E402
from grid_map import GridMap                             # noqa: E402
from locations import LOCATIONS                          # noqa: E402
from tasks import (DONE, PENDING, TO_DELIVERY, TO_PICKUP,  # noqa: E402
                   Task, assign)

MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'warehouse_map.yaml')
CONTACT = 2 * 0.38                  # robot centres closer than this = collision
DT = 0.1
failed = 0


def check(name, ok):
    global failed
    print(('PASS  ' if ok else 'FAIL  ') + name)
    failed += 0 if ok else 1


class SimRobot:
    def __init__(self, xy, rng):
        self.x, self.y = xy
        self.rng = rng
        self.route = []              # waypoints still to drive
        self.speed = 0.0
        self.stop_short = 0.0
        self.last_pose = (self.x, self.y)

    def drive(self, waypoints):
        self.route = [(x, y) for x, y, _ in waypoints]
        self.speed = self.rng.uniform(0.2, 0.5)
        self.stop_short = self.rng.uniform(0.0, 0.25)

    def stop(self):
        self.route = []

    def step(self):
        """Move for DT seconds. Returns True when the segment just finished."""
        if not self.route:
            return False
        budget = self.speed * DT
        while self.route and budget > 1e-9:
            tx, ty = self.route[0]
            d = math.hypot(tx - self.x, ty - self.y)
            last = len(self.route) == 1
            room = d - (self.stop_short if last else 0.0)
            if room <= 1e-6:                     # reached this waypoint
                self.route.pop(0)
                if last:
                    return True
                continue
            move = min(budget, room)
            self.x += (tx - self.x) / d * move
            self.y += (ty - self.y) / d * move
            budget -= move
        return False

    def amcl(self):
        """A noisy pose, but only after moving 0.25 m since the last one."""
        if math.hypot(self.x - self.last_pose[0], self.y - self.last_pose[1]) < 0.25:
            return None
        self.last_pose = (self.x, self.y)
        return (self.x + self.rng.gauss(0, 0.02), self.y + self.rng.gauss(0, 0.02))


def run(grid, robots, specs, seed, max_time=1500.0):
    """Run a full task list. Returns (all done, closest distance, sim seconds, stats)."""
    rng = random.Random(seed)
    sims = {n: SimRobot(xy, rng) for n, xy in robots.items()}
    fleet = Fleet(grid, list(robots), min_sep=default_min_sep(grid.cell))
    for n, s in sims.items():
        fleet.on_pose(n, (s.x, s.y))
    tasks = [Task(i, *spec.split(':')) for i, spec in enumerate(specs, start=1)]
    task_of = {n: None for n in robots}
    completed = {n: 0 for n in robots}
    driving = {n: False for n in robots}
    stats = {'replans': 0, 'deferred': 0, 'staged': 0, 'stalls': 0, 'fallbacks': 0}
    signature, last_activity = None, 0.0
    replan = True
    closest = float('inf')
    t = 0.0

    def goal(n):
        task = task_of[n]
        if task is None:
            return None
        return LOCATIONS[task.pickup if task.state == TO_PICKUP else task.delivery]

    while t < max_time:
        if all(task.state == DONE for task in tasks):
            return True, closest, t, stats
        # 1 Hz: hand pending tasks to idle robots (same as task_manager.tick)
        if abs(t - round(t)) < 1e-9:
            idle = {n: {'xy': (sims[n].x, sims[n].y), 'completed': completed[n]}
                    for n in robots if task_of[n] is None}
            pending = [task for task in tasks if task.state == PENDING]
            if idle and pending:
                for task, n, _ in assign(grid, pending, idle, LOCATIONS)[0]:
                    task.state, task.robot, task_of[n] = TO_PICKUP, n, task
                    replan = True
        if replan:                    # stop everyone, plan jointly from where they are
            for n, s in sims.items():
                s.stop()
                driving[n] = False
                fleet.on_pose(n, (s.x, s.y))
            fleet.stop_all()
            urgent = sorted((task_of[n].id, n) for n in robots if task_of[n])
            result = fleet.plan({n: goal(n) for n in robots}, [n for _, n in urgent])
            stats['replans'] += 1
            stats['deferred'] += len(result['deferred'])
            stats['staged'] += len(result['staging'])
            stats['fallbacks'] += result['method'] != 'cbs'
            replan = False
        waiting = False
        for n in robots:              # dispatch (same as task_manager.control)
            if driving[n]:
                continue
            what, data = fleet.next_segment(n)
            if what == 'drive':
                sims[n].drive(data)
                driving[n] = True
            elif what == 'done' and data == LEG:
                task = task_of[n]
                if task.state == TO_PICKUP:
                    task.state = TO_DELIVERY
                else:
                    task.state, task_of[n] = DONE, None
                    completed[n] += 1
                replan = True
            elif what == 'wait' or (task_of[n] and fleet.legs[n].kind != LEG):
                waiting = True        # rule-blocked, or deferred with a task to do
        if fleet.progress_signature() != signature or not waiting:
            signature, last_activity = fleet.progress_signature(), t
        elif t - last_activity > 20.0:   # nobody got further for 20 s: replan
            stats['stalls'] += 1
            replan = True
        for n, s in sims.items():     # physics
            if s.step():
                fleet.on_segment_done(n)
                driving[n] = False
            pose = s.amcl()
            if pose:
                fleet.on_pose(n, pose)
        names = list(sims)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                closest = min(closest, math.hypot(sims[a].x - sims[b].x, sims[a].y - sims[b].y))
        t = round(t + DT, 6)
    return False, closest, t, stats


def scenario(grid, title, robots, specs, seeds):
    print('--- %s: %d robots, tasks %s, %d random runs ---'
          % (title, len(robots), specs, len(seeds)))
    results = [run(grid, robots, specs, s) for s in seeds]
    done = sum(r[0] for r in results)
    closest = min(r[1] for r in results)
    slowest = max(r[2] for r in results)
    total = {k: sum(r[3][k] for r in results) for k in results[0][3]}
    print('      closest approach %.2f m, slowest run %.0f s sim time, %s'
          % (closest, slowest, total))
    check('%s: every task delivered in every run (no deadlock)' % title, done == len(seeds))
    check('%s: robots never touched (closest %.2f m > %.2f m)' % (title, closest, CONTACT),
          closest > CONTACT)
    return total


if __name__ == '__main__':
    grid = GridMap.from_yaml(MAP_YAML)
    sep = default_min_sep(grid.cell)
    print('separation used live: %d cells = %.2f m' % (sep, sep * grid.cell))
    check('Nav2 tolerance 0.25 m -> 5 cells; tolerance 0.1 m -> 4 cells',
          sep == 5 and default_min_sep(grid.cell, stop_tolerance=0.1) == 4)
    spawn = {'robot_1': (-8.0, 0.0), 'robot_2': (-8.3, -1.0)}
    scenario(grid, 'default tasks', spawn, ['P1:D1', 'P2:D2', 'P1:D2', 'P2:D1'], range(5))
    scenario(grid, 'shared pickup', spawn, ['P1:D1', 'P1:D2', 'P1:D1'], range(5))
    scenario(grid, 'head-on', spawn, ['P2:D1', 'D1:P1', 'P1:D2', 'D2:P2'], range(6))
    three = dict(spawn, robot_3=(8.0, 0.0))
    scenario(grid, 'three robots', three,
             ['P1:D1', 'D1:P2', 'P2:D2', 'D2:P1', 'P1:D2', 'D1:P1'], range(3))
    print('\n%s' % ('ALL TESTS PASSED' if failed == 0 else '%d TEST(S) FAILED' % failed))
    sys.exit(1 if failed else 0)
