"""Phase 7: failure detection (heartbeats and timeouts) and recovery bookkeeping.

Pure Python, no ROS, so it is easy to test. The task manager feeds it what it
sees and acts on the events it returns.

Two heartbeats per robot:
  * odometry      : Gazebo streams /<robot>/odom while the robot exists.
                    Silent for `heartbeat_timeout` s -> the robot is lost.
  * navigation    : the robot's Nav2 action servers are visible in the ROS graph.
                    Gone for `nav_timeout` s -> its navigation has crashed.
Robot states:
  starting -> up       both heartbeats healthy (first time: 'ready')
  up       -> down     a heartbeat timed out ('down', reason)
  down     -> up       both healthy again for `revive_after` s ('back')
A robot that is up but has not got further along its route for `stuck_timeout` s
while driving is reported 'stuck' (once per stall).

Blocked areas: cells to keep out of, each forgotten after `block_seconds`
(the obstacle may have been cleared by then).
Task attempts: a task given back by a failed robot is retried by another robot,
at most `max_attempts` times in total.
"""
STARTING, UP, DOWN = 'starting', 'up', 'down'


class Robot:
    def __init__(self):
        self.state = STARTING
        self.reason = ''
        self.last_odom = None         # time of the last odometry message
        self.nav_lost = 0.0           # time Nav2 was first seen missing, None if visible
        self.healthy_since = None     # time both heartbeats became healthy
        self.progress = None          # (plan id, progress index) last seen
        self.progress_time = None     # when progress last changed
        self.driving = False
        self.stuck_reported = False


class Supervisor:
    def __init__(self, names, heartbeat_timeout=3.0, nav_timeout=3.0, stuck_timeout=25.0,
                 revive_after=10.0, block_seconds=120.0, max_attempts=3):
        self.robots = {n: Robot() for n in names}
        self.heartbeat_timeout = heartbeat_timeout
        self.nav_timeout = nav_timeout
        self.stuck_timeout = stuck_timeout
        self.revive_after = revive_after
        self.block_seconds = block_seconds
        self.max_attempts = max_attempts
        self.blocks = []              # (cells, until)
        self.attempts = {}            # task id -> attempts used
        self.started = None           # time of the first check
        self.odom_grace = 15.0        # no odometry ever after this long: use Nav2 heartbeat only
        self.nav_only = set()

    # ---- inputs ------------------------------------------------------------------
    def odom(self, name, now):
        self.robots[name].last_odom = now

    def nav(self, name, visible, now):
        r = self.robots[name]
        if visible:
            r.nav_lost = None
        elif r.nav_lost is None:
            r.nav_lost = now

    def progress(self, name, value, driving, now):
        """value changes whenever the robot gets further (or gets a new plan)."""
        r = self.robots[name]
        if value != r.progress or not driving or not r.driving:
            r.progress, r.progress_time = value, now
            r.stuck_reported = False
        r.driving = driving

    def force_down(self, name, reason, now):
        r = self.robots[name]
        if r.state != DOWN:
            r.state, r.reason, r.healthy_since = DOWN, reason, None
            return [('down', name, reason)]
        return []

    # ---- checks ------------------------------------------------------------------
    def problem(self, name, now):
        """Why this robot is unhealthy right now, or '' if it is fine."""
        r = self.robots[name]
        if r.last_odom is None and name not in self.nav_only:
            return 'no odometry yet'
        if r.last_odom is not None and now - r.last_odom > self.heartbeat_timeout:
            return 'heartbeat lost: no odometry for %.0f s' % (now - r.last_odom)
        if r.nav_lost is not None and now - r.nav_lost > self.nav_timeout:
            return 'navigation not responding for %.0f s' % (now - r.nav_lost)
        if r.nav_lost is not None:
            return 'navigation not visible'
        return ''

    def check(self, now):
        """Advance every robot's state. Returns events:
        ('ready', name) first time healthy, ('back', name) recovered after a failure,
        ('down', name, reason), ('stuck', name, seconds without progress),
        ('nav_only', name) when a robot never sent odometry."""
        events = []
        self.started = now if self.started is None else self.started
        for name, r in self.robots.items():
            if r.last_odom is None and name not in self.nav_only and r.nav_lost is None \
                    and now - self.started > self.odom_grace:
                self.nav_only.add(name)
                events.append(('nav_only', name))
            problem = self.problem(name, now)
            if r.state == UP:
                if problem and not problem.startswith('navigation not visible'):
                    r.state, r.reason, r.healthy_since = DOWN, problem, None
                    events.append(('down', name, problem))
                elif r.driving and r.progress_time is not None and not r.stuck_reported \
                        and now - r.progress_time > self.stuck_timeout:
                    r.stuck_reported = True
                    events.append(('stuck', name, now - r.progress_time))
                continue
            if problem:                            # starting or down: wait until healthy
                r.healthy_since = None
                continue
            if r.healthy_since is None:
                r.healthy_since = now
            if r.state == STARTING:
                r.state = UP
                events.append(('ready', name))
            elif now - r.healthy_since >= self.revive_after:
                r.state, r.reason = UP, ''
                r.progress_time = now
                events.append(('back', name))
        return events

    def is_up(self, name):
        return self.robots[name].state == UP

    def down(self):
        """Robots that cannot be planned for: out of service, or not started yet."""
        return {n for n, r in self.robots.items() if r.state != UP}

    def no_progress_for(self, name, now):
        r = self.robots[name]
        return 0.0 if r.progress_time is None else now - r.progress_time

    # ---- blocked areas --------------------------------------------------------------
    def block(self, cells, now):
        self.blocks.append((frozenset(cells), now + self.block_seconds))

    def blocked(self, now):
        self.blocks = [(c, t) for c, t in self.blocks if t > now]
        out = set()
        for cells, _ in self.blocks:
            out |= cells
        return out

    # ---- task attempts ------------------------------------------------------------
    def give_back(self, task_id):
        """A robot gave the task back. True if another robot may try, False if it is
        out of attempts and should be marked FAILED."""
        self.attempts[task_id] = self.attempts.get(task_id, 1) + 1
        return self.attempts[task_id] <= self.max_attempts

    def attempt(self, task_id):
        return self.attempts.get(task_id, 1)
