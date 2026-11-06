"""Bounded XY visibility routes for the nominal controller, upstream of the QP.

Peer positions are frozen for one geometric search, not assumed stationary by
the safety filter. A route is guidance only: every acceleration still has to
pass the current-state HOCBF, actuator and complete-batch held-path checks.
"""
import heapq
import time
import numpy as np
from .settings import pair_distance, virtual_box
from .obstacles import obstacle_radius


def segment_distances(start, end, centers):
    """Exact point-to-closed-segment distances, including a zero-length edge."""
    delta = np.asarray(end)-start
    t = np.clip((np.asarray(centers)-start)@delta/max(float(delta@delta), 1e-15), 0., 1.)
    return np.linalg.norm(np.asarray(start)+t[:, None]*delta-centers, axis=1)


class VisibilityRoute:
    def __init__(self, cfg, index, options):
        self.cfg, self.index, self.a = cfg, index, options
        self.path = []
        self.target = None
        self.countdown = 0.
        self.searches = 0
        self.last_audit = {}

    def circles(self, q):
        centers = [np.asarray(o['center_xy'], float) for o in self.cfg['obstacles']]
        radii = [obstacle_radius(o, self.cfg) for o in self.cfg['obstacles']]
        labels = ['obstacle:'+o['name'] for o in self.cfg['obstacles']]
        for j in range(len(q)):
            if j != self.index:
                centers.append(q[j, :2]);radii.append(pair_distance(self.cfg))
                labels.append('pair:'+self.cfg['names'][j])
        return np.asarray(centers).reshape(-1, 2), np.asarray(radii), labels

    def planning_geometry(self, start, goal, q):
        centers, radii, labels = self.circles(q)
        # Extra planning clearance tapers at tight endpoints. The actual hard
        # safety radii are never reduced, including when no route can be found.
        extra = np.maximum(0., np.minimum(self.a['route_margin_m'], np.minimum(
            .4*(np.linalg.norm(start-centers, axis=1)-radii),
            .4*(np.linalg.norm(goal-centers, axis=1)-radii))))
        return centers, radii+extra, labels

    def direct_clear(self, start, goal, q):
        lo, hi = virtual_box(self.cfg)
        if np.any(goal <= lo[:2]) or np.any(goal >= hi[:2]):return False
        centers, radii, _ = self.circles(q)
        # Hysteresis retains the bypass until a genuinely clear goal segment
        # exists, rather than releasing on a closest-approach prediction alone.
        margin = min(.015, self.a['route_margin_m'])
        return bool(np.all(segment_distances(start, goal, centers) >= radii+margin))

    def plan(self, start, goal, q):
        began = time.perf_counter();self.searches += 1
        centers, radii, labels = self.planning_geometry(start, goal, q)
        lo, hi = (x[:2] for x in virtual_box(self.cfg))
        audit = dict(search=self.searches, nodes=0, status='blocked', reason='', hazards=labels)
        def done(path, reason):
            audit.update(status='found' if path else 'blocked', reason=reason,
                         search_wall_s=time.perf_counter()-began)
            return path, audit
        if np.any(start <= lo) or np.any(start >= hi) or np.any(goal <= lo) or np.any(goal >= hi):
            return done([], 'endpoint_outside_reference_arena')
        if np.any(np.linalg.norm(goal-centers, axis=1) < radii-1e-7):
            return done([], 'goal_occupied_by_inflated_hazard')
        samples = self.a['route_circle_samples']
        if 2+samples*len(centers) > self.a['route_max_nodes']:
            return done([], 'visibility_node_budget')
        # Circumscribed polygon chords stay outside their corresponding disk.
        angles = np.arange(samples)*2*np.pi/samples
        unit = np.column_stack((np.cos(angles), np.sin(angles)))
        rings = [center+(radius/np.cos(np.pi/samples)+.002)*unit
                 for center, radius in zip(centers, radii)]
        points = np.vstack([start, goal, *rings])
        inner_lo, inner_hi = lo+self.a['route_wall_margin_m'], hi-self.a['route_wall_margin_m']
        valid = (points >= inner_lo).all(axis=1) & (points <= inner_hi).all(axis=1)
        for center, radius in zip(centers, radii):
            valid &= np.linalg.norm(points-center, axis=1) >= radius-1e-7
        # Endpoints were separately checked against the hard arena. A valid
        # tight endpoint need not have the optional extra wall planning margin.
        valid[:2] = True
        points = points[valid];audit['nodes'] = len(points)
        delta = points[None, :, :]-points[:, None, :]
        lengths = np.linalg.norm(delta, axis=2)
        clear = np.ones(lengths.shape, bool)
        for center, radius in zip(centers, radii):
            t = np.clip(np.sum((center-points[:, None, :])*delta, axis=2)/np.maximum(lengths**2, 1e-15), 0., 1.)
            nearest = points[:, None, :]+t[:, :, None]*delta
            clear &= np.linalg.norm(nearest-center, axis=2) >= radius-1e-7
        np.fill_diagonal(clear, False)
        queue = [(0., 0)];distance = np.full(len(points), np.inf);distance[0] = 0.
        parents = {}
        while queue:
            cost, k = heapq.heappop(queue)
            if cost > distance[k]+1e-10:continue
            if k == 1:break
            for j in np.flatnonzero(clear[k]):
                value = cost+lengths[k, j]
                if value < distance[j]-1e-10:
                    distance[j] = value;parents[j] = k;heapq.heappush(queue, (value, int(j)))
        if not np.isfinite(distance[1]):return done([], 'no_visibility_path')
        indices = [1]
        while indices[-1] != 0:indices.append(parents[indices[-1]])
        audit['length_m'] = float(distance[1])
        return done([points[k].copy() for k in indices[-2::-1]], 'shortest_visibility_path')

    def velocity(self, target, q):
        """Return a bounded route velocity or None for the existing fallback."""
        dt = self.cfg['control']['dt_s'];self.countdown -= dt
        start, goal = q[self.index, :2], np.asarray(target)[:2]
        if self.direct_clear(start, goal, q):
            self.path = [];self.target = None
            return None, dict(status='clear', reason='direct_goal_segment_clear', searches=self.searches)
        changed = self.target is None or np.linalg.norm(goal-self.target) > self.a['route_goal_change_m']
        # Bound replanning frequency; the unchanged current-state QP handles
        # peer movement between snapshots. Expired waypoints are never safety
        # certificates and never alter the hard constraint set.
        audit = dict(status='latched', reason='following_waypoints', searches=self.searches)
        if self.countdown <= 0. or changed:
            self.path, audit = self.plan(start, goal, q)
            self.countdown = self.a['route_replan_s'];self.target = goal.copy()
            self.last_audit = audit.copy()
        if not self.path:return None, dict(self.last_audit, cached=True)
        while len(self.path) > 1 and np.linalg.norm(self.path[0]-start) < self.a['route_waypoint_tolerance_m']:
            self.path.pop(0)
        aim = self.path[0]-start
        velocity = 1.1*aim
        velocity *= min(1., self.a['passing_speed_mps']/max(float(np.linalg.norm(velocity)), 1e-12))
        audit.update(waypoint_m=self.path[0].tolist(), remaining_waypoints=len(self.path), target_m=goal.tolist())
        return velocity, audit
