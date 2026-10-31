"""Bounded plot histories; no interpolation or zero filling in the renderer."""
from collections import defaultdict, deque
import math
import numpy as np


class Histories:
    def __init__(self, points=6000, seconds=90.):
        self.points, self.seconds = points, seconds
        self.series = defaultdict(lambda: deque(maxlen=self.points))
        self.epoch, self.origin, self.last_batch = None, 0, 0
        self.lost_batches, self.stage, self.stages = 0, '', deque(maxlen=30)

    def accept(self, packet):
        if self.epoch != packet['epoch']:
            self.series.clear(); self.stages.clear()
            self.epoch, self.origin, self.last_batch = packet['epoch'], packet['origin_ns'], 0
            self.stage = ''
        batch = packet['batch']
        if batch <= self.last_batch: return
        if self.last_batch: self.lost_batches += max(0, batch-self.last_batch-1)
        self.last_batch = batch
        now = (packet['now_ns']-self.origin)/1e9
        for event in packet['events']:
            t = (event['t_ns']-self.origin)/1e9
            for key, value in event['values'].items():
                rows = self.series[key]
                if rows and t <= rows[-1][0]: continue
                y = float('nan') if value is None else float(value)
                rows.append((t, y))
        for rows in self.series.values():
            while rows and rows[0][0] < now-self.seconds: rows.popleft()
        stage = packet['status']['stage']
        if stage != self.stage:
            self.stages.append((now, stage)); self.stage = stage

    def data(self, key):
        rows = self.series.get(key)
        if not rows: return np.array([]), np.array([])
        a = np.asarray(rows, float)
        # Insert NaN at communication gaps so separate episodes are not joined.
        gaps = np.flatnonzero(np.diff(a[:,0]) > .5)+1
        if len(gaps): a = np.insert(a, gaps, [[np.nan,np.nan]], axis=0)
        return a[:,0], a[:,1]

    def xy(self, xkey, ykey):
        xs, ys = self.series.get(xkey, ()), self.series.get(ykey, ())
        # Coordinates from the same event must have identical timestamps.
        xmap = dict(xs); pairs = []; previous = None
        for t,y in ys:
            if t not in xmap: continue
            if previous is not None and t-previous>.5: pairs.append((np.nan,np.nan))
            pairs.append((xmap[t],y)); previous=t
        return (np.array([]),np.array([])) if not pairs else tuple(np.asarray(pairs, float).T)
