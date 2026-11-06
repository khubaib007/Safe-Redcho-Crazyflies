"""Bounded timestamped pose histories. A rejected sample never refreshes age."""
from collections import deque
import math
import numpy as np


def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi

def pose_arrays(pose):
    p=np.array([pose.position.x,pose.position.y,pose.position.z],float);o=pose.orientation;q=np.array([o.x,o.y,o.z,o.w],float)
    norm=np.linalg.norm(q)
    if not np.isfinite(p).all() or not np.isfinite(q).all() or not .95<=norm<=1.05:raise ValueError('Invalid pose/quaternion')
    return p,q/norm

def yaw(q):
    x,y,z,w=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def tilt(q):return math.acos(float(np.clip(1-2*(q[0]*q[0]+q[1]*q[1]),-1,1)))
def rotation_error(a,b):return float(2*math.acos(float(np.clip(abs(a@b),-1,1))))
def slerp(a,b,f):
    dot=float(a@b)
    if dot<0:b=-b;dot=-dot
    if dot>.9995:r=(1-f)*a+f*b
    else:
        t=math.acos(min(1,dot));r=(math.sin((1-f)*t)*a+math.sin(f*t)*b)/math.sin(t)
    return r/np.linalg.norm(r)

class History:
    def __init__(self,label='pose'):self.label=label;self.rows=deque(maxlen=600);self.rejected=0;self.last_raw=None;self.basis='ros_header';self.last_rejection=None
    def add(self,p,q,stamp,ros_now,mono,maximum_age,onboard=False):
        raw=stamp;basis='ros_header'
        if onboard and (stamp<=0 or abs(ros_now-stamp)>10):stamp=ros_now;basis='receipt_unaligned'
        if not np.isfinite([stamp,ros_now,mono]).all() or stamp<=0 or not -.05<=ros_now-stamp<=maximum_age:
            self.rejected+=1;self.last_rejection='invalid_clock_or_source_age';return False
        if self.rows and (stamp<=self.rows[-1][0] or (raw>0 and self.last_raw is not None and raw<=self.last_raw)):
            self.rejected+=1;self.last_rejection='nonmonotonic_source_stamp';return False
        self.rows.append((stamp,mono,np.asarray(p,float).copy(),np.asarray(q,float).copy()));self.last_raw=raw;self.basis=basis;self.last_rejection=None;return True
    def latest(self,ros_now,mono,age):
        if not self.rows:raise RuntimeError(f'{self.label}: Missing pose history')
        r=self.rows[-1]
        if mono-r[1]>age or not -.05<=ros_now-r[0]<=age:raise RuntimeError(f'{self.label}: Stale pose: source={ros_now-r[0]:.4f}s receipt={mono-r[1]:.4f}s limit={age:.3f}s')
        return r
    def stats(self):
        if len(self.rows)<2:return 0.,0.
        last=self.rows[-1][0];rows=[r for r in self.rows if r[0]>=last-1.]
        coverage=rows[-1][0]-rows[0][0];return ((len(rows)-1)/coverage if coverage else 0.),coverage
    def velocity(self,window,maxgap):
        if not self.rows:raise RuntimeError('Missing mocap velocity history')
        rows=[r for r in self.rows if r[0]>=self.rows[-1][0]-window]
        if len(rows)<5 or rows[-1][0]-rows[0][0]<window*.5:raise RuntimeError('Insufficient mocap velocity window')
        ts=np.array([r[0]-rows[0][0] for r in rows]);gaps=np.diff(ts)
        if np.min(gaps)<=0 or np.max(gaps)>maxgap:raise RuntimeError('Gapped mocap velocity window')
        ts-=ts.mean();p=np.array([r[2] for r in rows]);v=ts@p/(ts@ts)
        return v
    def at(self,stamp,maxskew=.04):
        before=[r for r in self.rows if r[0]<=stamp];after=[r for r in self.rows if r[0]>=stamp]
        if before and after and after[0][0]-before[-1][0]<=maxskew:
            a,b=before[-1],after[0];f=(stamp-a[0])/max(1e-12,b[0]-a[0]);return (1-f)*a[2]+f*b[2],slerp(a[3],b[3],f)
        if not self.rows:raise RuntimeError('No matching mocap pose')
        r=min(self.rows,key=lambda r:abs(r[0]-stamp))
        if abs(r[0]-stamp)>maxskew:raise RuntimeError('No mocap/onboard source-time match within 40 ms')
        return r[2],r[3]
    def touchdown(self,floor,ros_now,mono,maxage):
        try:self.latest(ros_now,mono,maxage)
        except RuntimeError as e:return dict(verified=False,reason=str(e))
        rows=[r for r in self.rows if mono-r[1]<=.6]
        if len(rows)<5:return dict(verified=False,reason='Insufficient touchdown samples')
        p=np.array([r[2] for r in rows]);coverage=rows[-1][0]-rows[0][0];gap=max(np.diff([r[0] for r in rows]));span=float(np.max(np.ptp(p,axis=0)));err=float(np.max(np.abs(p[:,2]-floor)))
        good=coverage>=.45 and gap<=maxage and span<.025 and err<.06
        return dict(verified=bool(good),coverage_s=coverage,maximum_gap_s=float(gap),maximum_axis_span_m=span,maximum_floor_error_m=err)
