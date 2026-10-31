"""Timestamp-aware measurements and diagnostics. Never extrapolates a missing pose."""
from collections import deque
import copy
import json
import math
from pathlib import Path
import time
import numpy as np

VERSION = 'health-sync-2026-10-23-v1'


def history(node):
    if not hasattr(node, '_health_sync_history'):
        node._health_sync_history = {kind:{n:deque(maxlen=600) for n in node.names} for kind in ('mocap','onboard')}
        node._health_sync_counters = {}
    return node._health_sync_history


def count(node, key):
    node._health_sync_counters[key] = node._health_sync_counters.get(key,0)+1


def quaternion(pose):
    q = pose.orientation
    a = np.array([q.x,q.y,q.z,q.w],float)
    norm = np.linalg.norm(a)
    if not np.isfinite(a).all() or not .95 <= norm <= 1.05:
        raise ValueError('Invalid quaternion norm')
    return a/norm


def angle(a,b):
    return float(2*math.acos(float(np.clip(abs(np.dot(a,b)),-1.,1.))))


def interpolate(a,b,fraction):
    dot = float(a@b)
    if dot < 0: b, dot = -b,-dot
    if dot > .9995:
        value = (1-fraction)*a+fraction*b
    else:
        theta = math.acos(float(np.clip(dot,-1.,1.)))
        value = (math.sin((1-fraction)*theta)*a+math.sin(fraction*theta)*b)/math.sin(theta)
    return value/np.linalg.norm(value)


def observe(node, topic, message):
    """Runs before an existing callback, without changing its message or outcome."""
    h = history(node)
    received = time.monotonic()
    ros_ns = int(node.get_clock().now().nanoseconds)
    stamp = message.header.stamp
    raw_source_ns = int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
    source_ns, time_basis = raw_source_ns, 'ros_header'
    if topic == '/poses':
        kind, entries = 'mocap', [(p.name,p.pose) for p in message.poses]
        names = [n for n,_ in entries]
        duplicates = {n for n in names if names.count(n)>1}
        if duplicates: count(node,'duplicate_mocap_names')
    else:
        parts = topic.strip('/').split('/')
        if len(parts)!=2 or parts[1]!='pose' or parts[0] not in node.names: return
        kind, entries, duplicates = 'onboard', [(parts[0],message.pose)], set()
    if message.header.frame_id != node.profile['frame_id']:
        count(node,kind+'_frame_rejected');return
    # Mocap must supply a ROS-clock header. Some onboard backends supply zero
    # or boot-relative pose stamps. Retain their fresh received poses but make
    # no source-time alignment claim in that case; preserve the original angle
    # comparison instead. This avoids imposing an unsupported backend clock.
    if kind == 'onboard':
        if source_ns<=0 or abs(ros_ns-source_ns)>10_000_000_000:
            source_ns,time_basis=ros_ns,'receipt_fallback_unaligned'
        elif (ros_ns-source_ns>int(node.profile['safety']['maximum_estimator_age_s']*1e9)
              or source_ns-ros_ns>50_000_000):
            count(node,kind+'_header_rejected');return
    if kind == 'mocap' and (source_ns<=0 or ros_ns-source_ns>150_000_000 or source_ns-ros_ns>50_000_000):
        count(node,kind+'_header_rejected');return
    for name,pose in entries:
        if name not in node.names or name in duplicates: continue
        try:
            p=np.array([pose.position.x,pose.position.y,pose.position.z],float)
            q=quaternion(pose)
            if not np.isfinite(p).all():raise ValueError('Invalid position')
        except ValueError:
            count(node,kind+'_'+name+'_invalid');continue
        rows=h[kind][name]
        if rows and (source_ns<=rows[-1]['source_ns'] or
                     (raw_source_ns>0 and raw_source_ns<=rows[-1].get('raw_source_ns',0))):
            count(node,kind+'_'+name+'_nonincreasing_header');continue
        rows.append({'source_ns':source_ns,'receipt_mono':received,'receipt_ros_ns':ros_ns,
                     'raw_source_ns':raw_source_ns,'time_basis':time_basis,'p':p,'q':q})


def wrap_callback(node, topic, callback):
    interested = topic == '/poses' or topic in {f'/{n}/pose' for n in getattr(node,'names',())}
    if not interested:return callback
    def received(message):
        # Instrumentation must not prevent the original safety receiver running.
        try:observe(node,topic,message)
        except Exception as exc:
            history(node);count(node,'capture_exception_'+type(exc).__name__)
        return callback(message)
    return received


def fit_source_velocity(rows, window, maximum_gap, now, maximum_age):
    if not rows or now-rows[-1]['receipt_mono']>maximum_age:
        raise RuntimeError('No fresh mocap sample for source-time velocity fit')
    latest=rows[-1]['source_ns']
    fit=[r for r in rows if r['source_ns']>=latest-int(window*1e9)]
    if len(fit)<5 or (fit[-1]['source_ns']-fit[0]['source_ns'])/1e9 < window*.5:
        raise RuntimeError('Insufficient source-time mocap history for velocity fit')
    gaps=np.diff([r['source_ns'] for r in fit])/1e9
    if not len(gaps) or np.min(gaps)<=0 or np.max(gaps)>maximum_gap:
        raise RuntimeError('Source-time velocity window contains a mocap gap')
    ts=np.array([(r['source_ns']-fit[0]['source_ns'])/1e9 for r in fit]);ts-=ts.mean()
    positions=np.array([r['p'] for r in fit])
    velocity=ts@positions/float(ts@ts)
    residual=positions-(positions.mean(axis=0)+ts[:,None]*velocity)
    rms=float(np.sqrt(np.mean(np.sum(residual*residual,axis=1))))
    return velocity, {'fit_samples':len(fit),'source_coverage_s':float(np.ptp(ts)),
                     'source_maximum_gap_s':float(np.max(gaps)),'fit_residual_rms_m':rms}


def snapshot(node):
    h, now, s = history(node),time.monotonic(),node.profile['safety']
    positions, velocities = [],[]
    detail={};node._health_sync_velocity_detail=detail
    for name in node.names:
        rows=h['mocap'][name]
        vv,stats=fit_source_velocity(rows,s['mocap_velocity_window_s'],s['mocap_maximum_age_s'],
                                     now,s['mocap_maximum_age_s'])
        positions.append(rows[-1]['p'].copy());velocities.append(vv)
        stats['speed_mps']=float(np.linalg.norm(vv));stats['latest_source_ns']=rows[-1]['source_ns'];detail[name]=stats
    node._health_sync_velocity_detail=detail
    return np.array(positions),np.array(velocities)


def aligned_pair(node,name):
    h=history(node); mc, est = h['mocap'][name],h['onboard'][name]
    s=node.profile['safety'];now=time.monotonic()
    if not mc or not est:raise RuntimeError(f'{name}: timestamped mocap/onboard history not ready')
    if now-mc[-1]['receipt_mono']>s['mocap_maximum_age_s']:
        raise RuntimeError(f'{name}: mocap history is stale; no attitude extrapolation')
    if now-est[-1]['receipt_mono']>s['maximum_estimator_age_s']:
        raise RuntimeError(f'{name}: onboard history is stale; no attitude extrapolation')
    if est[-1].get('time_basis','ros_header') != 'ros_header':
        return {'angle_rad':angle(mc[-1]['q'],est[-1]['q']),
                'position_difference_m':float(np.linalg.norm(mc[-1]['p']-est[-1]['p'])),
                'header_pair_skew_s':None,'latest_unaligned_angle_rad':angle(mc[-1]['q'],est[-1]['q']),
                'mocap_receipt_age_s':now-mc[-1]['receipt_mono'],
                'onboard_receipt_age_s':now-est[-1]['receipt_mono'],
                'timing_scope':'Unaligned latest-pose comparison retained: onboard ROS-clock header unavailable'}
    target=est[-1]['source_ns']; allowed_ns=40_000_000
    before=[r for r in mc if r['source_ns']<=target]
    after=[r for r in mc if r['source_ns']>=target]
    if before and after and after[0]['source_ns']-before[-1]['source_ns']<=allowed_ns:
        a,b=before[-1],after[0]
        f=(target-a['source_ns'])/max(1,b['source_ns']-a['source_ns'])
        mq=interpolate(a['q'],b['q'],f);mp=(1-f)*a['p']+f*b['p'];skew=0.
    else:
        closest=min(mc,key=lambda r:abs(r['source_ns']-target))
        if abs(closest['source_ns']-target)>allowed_ns:
            raise RuntimeError(f'{name}: no mocap/onboard header match within 40 ms')
        mq,mp,skew=closest['q'],closest['p'],(closest['source_ns']-target)/1e9
    return {'angle_rad':angle(mq,est[-1]['q']),
            'position_difference_m':float(np.linalg.norm(mp-est[-1]['p'])),
            'header_pair_skew_s':float(skew),'latest_unaligned_angle_rad':angle(mc[-1]['q'],est[-1]['q']),
            'mocap_receipt_age_s':now-mc[-1]['receipt_mono'],
            'onboard_receipt_age_s':now-est[-1]['receipt_mono'],
            'timing_scope':'ROS header alignment; firmware/radio latency not calibrated'}


def aligned_orientation(node,name):return aligned_pair(node,name)['angle_rad']


def clean(value):
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple,deque)):return [clean(v) for v in value]
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,(float,np.floating)) and not math.isfinite(value):return None
    if isinstance(value,np.generic):return value.item()
    return value


def describe(node):
    h,now=history(node),time.monotonic()
    result={'version':VERSION,'monotonic_time':now,'limits':copy.deepcopy(node.profile['safety']),
            'aircraft':{},'capture_counters':dict(node._health_sync_counters)}
    for name in node.names:
        row={}
        for kind in ('mocap','onboard'):
            samples=list(h[kind][name]);recent=[r for r in samples if r['receipt_mono']>=now-2.]
            covered=(recent[-1]['source_ns']-recent[0]['source_ns'])/1e9 if len(recent)>1 else 0.
            row[kind]={'samples':len(recent),'receipt_age_s':now-samples[-1]['receipt_mono'] if samples else None,
                       'effective_rate_hz':(len(recent)-1)/covered if covered>0 else 0.,
                       'rate_time_basis':samples[-1].get('time_basis','ros_header') if samples else None,
                       'raw_header_ns':samples[-1].get('raw_source_ns') if samples else None,
                       'maximum_source_gap_s':max(np.diff([r['source_ns'] for r in recent])/1e9,default=None),
                       'latest_position_m':samples[-1]['p'] if samples else None}
            legacy=getattr(node,kind,{}).get(name) if isinstance(getattr(node,kind,None),dict) else None
            if legacy is not None and hasattr(legacy,'stats'):
                try:row[kind]['existing_receiver_stats']=legacy.stats()
                except Exception as exc:row[kind]['existing_receiver_stats_error']=str(exc)
        try:row['aligned']=aligned_pair(node,name)
        except RuntimeError as exc:row['alignment_error']=str(exc)
        try:
            vv,fit=fit_source_velocity(h['mocap'][name],node.profile['safety']['mocap_velocity_window_s'],
                node.profile['safety']['mocap_maximum_age_s'],now,node.profile['safety']['mocap_maximum_age_s'])
            fit['speed_mps']=float(np.linalg.norm(vv));row['velocity_fit']=fit
        except RuntimeError as exc:
            row['velocity_fit']=None;row['velocity_fit_error']=str(exc)
        result['aircraft'][name]=row
    return clean(result)


def fault_report(node,exc,ground=False,airborne=False):
    try:
        report=describe(node)
        report.update(error=f'{type(exc).__name__}: {exc}',ground=ground,airborne=airborne)
        node._health_sync_last_fault=report
        # Ground discovery faults can repeat rapidly; bound diagnostic disk I/O.
        now=time.monotonic()
        if now-getattr(node,'_health_sync_last_write',-math.inf)>.5:
            target=Path(node.output)/'health_fault.json';target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
            node._health_sync_last_write=now
        speeds={n:r['velocity_fit'].get('speed_mps') for n,r in report['aircraft'].items() if r['velocity_fit']}
        return f'{exc}; measured_source_time_speeds={speeds}; details={Path(node.output)/"health_fault.json"}'
    except Exception as nested:
        return f'{exc}; health diagnostic error={type(nested).__name__}: {nested}'


def touchdown_evidence(node,name,now=None):
    now=time.monotonic() if now is None else now
    rows=[r for r in history(node)['mocap'][name] if r['receipt_mono']>=now-.5]
    if len(rows)<5:return {'verified':False,'samples':len(rows),'reason':'insufficient recent samples'}
    age=now-rows[-1]['receipt_mono']
    coverage=rows[-1]['receipt_mono']-rows[0]['receipt_mono']
    gaps=np.diff([r['receipt_mono'] for r in rows])
    positions=np.array([r['p'] for r in rows]);ground=node.config['_starts'][name][2]
    error=float(np.max(np.abs(positions[:,2]-ground)));span=float(np.max(np.ptp(positions,axis=0)))
    good=(age<=node.profile['safety']['mocap_maximum_age_s'] and coverage>=.45 and
          float(np.max(gaps))<=node.profile['safety']['mocap_maximum_age_s'] and error<.06 and span<.02)
    return {'verified':bool(good),'samples':len(rows),'receipt_age_s':age,'coverage_s':coverage,
            'maximum_gap_s':float(np.max(gaps)),'maximum_height_error_m':error,'maximum_axis_span_m':span,
            'reason':'qualified' if good else 'recent window not continuously fresh, near ground and stationary'}


def recheck_touchdown(node,report):
    """After the existing landing, allow bounded reacquisition, never blind disarm."""
    from crazyflie_interfaces.srv import Arm
    verified=set(report.get('touchdown_verified',[]))
    initial_missing=[n for n in node.names if n not in verified]
    if not initial_missing:return report
    deadline=time.monotonic()+3.
    evidence={}
    while time.monotonic()<deadline:
        node.spin_for(.02)
        for name in initial_missing:
            evidence[name]=touchdown_evidence(node,name)
        newly=[n for n in initial_missing if evidence[n]['verified'] and n not in verified]
        if newly:
            requests={}
            for name in newly:
                req=Arm.Request();req.arm=False;requests[name]=req
            try:
                node.call_many('arm',requests)
                verified.update(newly)
                report.setdefault('disarm_requests_acknowledged',[]).extend(newly)
            except Exception as exc:
                report['health_sync_disarm_error']=str(exc);break
        if all(n in verified for n in node.names):break
    report['health_sync_touchdown_recheck']=clean(evidence)
    report['touchdown_verified']=[n for n in node.names if n in verified]
    report['touchdown_not_verified']=[n for n in node.names if n not in verified]
    return report
