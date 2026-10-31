"""Single-delivery pose adapter and finite fault diagnostics; no control law here."""
import json
import math
from pathlib import Path
import time
import numpy as np

VERSION='source-reviewed-pose-merge-2026-10-23-v2'


def clean(value):
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [clean(v) for v in value]
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,(float,np.floating)) and not math.isfinite(value):return None
    if isinstance(value,np.generic):return value.item()
    return value


def reject(node,name,reason):
    if not hasattr(node,'pose_rejections'):node.pose_rejections={}
    entry=node.pose_rejections.setdefault(name,{'counts':{},'last_reason':None})
    entry['counts'][reason]=entry['counts'].get(reason,0)+1
    entry['last_reason']=reason


def joined_callback(node,topic,callback):
    parts=topic.strip('/').split('/')
    names=getattr(node,'names',())
    if len(parts)!=2 or parts[0] not in names or parts[1]!='pose':return callback
    name=parts[0]
    if not hasattr(node,'_joined_pose_names'):node._joined_pose_names=[]
    node._joined_pose_names.append(name)
    def receive(message):
        # The timestamped receiver decides whether the message is current and
        # valid. Rejected packets must not refresh the legacy stream's age.
        if node.receive_estimator_history(message,name):
            return callback(message)
    return receive


def require_joined_receivers(node):
    names=getattr(node,'_joined_pose_names',[])
    if set(names)!=set(node.names) or any(names.count(n)!=1 for n in node.names):
        raise RuntimeError('Source-reviewed adapter requires exactly one inherited /cfN/pose subscription per aircraft; '
                           f'observed={names}. No flight commands have been sent.')


def stream_details(node,name):
    s=node.profile['safety'];result={'name':name,'stage':node.stage_name,
        'limits':{'mocap_maximum_age_s':s['mocap_maximum_age_s'],
                  'maximum_estimator_age_s':s['maximum_estimator_age_s'],
                  'minimum_mocap_rate_hz':s['minimum_mocap_rate_hz'],
                  'minimum_estimator_rate_hz':s['minimum_estimator_rate_hz']},
        'rejected_pose_packets':getattr(node,'pose_rejections',{}).get(name,{})}
    source_now=node.get_clock().now().nanoseconds*1e-9
    for key,stream in [('mocap',node.mocap[name]),('onboard',node.onboard[name])]:
        result[key]={'received_pose':stream.pose is not None,'frame_id':stream.frame_id,'stats':stream.stats()}
    for key,histories in [('mocap_source',node.mocap_history),('onboard_source',node.estimator_history)]:
        rows=histories[name].rows
        result[key]={'samples':len(rows)}
        if rows:
            result[key].update(source_stamp_s=rows[-1][0],source_age_s=source_now-rows[-1][0],
                               receipt_age_s=time.monotonic()-rows[-1][1])
    return clean(result)


def pose_fault(node,name,message):
    details=stream_details(node,name)
    es=details['onboard']['stats'];s=node.profile['safety'];reasons=[]
    ms=details['mocap']['stats']
    if not details['mocap']['received_pose']:reasons.append('no_valid_mocap_pose')
    if details['mocap']['frame_id']!=node.profile['frame_id']:reasons.append('mocap_frame_mismatch')
    if ms.get('age_s') is None or ms['age_s']>s['mocap_maximum_age_s']:reasons.append('mocap_age_exceeded')
    if ms.get('rate_hz',0)<s['minimum_mocap_rate_hz']:reasons.append('mocap_rate_below_minimum')
    if ms.get('maximum_gap_s') is None or ms['maximum_gap_s']>s['mocap_maximum_age_s']:reasons.append('mocap_gap_exceeded')
    if not details['onboard']['received_pose']:reasons.append('no_valid_onboard_pose')
    if details['onboard']['frame_id']!=node.profile['frame_id']:reasons.append('onboard_frame_mismatch')
    if es.get('age_s') is None or es['age_s']>s['maximum_estimator_age_s']:reasons.append('onboard_age_exceeded')
    if es.get('rate_hz',0)<s['minimum_estimator_rate_hz']:reasons.append('onboard_rate_below_minimum')
    node.guard_fault(message,name=name,reasons=reasons,pose_streams=details)


def persist_fault(node):
    # Existing final measurement_diagnostics.json is preserved. This small
    # immediate report also covers failure before final cleanup completes.
    report=clean({'version':VERSION,'last_fault':node.last_fault,
                  'profile_path':node.profile.get('_path'),
                  'pose_rejections':getattr(node,'pose_rejections',{}),
                  'latest_measurement':getattr(node,'latest_measurement',None)})
    now=time.monotonic()
    if now-getattr(node,'_last_immediate_fault_write',-math.inf)<.5:return
    try:
        (Path(node.output)/'health_fault.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        node._last_immediate_fault_write=now
    except Exception as exc:
        # Diagnostic I/O cannot mask the original health failure.
        node.last_fault['diagnostic_write_error']=str(exc)


def minimum_forward_budget(profile):
    m=profile['mission'];b=profile['battery']
    return (m['observer_warmup_s']+m['nominal_forward_duration_s']/b['minimum_speed_scale']+
            m['endpoint_dwell_s']+m['final_hold_s'])


def validate_forward_budget(profile):
    minimum=minimum_forward_budget(profile)
    if profile['mission']['maximum_wall_duration_s']<=minimum:
        raise RuntimeError('Forward mission wall timeout is too short: '
            f"{profile['mission']['maximum_wall_duration_s']:.3f}s configured; "
            f'more than {minimum:.3f}s required before wall-time scheduling margin. '
            'Correct mission.maximum_wall_duration_s before flight.')
