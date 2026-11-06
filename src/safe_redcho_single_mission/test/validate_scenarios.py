"""Reproduce the enhanced-controller numerical matrix; no ROS/radio imports."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import yaml
from safe_redcho_single_mission.settings import load,make_plan,validate,NAMES
from safe_redcho_single_mission.preview import run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',default=str(Path.home()/'safe_redcho_hardware'))
    parser.add_argument('--output',required=True)
    args=parser.parse_args();root=Path(args.workspace);out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    summary={}
    for name in ('default','combined','obstacle','interagent','combined_fast'):
        scenario='combined' if name=='combined_fast' else name
        stem='single_mission'+('' if scenario=='default' else '_'+scenario)
        c=load(root/'config'/(stem+'.yaml'))
        if name=='combined_fast':c['acquisition']['nominal_duration_s']=7.5;validate(c)
        if scenario=='default':
            z=c['arena']['floor_z_m'];ground=[[-.05,.76,z],[-.78,0,z],[-.05,-.76,z]];yaws=[2.5,-1.8,.9]
        else:
            fixture=yaml.safe_load((root/'config'/(stem+'_fixture.yaml')).read_text())
            ground=[fixture['positions'][n] for n in NAMES];yaws=[fixture['yaw_rad'][n] for n in NAMES]
        plan=make_plan(c,ground,yaws)
        for battery,volts in [('nominal',[4.,3.8,3.7]),('unequal',[3.55,3.45,3.35])]:
            key=name+'_'+battery;directory=out/key;directory.mkdir(exist_ok=True)
            try:
                report,rows=run(c,plan,voltages=volts,trace=True,evidence_path=directory/'hocbf_rows.csv')
                report.update(configuration=deepcopy(c),plan=plan,voltages=volts)
                summary[key]={k:report[k] for k in ('status','duration_s','cycles','minimum_virtual_xy_separation_m',
                    'required_reference_xy_separation_m','minimum_synthetic_xy_separation_m','maximum_synthetic_tracking_error_m',
                    'maximum_reference_slew_mps3','avoidance_active_agent_samples','minimum_motion_progress_scale','stage_events','final_endpoint_errors_m')}
                summary[key]['minimum_normalized_qp_residual']=min(v['minimum_filtered_residual'] for v in report['hocbf_evidence'].values())
                (directory/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
                keys=('time','measured_position','reference_position','reference_velocity','safe_acceleration','position_redcho','motion_derivatives','yaw')
                np.savez_compressed(directory/'trace.npz',**{k:np.array([r[i] for r in rows]) for i,k in enumerate(keys)})
            except Exception as exc:
                summary[key]=dict(status='FAIL',error=type(exc).__name__+': '+str(exc))
            print(key,json.dumps(summary[key]),flush=True)
            (out/'summary.json').write_text(json.dumps(summary,indent=2))
    if any(v['status']!='PASS' for v in summary.values()):raise SystemExit(1)


if __name__=='__main__':main()
