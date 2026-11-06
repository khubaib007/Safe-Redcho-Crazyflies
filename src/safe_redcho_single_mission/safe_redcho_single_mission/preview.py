"""Synthetic tracking preview; never imports ROS or sends hardware commands."""
import argparse,json,time
from pathlib import Path
import numpy as np
from .settings import load,make_plan,NAMES,pair_distance,path_of
from .core import LocalController,StageMachine,accept_batch,wrap
from .avoidance import motion_progress

def run(c,plan,voltages=(4.,3.8,3.7),acquire_only=False,trace=False,evidence_path=None):
    from .evidence import Evidence
    from .obstacles import check_measured_clearance, obstacle_radius
    trace = trace or evidence_path is not None
    evidence = Evidence(evidence_path)
    obstacle_minima = {o["name"]: {"minimum_virtual_center_distance_m":float("inf"), "minimum_synthetic_center_distance_m":float("inf"), "required_reference_center_distance_m":obstacle_radius(o,c)} for o in c["obstacles"]}
    machine=None;seq=0;q=v=previous=None
    try:
        began=time.monotonic();cores=[LocalController(c,i,plan,trace) for i in range(3)];machine=StageMachine(c,plan,acquire_only)
        q=np.asarray(plan['hover']).copy();v=np.zeros((3,3));previous=np.zeros((3,3));p=q.copy();pv=v.copy();yaw=np.asarray(plan['initial_yaw']);dt=c['control']['dt_s'];rows=[]
        maxerr=0.;minsep=100.;minvirtual=100.;maxspeed=0.;minimum_margin=100.;seq=0;qp_active=0;solver_calls=0;nominal_violations=0
        relaxed_samples=0;maximum_slew=0.;maximum_qp_wall_s=0.;progress=1.;avoidance_samples=0;minimum_progress=1.
        while True:
            seq+=1;frame=dict(seq=seq,stage=machine.stage,q=q.tolist(),v=v.tolist(),previous=previous.tolist(),voltage=list(voltages),motion_progress_scale=progress)
            peers=[core.prepare(frame) for core in cores];results=[core.finish({NAMES[j]:peers[j] for j in range(3) if j!=i}) for i,core in enumerate(cores)]
            progress=motion_progress(results,c);minimum_progress=min(minimum_progress,progress);avoidance_samples+=sum(r['avoidance']['active'] for r in results)
            u=np.array([r['u'] for r in results]);qn,vn=accept_batch(q,v,previous,u,c)
            maximum_slew=max(maximum_slew,float(np.max(np.abs(u-previous)))/dt)
            relaxed_samples+=sum(bool(r.get('qp_slew_relaxed',False)) for r in results)
            maximum_qp_wall_s=max(maximum_qp_wall_s,max(r['qp_wall_s'] for r in results))
            pv=(qn-p)/c['acquisition']['preview_tracking_time_constant_s'];p+=dt*pv
            check_measured_clearance(p,c)
            for o in c['obstacles']:
                stat=obstacle_minima[o['name']]
                stat['minimum_virtual_center_distance_m']=min(stat['minimum_virtual_center_distance_m'],float(np.min(np.linalg.norm(qn[:,:2]-o['center_xy'],axis=1))))
                stat['minimum_synthetic_center_distance_m']=min(stat['minimum_synthetic_center_distance_m'],float(np.min(np.linalg.norm(p[:,:2]-o['center_xy'],axis=1))))
            yaw+=dt*wrap(machine.yaw_reference()-yaw)/.15
            error=float(np.max(np.linalg.norm(p-qn,axis=1)));maxerr=max(maxerr,error)
            maxspeed=max(maxspeed,float(np.max(np.linalg.norm(pv,axis=1))))
            minvirtual=min(minvirtual,min(float(np.linalg.norm(qn[i,:2]-qn[j,:2])) for i in range(3) for j in range(i)))
            sep=min(np.linalg.norm(p[i,:2]-p[j,:2]) for i in range(3) for j in range(i));minsep=min(minsep,float(sep))
            if error>c['acquisition']['preview_tracking_error_limit_m']:raise RuntimeError('Synthetic tracking error exceeds preview budget')
            if sep<c['safety']['measured_abort_xy_separation_m']:raise RuntimeError('Synthetic measured separation violated')
            if trace:
                evidence.add(machine.total,c['names'],results,c['control']['qp_tolerance'])
                qp_active+=sum(r['diagnostics']['qp']['hocbf_rows_binding'] for r in results)
                solver_calls+=sum(r['diagnostics']['qp']['inequality_solver_invoked'] for r in results)
                nominal_violations+=sum(r['diagnostics']['qp']['nominal_hocbf_violation_mps2']>1e-6 for r in results)
            if seq%5==0:rows.append((machine.total+dt,p.copy(),qn.copy(),vn.copy(),u.copy(),np.array([r['position_estimate'] for r in results]),np.array([r['motion_derivatives'] for r in results]),yaw.copy()))
            q,v,previous=qn,vn,u
            if machine.advance(results,p,pv,yaw):break
        return dict(status='PASS',scope='Numerical single position REDCHO/QP and synthetic tracking; no ROS/radio/hardware',duration_s=machine.total,wall_s=time.monotonic()-began,cycles=seq,
                    maximum_synthetic_tracking_error_m=maxerr,maximum_synthetic_speed_mps=maxspeed,minimum_synthetic_xy_separation_m=minsep,minimum_virtual_xy_separation_m=minvirtual,required_reference_xy_separation_m=pair_distance(c),
                    final_endpoint_errors_m=np.linalg.norm(p-machine.goal(not acquire_only),axis=1).tolist(),final_height_spread_m=float(np.ptp(p[:,2])),stage_events=machine.events,
                    obstacle_models=c["obstacles"],obstacle_clearances=obstacle_minima,hocbf_evidence=evidence.rows,
                    slew_relaxed_agent_samples=relaxed_samples,maximum_reference_slew_mps3=maximum_slew,maximum_local_qp_wall_s=maximum_qp_wall_s,
                    avoidance_active_agent_samples=avoidance_samples,minimum_motion_progress_scale=minimum_progress,
                    qp_diagnostics_collected=trace,qp_binding_agent_samples=qp_active if trace else None,inequality_solver_agent_samples=solver_calls if trace else None,nominal_hocbf_violation_agent_samples=nominal_violations if trace else None),rows
    except Exception as exc:
        if evidence_path is not None:
            # Preserve the first failed cycle instead of losing the state in a
            # traceback. This is offline only; no flight command is generated.
            failure=dict(status='FAIL',scope='Numerical preview failure; no ROS/radio/hardware',
                error=type(exc).__name__+': '+str(exc),cycle=seq,
                time_s=machine.total if machine is not None else None,
                stage=machine.stage if machine is not None else None,
                stage_events=machine.events if machine is not None else [],
                reference_position_m=q.tolist() if q is not None else None,
                reference_velocity_mps=v.tolist() if v is not None else None,
                previous_acceleration_mps2=previous.tolist() if previous is not None else None,
                qp_diagnostics=getattr(exc,'diagnostics',None),plan=plan)
            failure_path=Path(evidence_path).with_name('preview_failure.json')
            failure_path.write_text(json.dumps(failure,indent=2,allow_nan=False))
            exc.preview_failure_path=str(failure_path)
        raise
    finally:
        evidence.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',required=True);parser.add_argument('--output',required=True);parser.add_argument('--starts');parser.add_argument('--captured',action='store_true');parser.add_argument('--acquire-only',action='store_true');parser.add_argument('--voltages',type=float,nargs=3,default=[4.,3.8,3.7],metavar=('CF2','CF3','CF4'));args=parser.parse_args()
    c=load(args.config)
    if args.captured:args.starts=str(path_of(c,'starts_file'))
    if args.starts:
        import yaml
        s=yaml.safe_load(Path(args.starts).read_text());ground=[s['positions'][n] for n in NAMES];yaws=[s['yaw_rad'][n] for n in NAMES]
    else:
        z=c['arena']['floor_z_m'];ground=[[-.05,.76,z],[-.78,0,z],[-.05,-.76,z]];yaws=[2.5,-1.8,.9]
    plan=make_plan(c,ground,yaws);out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    (out/'plan.json').write_text(json.dumps(plan,indent=2))
    (out/'effective_profile.json').write_text(json.dumps(c,indent=2))
    try:
        report,rows=run(c,plan,voltages=args.voltages,trace=True,acquire_only=args.acquire_only,evidence_path=out/'hocbf_rows.csv')
    except (RuntimeError,ValueError) as exc:
        failure=dict(status='FAIL',error=type(exc).__name__+': '+str(exc),
                     failure_report=getattr(exc,'preview_failure_path',None))
        (out/'offline_report.json').write_text(json.dumps(failure,indent=2));print(json.dumps(failure,indent=2))
        raise SystemExit(1)
    (out/'offline_report.json').write_text(json.dumps(report,indent=2))
    keys=('time','measured_position','reference_position','reference_velocity','safe_acceleration','position_redcho','motion_derivatives','yaw');np.savez_compressed(out/'offline_trace.npz',**{k:np.array([r[i] for r in rows]) for i,k in enumerate(keys)})
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
