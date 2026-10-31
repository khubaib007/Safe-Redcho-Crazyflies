from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from safe_redcho_three_mission.liveplots.configuration import fingerprint


def cfg():
    p=dict(names=['cf2','cf3','cf4'],command_interface='position',frame_id='world',
        mission=dict(center_start_xy=[-.325,0.],offsets_xyz={n:[.475,y,0.] for n,y in zip(['cf2','cf3','cf4'],[.95,0.,-.95])},hover_height_above_floor_m=.5),
        arena=dict(x_limits=[-1.5,1.5],y_limits=[-1.5,1.5],floor_z_m=.0298,ceiling_z_m=1.3,
                   body_radius_m=.05,wall_surface_clearance_m=.15,ceiling_clearance_m=.2),
        safety=dict(tracking_tube_m=.14,mocap_position_uncertainty_m=.02,pair_reserve_m=.09,
                    minimum_physical_xy_separation_m=.5,measured_abort_xy_separation_m=.65,
                    measured_maximum_speed_mps=.4,mocap_maximum_age_s=.08,
                    maximum_estimator_age_s=.3,mocap_velocity_window_s=.16,watchdog_timeout_s=.2),
        battery=dict(status_maximum_age_s=3.,minimum_ground_voltage_v=3.7,land_voltage_v=3.0,
                     full_speed_voltage_v=3.5,minimum_speed_scale=.75),
        control=dict(maximum_publish_gap_s=.08,dt_s=.02,qp_tolerance=1e-6))
    return dict(profile=p,names=p['names'],config_hash=fingerprint(p),refresh_hz=5.,history_s=90.,
                max_points_per_series=6000,command_max_age_s=2.,alignment_max_skew_s=.04,
                output_root='/tmp/redcho-liveplots-test',default_axis='x')


def packet(c,name,seq=1,stamp=1_000_000_000_000,stage='ALIGN',active=True):
    v=lambda x:dict(x=x,y=0.,z=0.)
    i=c['names'].index(name)
    data=dict(governor=dict(speed_scale=.9,input_voltage_v=3.6,input_unscaled_velocity_mps=v(.1),output_velocity_mps=v(.09)),
              svf=dict(output_velocity_mps=v(.08),output_acceleration_mps2=v(.01),output_jerk_mps3=v(.001)),
              nominal=dict(desired_position_m=[.15,0.,.5298],virtual_position_m=[.15,0.,.5298],
                           desired_velocity_mps=[.08,0.,0.],virtual_velocity_mps=[.07,0.,0.],desired_acceleration_mps2=[.01,0.,0.]),
              qp=dict(nominal_acceleration_mps2=[.03,0.,0.],filtered_acceleration_mps2=[.02,0.,0.],
                      intervention_norm_mps2=.01,inequality_solver_invoked=False,hocbf_rows_binding=False,
                      solver_success=True,solver_wall_s=.0005,max_violation_mps2=0.,
                      rows=dict(pair_cf3=dict(h=.25,psi1=.8,residual_b_minus_Au=.7),
                                upper_0=dict(h=.5,psi1=2.,residual_b_minus_Au=.8))))
    for bank,fields in [('velocity_redcho',('velocity_mps','acceleration_mps2','jerk_mps3')),
                        ('position_redcho',('displacement_m','velocity_mps','acceleration_mps2'))]:
        data[bank]={'input':{f:v(float(i)) for f in fields},'output':{f:v(1.+i*.01) for f in fields}}
    return dict(schema_version=1,name=name,session='session-1',seq=seq,stage=stage,flight_active=active,
                streaming=active,config_hash=c['config_hash'],producer_ros_ns=stamp,
                producer_monotonic_ns=stamp,error=None,emitter_dropped_before=0,data=data)
