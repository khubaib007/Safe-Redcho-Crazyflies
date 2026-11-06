"""Chart definitions. Different physical units always use separate axes."""
from .configuration import NAMES


def curve(key, label, agent=None, dash=False, xy=None):
    return dict(key=key, label=label, agent=agent, dash=dash, xy=xy)


def agents(key, label='', dash=False):
    return [curve(n+'.'+key, n+(' '+label if label else ''), n, dash) for n in NAMES]


def panel(title, unit, lines, thresholds=(), xy=False):
    return dict(title=title, unit=unit, lines=lines, thresholds=list(thresholds), xy=xy)


def specifications(group):
    if group == 'flight':
        return [
            panel('World Z: mocap / host-published setpoint', 'm', agents('mocap.z','mocap')+agents('command.z','host command',True)),
            panel('XY paths: mocap / host-published setpoint', 'm',
                  [curve(n+'.mocap.y',n+' mocap',n,xy=n+'.mocap.x') for n in NAMES]+
                  [curve(n+'.command.y',n+' host command',n,True,xy=n+'.command.x') for n in NAMES], xy=True),
            panel('Time-aligned mocap vs host command error', 'm', agents('tracking.norm'), [('tracking tube','safety.tracking_tube_m')]),
            panel('Mocap-derived 3D speed (window fit)', 'm/s', agents('mocap.speed'), [('measured speed guard','safety.measured_maximum_speed_mps')]),
            panel('Measured XY pair distances (same mocap frame)', 'm',
                  [curve('pair.cf2_cf3','cf2–cf3','cf2'),curve('pair.cf2_cf4','cf2–cf4','cf3'),curve('pair.cf3_cf4','cf3–cf4','cf4')],
                  [('physical minimum','safety.minimum_physical_xy_separation_m'),('measured abort','safety.measured_abort_xy_separation_m')]),
            panel('World Z: onboard estimator / mocap', 'm', agents('onboard.z','onboard')+agents('mocap.z','mocap',True)),
            *[panel('Mocap '+a, 'rad', agents('mocap.'+a)) for a in ('roll','pitch','yaw')],
            panel('Optional sampled onboard field: {field}', 'raw / configured', agents('firmware.{field}')),
        ]
    if group == 'battery':
        return [panel('Measured battery voltage', 'V', agents('voltage'),
                      [('ground minimum','battery.minimum_ground_voltage_v'),('land threshold','battery.land_voltage_v'),
                       ('full-speed voltage','battery.full_speed_voltage_v')]),
                panel('Governor speed scale (algorithm telemetry)', '1', agents('scale'),
                      [('minimum scale','battery.minimum_speed_scale')]),
                panel('Voltage actually supplied to governor', 'V', agents('governor_voltage'))]
    if group == 'timing':
        return [
            panel('Mocap source age', 'ms', agents('mocap_age_ms'), [('mocap age guard','safety.mocap_maximum_age_s',1000)]),
            panel('Onboard pose source age', 'ms', agents('onboard_age_ms'), [('estimator age guard','safety.maximum_estimator_age_s',1000)]),
            panel('Command age seen through CSV (includes file-reader lag)', 'ms', agents('command_age_ms')),
            panel('Original host publication interval (CSV timestamps)', 'ms', agents('command_gap_ms'), [('publication-gap guard','control.maximum_publish_gap_s',1000)]),
            panel('Observed /frame DDS receipt interval; NOT CPU cycle time', 'ms', [curve('fleet.frame_interval_ms','frame receipt interval')], [('nominal dt','control.dt_s',1000)]),
            panel('Mission heartbeat receipt age', 'ms', [curve('fleet.heartbeat_age_ms','heartbeat receipt age')], [('watchdog timeout','safety.watchdog_timeout_s',1000)]),
            panel('Agent diagnostic emitter drop counters', 'count', agents('emitter_drops')),
        ]
    if group == 'reference':
        return [panel('Governor input: unscaled request ({axis})', 'm/s', agents('raw.{axis}')),
                panel('Governor output = SVF input ({axis})', 'm/s', agents('governed.{axis}')),
                panel('SVF output velocity ({axis})', 'm/s', agents('svf_v.{axis}')),
                panel('SVF output acceleration ({axis})', 'm/s^2', agents('svf_a.{axis}')),
                panel('SVF output jerk ({axis})', 'm/s^3', agents('svf_j.{axis}'))]
    if group == 'position_redcho':
        fields = [
                  ('position_m','m'),('velocity_mps','m/s'),('acceleration_mps2','m/s^2')]
        plots = []
        for field, unit in fields:
            quantity = 'center position' if field=='position_m' else field.split('_')[0]
            plots.append(panel(quantity+' input / output ({axis})'+(' — absolute world coordinates' if field=='position_m' else ''), unit,
                         agents('input.'+field+'.{axis}','input',True)+agents('output.'+field+'.{axis}','output')+
                         [curve('fleet.average.'+field+'.{axis}','same-cycle mean input')]))
            plots.append(panel(quantity+': all-axis convergence errors', unit,
                         [curve('fleet.disagreement.'+field,'max pair disagreement','cf2'),
                          curve('fleet.average_error.'+field,'max error to mean input','cf4')]))
        return plots
    if group == 'control':
        return [panel('Passing guidance active / blend', '0 to 1', agents('avoidance_active','active')+agents('avoidance_blend','blend',True)),
                panel('Common motion progress scale', '0 to 1', agents('motion_progress')),
                panel('Nominal / QP-filtered acceleration ({axis})', 'm/s^2', agents('nominal.{axis}','nominal',True)+agents('filtered.{axis}','QP-filtered')),
                panel('QP correction norm — not proof of obstacle activation', 'm/s^2', agents('correction')),
                panel('Nominal-controller position input / target ({axis})', 'm', agents('virtual.{axis}','virtual input',True)+agents('target.{axis}','target')),
                panel('Nominal-controller velocity input / desired ({axis})', 'm/s', agents('virtual_v.{axis}','virtual input',True)+agents('desired_v.{axis}','desired')),
                panel('Feedforward acceleration supplied to nominal controller ({axis})', 'm/s^2', agents('feedforward.{axis}')),
                panel('Optional sampled onboard field: {field} (not packet acknowledgement)', 'raw / configured', agents('firmware.{field}'))]
    if group == 'hocbf':
        return [panel('Minimum normalized HOCBF row residual b − A·u', 'm/s^2', agents('residual'), [('feasible boundary',0.)]),
                panel('Minimum pair/obstacle h (virtual references)', 'm^2', agents('h_squared'), [('boundary',0.)]),
                panel('Minimum wall/height h (virtual references)', 'm', agents('h_wall'), [('boundary',0.)]),
                panel('Minimum pair/obstacle ψ₁', 'm^2/s', agents('psi_squared')),
                panel('Minimum wall/height ψ₁', 'm/s', agents('psi_wall')),
                panel('Maximum QP constraint violation', 'm/s^2', agents('violation'), [('QP tolerance','control.qp_tolerance')]),
                panel('Inequality solver invoked / row binding', '0 or 1', agents('solver_invoked','solver',True)+agents('binding','binding')),
                panel('Solver computation time', 'ms', agents('solve_ms')),
                panel('Solver success flag', '0 or 1', agents('success'))]
    raise ValueError('Unknown plot group '+group)
