"""Same-cycle convergence; no assembly of unrelated latest samples."""
import numpy as np


def convergence(packets, names, thresholds):
    if set(packets) != set(names): raise ValueError('Need all six exact-cycle packets')
    identities = {(p['session'], p['seq'], p['config_hash']) for p in packets.values()}
    if len(identities) != 1: raise ValueError('Mixed session/sequence/configuration')
    output = {}
    for bank, variables in [('velocity_redcho', ('velocity_mps', 'acceleration_mps2', 'jerk_mps3')),
                            ('position_redcho', ('displacement_m', 'velocity_mps', 'acceleration_mps2'))]:
        rows = {}
        for var in variables:
            inputs = np.asarray([[packets[n]['data'][bank]['input'][var][a] for a in ('x','y','z')] for n in names])
            estimates = np.asarray([[packets[n]['data'][bank]['output'][var][a] for a in ('x','y','z')] for n in names])
            if not np.isfinite(inputs).all() or not np.isfinite(estimates).all(): raise ValueError('Nonfinite convergence input')
            average = inputs.mean(axis=0)
            disagreement = max(float(np.linalg.norm(estimates[i]-estimates[j])) for i in range(6) for j in range(i))
            tracking = float(np.max(np.linalg.norm(estimates-average, axis=1)))
            ring = max(float(np.linalg.norm(estimates[i]-estimates[(i+1)%6])) for i in range(6))
            threshold = float(thresholds[bank][var])
            rows[var] = {'average_input': dict(zip(('x','y','z'), average.tolist())),
                         'max_pair_disagreement': disagreement, 'max_ring_disagreement': ring,
                         'max_average_tracking_error': tracking,
                         'average_preservation_error': float(np.linalg.norm(estimates.mean(axis=0)-average)),
                         'threshold': threshold, 'within_threshold': disagreement <= threshold and tracking <= threshold}
        output[bank] = rows
    return {'convergence': output}
