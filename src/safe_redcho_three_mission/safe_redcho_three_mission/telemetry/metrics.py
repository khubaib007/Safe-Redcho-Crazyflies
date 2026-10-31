"""Three-agent convergence for identical session, cycle, profile and stage."""
import numpy as np
from .configuration import NAMES


def convergence(packets,names,thresholds):
    if names!=NAMES or set(packets)!=set(names): raise ValueError('Need cf2/cf3/cf4 exact-cycle packets')
    identities={(p['session'],p['seq'],p['config_hash'],p['stage'],p['flight_active'],p['streaming']) for p in packets.values()}
    if len(identities)!=1: raise ValueError('Mixed session, cycle, profile, stage or flight flags')
    for n,p in packets.items():
        if p['name']!=n or p.get('error'): raise ValueError('Wrong identity or failed controller result')
    output={}
    for bank,variables in [('velocity_redcho',('velocity_mps','acceleration_mps2','jerk_mps3')),
                           ('position_redcho',('displacement_m','velocity_mps','acceleration_mps2'))]:
        rows={}
        for variable in variables:
            inputs=np.asarray([[packets[n]['data'][bank]['input'][variable][a] for a in ('x','y','z')] for n in names],float)
            estimates=np.asarray([[packets[n]['data'][bank]['output'][variable][a] for a in ('x','y','z')] for n in names],float)
            if inputs.shape!=(3,3) or estimates.shape!=(3,3) or not np.isfinite(inputs).all() or not np.isfinite(estimates).all():
                raise ValueError('Nonfinite or invalid bank values')
            average=inputs.mean(axis=0)
            disagreement=max(float(np.linalg.norm(estimates[i]-estimates[j])) for i in range(3) for j in range(i))
            ring=max(float(np.linalg.norm(estimates[i]-estimates[(i+1)%3])) for i in range(3))
            tracking=float(np.max(np.linalg.norm(estimates-average,axis=1)))
            threshold=float(thresholds[bank][variable])
            rows[variable]={'average_input':dict(zip(('x','y','z'),average.tolist())),
                'max_pair_disagreement':disagreement,'max_ring_disagreement':ring,
                'max_average_tracking_error':tracking,
                'average_preservation_error':float(np.linalg.norm(estimates.mean(axis=0)-average)),
                'threshold':threshold,'within_threshold':disagreement<=threshold and tracking<=threshold}
        output[bank]=rows
    return {'convergence':output,'convergence_scope':{'agents':3,
        'position_quantity':'absolute_world_center_and_height',
        'flight_active':packets[names[0]]['flight_active'],'streaming':packets[names[0]]['streaming']}}
