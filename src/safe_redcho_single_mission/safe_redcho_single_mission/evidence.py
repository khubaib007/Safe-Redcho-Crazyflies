"""Constraint-specific evidence; acceleration correction alone proves nothing."""
import csv
import numpy as np

class Evidence:
    fields = ('time_s','agent','constraint','h','psi1','nominal_residual',
              'bounded_free_residual','filtered_residual','binding')
    def __init__(self, path=None):
        self.rows = {}; self.file = open(path,'w',newline='') if path else None
        self.writer = csv.DictWriter(self.file,fieldnames=self.fields) if self.file else None
        if self.writer:self.writer.writeheader()
    def add(self, time_s, names, results, tolerance):
        for name,result in zip(names,results):
            qp = result['diagnostics']['qp']
            for label,row in qp['rows'].items():
                key = name+'/'+label
                stat = self.rows.setdefault(key,dict(samples=0,binding_samples=0,nominal_unsafe_samples=0,
                    bounded_free_unsafe_samples=0,minimum_filtered_residual=None,minimum_h=None,minimum_psi1=None))
                A = np.asarray(row['A']); b = row['b']
                nominal = float(b-A@qp['nominal_acceleration_mps2'])
                free = float(b-A@qp['bounded_weighted_minimizer_mps2'])
                residual = row['residual_b_minus_Au']
                stat['samples']+=1;stat['binding_samples']+=int(row['binding'])
                stat['nominal_unsafe_samples']+=int(nominal < -tolerance)
                stat['bounded_free_unsafe_samples']+=int(free < -tolerance)
                for field,value in [('minimum_filtered_residual',residual),('minimum_h',row['h']),('minimum_psi1',row['psi1'])]:
                    stat[field] = value if stat[field] is None else min(stat[field],value)
                if self.writer:self.writer.writerow(dict(zip(self.fields,(time_s,name,label,row['h'],row['psi1'],nominal,free,residual,row['binding']))))
    def close(self):
        if self.file:self.file.close();self.file=None
