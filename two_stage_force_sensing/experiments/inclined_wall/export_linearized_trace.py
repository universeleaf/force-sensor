"""Export actual nonlinear outer-iteration checks from saved MATLAB traces."""
from pathlib import Path
import csv,sys
import h5py,numpy as np
ROOT=Path(__file__).resolve().parents[2]
stamp=sys.argv[1] if len(sys.argv)>1 else '20261011_v2'
pub=ROOT/'results/published'/f'linearized_modes_{stamp}';pub.mkdir(parents=True,exist_ok=True)
rows=[]
keys=['iteration','radius','step','innerIterations','innerExitflag','innerViolation','trueViolation','predictedReduction','actualReduction','ratio','penalty','modelFbgRmsError','accepted','seconds','trueObjective','trueFbgRms','predictedObjective']
for case in ['insertion','rotation']:
 path=ROOT/'results/runs'/f'inclined_{case}_linearized_{stamp}'/'comparison.mat'
 with h5py.File(path) as f:
  refs=f['runs'][()]
  for k in range(refs.shape[0]):
   for j in range(refs.shape[1]):
    if case=='rotation' and k==0:continue
    sol=f[refs[k,j]]['mechanics'];attempts=sol['attempts'][()].ravel()
    for a,ref in enumerate(attempts):
     branch=f[ref];mode=branch['mode'][()];mode_label='free' if branch['mode'].attrs.get('MATLAB_empty',False) else ','.join(str(int(x)) for x in mode.ravel())
     trace=branch['outerTrace'];n=int(branch['outerIterations'][0,0]);accepted=0
     for t in range(n):
      row=dict(scenario=case,frame=k+1,method=j+1,mu=.3 if j==0 else 0,attempt=a+1,mode=mode_label)
      for key in keys:
       data=trace[key][()]
       row[key]=float(f[data.ravel()[t]][()].ravel()[0]) if data.dtype==h5py.ref_dtype else float(data.ravel()[t])
      rows.append(row)
with (pub/'outer_iterations.csv').open('w',newline='') as out:
 writer=csv.DictWriter(out,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print('Exported',len(rows),'outer iterations')
