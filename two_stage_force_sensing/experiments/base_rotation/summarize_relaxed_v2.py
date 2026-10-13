"""Summarize every relaxed-rotation result, including unsuccessful solves."""
from pathlib import Path
import csv,json,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'results/published/base_rotation_relaxed_v2_comparison_20261007'
OUT.mkdir(parents=True,exist_ok=True)
def read(path):
    with path.open(newline='',encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def vec(rows,key):return np.array([float(r[key]) for r in rows])
configs=[];summaries=[]
for step,tag in [(3.75,'375deg'),(1,'1deg')]:
    folder=ROOT/f'results/published/base_rotation_relaxed_v2_{tag}_20261007'
    for solver in ['ipopt','fmincon']:
        rows=read(folder/f'{solver}_frames.csv'); configs.append((step,solver,rows))
        s={'stepDeg':step,'solver':solver,'frames':len(rows),'nativeSuccess':int(vec(rows,'converged').sum()),
          'reportedOptimalityAt1e8':int(np.sum(vec(rows,'stationarity')<=1e-8)),
          'statusCounts':dict(collections.Counter(r['reason'] for r in rows)),
          'flagCounts':dict(collections.Counter(f for r in rows for f in r['flags'].split(';') if f)),
          'failedAngles':[float(r['angleDeg']) for r in rows if r['converged']=='0']}
        for key in ['tipErrorN','contactErrorN','totalErrorN','shapeRmsMm']:
            x=vec(rows,key);s[key+'_rms']=float(np.sqrt(np.mean(x*x)));s[key+'_max']=float(x.max())
        for key in ['seconds','mechanicsSeconds','geometrySeconds','iterations','ivps','rhsEvaluations']:
            x=vec(rows,key);s[key+'_mean']=float(x.mean());s[key+'_median']=float(np.median(x));s[key+'_max']=float(x.max());s[key+'_sum']=float(x.sum())
        for key in ['projectionN','normalProductNmm','frictionDualityGapNmm','coneExcessN','frictionWorkNmm','tipMomentNmm','penetrationMm','truthPlanePenetrationMm','tangency','relaxedViolation','normalErrorDeg']:
            s[key+'_max']=float(vec(rows,key).max())
        s['arcErrorMm_maxabs']=float(np.abs(vec(rows,'arcErrorMm')).max())
        s['stationarity_median']=float(np.median(vec(rows,'stationarity')))
        s['lastFrame']=rows[-1];s['relaxedFeasibleCount']=int(vec(rows,'relaxedFeasible').sum());s['exactLawFlagCounts']=dict(collections.Counter(f for r in rows for f in r['exactLawFlags'].split(';') if f));s['curvatureRmsMedian']=float(np.median(vec(rows,'curvatureRmsPerMm')));summaries.append(s)
(OUT/'summary.json').write_text(json.dumps(summaries,indent=2))
fields=['stepDeg','solver','frames','nativeSuccess','reportedOptimalityAt1e8','tipErrorN_rms','contactErrorN_rms','shapeRmsMm_rms','seconds_mean','seconds_sum','iterations_mean','ivps_mean','projectionN_max','relaxedViolation_max']
with (OUT/'summary.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fields,extrasaction='ignore');w.writeheader();w.writerows(summaries)
plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for step,solver,rows in configs:
    a=vec(rows,'angleDeg');label=f'{solver}, {step:g}°';style='-' if step==3.75 else '--';color='#0072B2' if solver=='ipopt' else '#D55E00'
    for ax,key in zip(axs.flat,['tipErrorN','shapeRmsMm','seconds','iterations']):
        ax.plot(a,vec(rows,key),style,color=color,label=label,linewidth=1.5,marker='.' if step==3.75 else None)
        failed=vec(rows,'converged')==0;ax.scatter(a[failed],vec(rows,key)[failed],marker='x',s=24,color=color)
for ax,title in zip(axs.flat,['Tip-force error (N)','Centerline RMS error (mm)','Total sensing time / frame (s)','Solver iterations / frame']):
    ax.set(title=title,xlabel='Base rotation (degrees)');ax.set_yscale('log')
axs[0,0].legend(fontsize=9);fig.suptitle('Revised relaxation: no force priors, σ = 10⁻⁴ /mm; × = unsuccessful native status')
fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,(step,solver,rows) in zip(axs.flat,configs):
    tag='375deg' if step==3.75 else '1deg';tr=read(ROOT/f'results/published/base_rotation_relaxed_v2_{tag}_20261007/{solver}_iterations.csv')
    for angle in ([3.75,22.5,41.25] if step==3.75 else [1,22,41]):
        frame=next(int(r['frame']) for r in rows if float(r['angleDeg'])==angle);q=[r for r in tr if int(r['frame'])==frame]
        ax.plot(vec(q,'iteration'),np.maximum(vec(q,'objective'),1e-12),label=f'{angle:g}°')
    ax.set(title=f'{solver}, {step:g}° increments',xlabel='Iteration',ylabel='Unscaled MAP objective');ax.set_yscale('log');ax.legend()
fig.suptitle('Convergence paths: terminal objective can stabilize before optimality is certified')
fig.savefig(OUT/'convergence.png',dpi=160);plt.close(fig)
print(json.dumps([{k:s[k] for k in fields} for s in summaries],indent=2))
