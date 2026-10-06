"""Summarize measured MATLAB replays; no simulation is performed here."""
from pathlib import Path
import argparse
import csv
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description='Historical instrumented-audit post-processing only; not an unchanged-baseline runner.')
parser.add_argument('--input', type=Path, default=ROOT/'results/runs/baseline_legacy_20261005')
parser.add_argument('--output', type=Path, default=ROOT/'results/runs/historical_audit_summary')
args = parser.parse_args()
P = args.input
OUT = args.output
OUT.mkdir(parents=True, exist_ok=True)
def read(name): return json.loads((P/name).read_text(encoding='utf-8'))
def sequence(x): return x if isinstance(x,list) else [x]
planar=read('planar_summary.json')
window=read('window_summary.json')
frozen=read('window_frozen_summary.json')
slide=read('sliding_summary.json')
analysis=read('independent_analysis.json')
frames=sequence(slide['frames'])
rows=[]
for f in frames:
    trace=sequence(f['trace'])[0]; hom=sequence(trace['homotopyTrace']); c=f['counts']
    rows.append(dict(frame=f['index'],seconds=f['seconds'],iterations=trace['totalIterations'],
        seed_iterations=f['seedInfo'].get('iterations',0),map_calls=c[0],map_misses=c[1],ivps=c[2],
        ode_segments=c[3],rhs=c[4],mechanics_seconds=c[5],failed_maps=c[6],
        homotopy_iterations=sum(t['iterations'] for t in hom),polish_iterations=trace['polish']['iterations'],
        homotopy_seconds=sum(t['seconds'] for t in hom),polish_seconds=trace['polish']['seconds'],
        capped_stages=sum(t['exitflag']==0 for t in hom),final_exit=trace['exitflag']))
with (OUT/'sliding_frame_metrics.csv').open('w',newline='',encoding='utf-8') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
t=np.array([r['seconds'] for r in rows]); pt=np.array([r['elapsedSeconds'] for r in planar])
diagnostics=sequence(analysis['sliding'])
summary=dict(sliding=dict(total_seconds=slide['seconds'],mean_seconds=float(t.mean()),median_seconds=float(np.median(t)),
    p95_seconds=float(np.percentile(t,95)),max_seconds=float(t.max()),
    total_ivps=sum(r['ivps'] for r in rows),total_map_misses=sum(r['map_misses'] for r in rows),
    total_iterations=sum(r['iterations'] for r in rows),mechanics_fraction=sum(r['mechanics_seconds'] for r in rows)/t.sum(),
    condition_range=[min(r['forceCondition'] for r in diagnostics),max(r['forceCondition'] for r in diagnostics)],
    errors={k:slide[k] for k in ['contactRmseN','tipRmseN','totalRmseN']}),
    planar=dict(total_seconds=float(pt.sum()),mean_seconds=float(pt.mean()),median_seconds=float(np.median(pt)),
    range_seconds=[float(pt.min()),float(pt.max())],total_ivps=sum(x['counts'][7] for x in planar),
    contact_rmse=float(np.sqrt(np.mean(np.square([x['contactErrorN'] for x in planar])))),
    tip_rmse=float(np.sqrt(np.mean(np.square([x['tipErrorN'] for x in planar])))),
    total_rmse=float(np.sqrt(np.mean(np.square([x['totalErrorN'] for x in planar]))))),
    window=dict(seconds=window['elapsedSeconds'],ivps=window['solver']['mechanicalEvaluations']),
    frozen_window=dict(seconds=frozen['elapsedSeconds'],ivps=frozen['solver']['mechanicalEvaluations']))
if (P/'sliding_frozen_summary.json').exists():
    sf=read('sliding_frozen_summary.json')
    summary['frozen_sliding']=dict(seconds=sf['seconds'],frame_seconds=sf['frameSeconds'],ivps=sf['counts'][2],
        contact_rmse=sf['contactRmseN'],tip_rmse=sf['tipRmseN'])
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
x=np.arange(1,len(rows)+1)
ax[0,0].bar(x,[r['mechanics_seconds'] for r in rows],label='Nested mechanics')
ax[0,0].bar(x,[r['seconds']-r['mechanics_seconds'] for r in rows],bottom=[r['mechanics_seconds'] for r in rows],label='Other work')
ax[0,0].set(title='Sliding clean: measured frame time',xlabel='Frame',ylabel='Seconds');ax[0,0].legend()
ax[0,1].semilogy(x,[r['ivps'] for r in rows],'o-',label='Whole-rod IVPs')
ax[0,1].semilogy(x,[r['map_misses'] for r in rows],'s-',label='Uncached equilibrium solves')
ax[0,1].semilogy(x,[r['iterations'] for r in rows],'^-',label='Outer iterations')
ax[0,1].set(title='Sliding clean: nested evaluation count',xlabel='Frame',ylabel='Count, logarithmic scale');ax[0,1].legend(loc='center right',bbox_to_anchor=(0.98,0.36))
ax[1,0].bar(['Free geometry','Frozen geometry'],[window['elapsedSeconds'],frozen['elapsedSeconds']],color=['#4569aa','#42957a'])
ax[1,0].set(title='3-D two-contact window: two frames',ylabel='Seconds including covariance')
ax[1,0].text(.5,.88,'Frozen values supplied by completed baseline',ha='center',transform=ax[1,0].transAxes,fontsize=9)
v=np.array(diagnostics[len(diagnostics)//2]['forceSingularValues']);v=v.ravel()
vp=np.array(analysis['planarSensitivity']['forceSingularValues']).ravel()
ax[1,1].semilogy(np.arange(1,len(v)+1),v,'o-',label='Sliding: 6 force components')
ax[1,1].semilogy(np.arange(1,len(vp)+1),vp,'s-',label='Planar channel: 4 components')
ax[1,1].set(title='Conditional force-to-curvature sensitivity',xlabel='Singular-value index',ylabel='1 / (mm N)');ax[1,1].legend()
fig.savefig(OUT/'bottleneck_summary.png',dpi=180)
print(json.dumps(summary,indent=2))
