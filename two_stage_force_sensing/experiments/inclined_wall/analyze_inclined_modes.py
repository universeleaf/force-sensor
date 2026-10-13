"""Summarize the isolated explicit-mode inclined-wall benchmark; no solver changes."""
from pathlib import Path
import csv, json, hashlib
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
PUB = ROOT / 'results/published/inclined_wall_modes_20261010'
OLD = ROOT / 'results/runs/inclined_wall_bidirectional_20261008/inclined_wall'

def read(path):
    with path.open(newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))
def vals(rows, key):
    return np.array([float(r[key]) for r in rows])
def rms(a):
    return float(np.sqrt(np.mean(np.asarray(a)**2)))
def stats(rows, new=True):
    feasible = vals(rows, 'modeFeasible' if new else 'relaxedFeasible').astype(bool)
    native = vals(rows, 'converged').astype(bool)
    return dict(frames=len(rows), native_positive=int(sum(native)), feasible=int(sum(feasible)),
        success=int(sum(native & feasible)), exit1=sum(r['reason']=='fmincon-exit-1' for r in rows),
        warning_free=sum(not r['flags'] for r in rows),
        total_seconds=float(sum(vals(rows,'seconds'))), mean_seconds=float(np.mean(vals(rows,'seconds'))),
        median_seconds=float(np.median(vals(rows,'seconds'))), max_seconds=float(max(vals(rows,'seconds'))),
        mean_geometry_seconds=float(np.mean(vals(rows,'geometrySeconds'))),
        mean_mechanics_seconds=float(np.mean(vals(rows,'mechanicsSeconds'))),
        total_iterations=int(sum(vals(rows,'iterations'))), mean_iterations=float(np.mean(vals(rows,'iterations'))),
        total_ivps=int(sum(vals(rows,'ivps'))), mean_ivps=float(np.mean(vals(rows,'ivps'))),
        tip_rmse_N=rms(vals(rows,'tipErrorN')), tip_max_N=float(max(vals(rows,'tipErrorN'))),
        contact_rmse_N=rms(vals(rows,'contactErrorN')), contact_max_N=float(max(vals(rows,'contactErrorN'))),
        shape_rms_mm=rms(vals(rows,'shapeRmsMm')), shape_max_mm=float(max(vals(rows,'shapeRmsMm'))),
        max_violation=float(max(vals(rows,'modeViolation' if new else 'relaxedViolation'))),
        max_projection_N=float(max(vals(rows,'projectionN'))), max_penetration_mm=float(max(vals(rows,'penetrationMm'))),
        max_tip_moment_Nmm=float(max(vals(rows,'tipMomentNmm'))), max_positive_work_Nmm=float(max(0.,max(vals(rows,'workNmm')))),
        max_abs_y_tip_N=float(max(abs(vals(rows,'tipFy')))),
        reasons=dict(Counter(r['reason'] for r in rows)),
        flags=dict(Counter(s for r in rows for s in r['flags'].split(';') if s)))

rows=read(PUB/'estimates.csv'); branches=read(PUB/'branch_attempts.csv'); old=read(OLD/'estimates_unreduced_attempt.csv'); truth=read(OLD/'truth_frames.csv')
assert len(rows)==82, f'Full trajectory unfinished: {len(rows)}/82 rows'
summary={}
for mu in [.3,0.]:
    rr=[r for r in rows if float(r['mu'])==mu]
    assert [int(r['frame']) for r in rr]==list(range(1,42))
    assert all(int(r['objectiveTerms'])==48 for r in rr)
    bb=[r for r in branches if float(r['mu'])==mu]
    for r in rr:
        b=[x for x in bb if x['frame']==r['frame']]
        assert len(b)==int(r['assignments'])
        assert sum(int(x['ivps']) for x in b)==int(r['ivps'])
        assert sum(int(x['iterations']) for x in b)==int(r['iterations'])
        assert b[0]['modes']==r['previousMode']
    s=stats(rr); s['contact_frames']=stats([r for r in rr if 4<=int(r['frame'])<=38])
    s['mean_selected_seconds']=float(np.mean(vals(rr,'selectedSeconds')))
    s['mean_selected_iterations']=float(np.mean(vals(rr,'selectedIterations')))
    s['mean_selected_ivps']=float(np.mean(vals(rr,'selectedIvps')))
    s['failed_ivps']=int(sum(vals(rr,'failedIVPs')))
    s['selected_modes']=dict(Counter(r['mode'] for r in rr))
    s['non_success_frames']=[int(r['frame']) for r in rr if not int(r['solverSuccess'])]
    s['flagged_frames']=[int(r['frame']) for r in rr if r['flags']]
    s['branches']={m:dict(attempts=len(b),seconds=float(sum(vals(b,'seconds'))),
        iterations=int(sum(vals(b,'iterations'))), ivps=int(sum(vals(b,'ivps'))),
        selected=sum(int(x['selected']) for x in b), feasible=sum(int(x['feasible']) for x in b),
        reasons=dict(Counter(x['reason'] for x in b)))
        for m in sorted(set(x['modes'] for x in bb)) if (b:=[x for x in bb if x['modes']==m])}
    summary[f'modes_mu_{mu:g}']=s
baseline=[r for r in old if float(r['mu'])==.3]
summary['relaxed_mu_0.3']=stats(baseline,False)
summary['relaxed_mu_0.3']['contact_frames']=stats([r for r in baseline if 4<=int(r['frame'])<=38],False)
summary['baseline_mu_0_note']='Only 3 pre-contact frames returned; original solver crashed on frame 4. No full-trajectory accuracy or runtime comparison is available.'
# Both methods must use identical stage-1 estimates to the prior replay.
for r in rows:
    ref=baseline[int(r['frame'])-1]
    aa=float(r['estimatedArcMm']); ab=float(ref['estimatedArcMm'])
    assert (np.isnan(aa) and np.isnan(ab)) or abs(aa-ab)<1e-9
summary['stage1_matches_prior']=True
summary['stage1_max_errors']={k:float(np.nanmax(np.abs(vals(baseline,k)))) for k in ['arcErrorMm','normalErrorDeg','stage1ShapeRmsMm']}
summary['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [OLD/'truth.mat',ROOT/'matlab/+tsfs/mode_branch.m',ROOT/'matlab/+tsfs/mode_solve.m',ROOT/'experiments/inclined_wall/run_inclined_modes.m']}
(PUB/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

plt.rcParams.update({'font.size':10})
fig,axs=plt.subplots(3,2,figsize=(12,10),sharex=True)
series=[(baseline,'Relaxed mu=0.3','#999999'),([r for r in rows if float(r['mu'])==.3],'Integer modes mu=0.3','#1769aa'),([r for r in rows if float(r['mu'])==0.],'Integer modes mu=0','#df761d')]
for ax,(key,label) in zip(axs.flat,[('tipErrorN','Tip-force error (N)'),('contactErrorN','Contact-force error (N)'),('shapeRmsMm','Shape RMS error (mm)'),('seconds','Total estimation time (s)'),('iterations','Total SQP iterations'),('ivps','Total IVP evaluations')]):
    for rr,name,color in series:
        ax.plot(vals(rr,'frame'),vals(rr,key) if key=='iterations' else np.maximum(vals(rr,key),1e-10),label=name,color=color,lw=1.6)
    ax.set_yscale('symlog',linthresh=1) if key=='iterations' else ax.set_yscale('log')
    ax.axvline(21,color='black',ls=':',alpha=.45);ax.set_ylabel(label);ax.grid(alpha=.2)
axs[0,0].legend(fontsize=8);axs[2,0].set_xlabel('Frame (insertion 1-21; withdrawal 22-41)');axs[2,1].set_xlabel('Frame')
fig.suptitle('Full trajectory: exact integer contact modes, unchanged nonlinear rod\nAll attempted modes included in cost; noiseless reference, true mu=0.3')
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(PUB/'trajectory_comparison.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(3,1,figsize=(11,9),sharex=True)
axs[0].plot(vals(truth,'frame'),vals(truth,'normalN'),'k--',label='Reference');axs[1].plot(vals(truth,'frame'),vals(truth,'tangentN'),'k--',label='Reference')
for rr,name,color in series[1:]:
    axs[0].plot(vals(rr,'frame'),vals(rr,'normalN'),color=color,label=name)
    axs[1].plot(vals(rr,'frame'),vals(rr,'tangentN'),color=color,label=name)
    axs[2].plot(vals(rr,'frame'),vals(rr,'tipFx'),color=color,label=name+' Fx')
    axs[2].plot(vals(rr,'frame'),vals(rr,'tipFz'),color=color,ls=':',label=name+' Fz')
axs[2].axhline(1,color='k',ls='--');axs[2].axhline(0,color='k',ls=':',alpha=.5)
for ax,label in zip(axs,['Normal contact force (N)','Signed tangential force (N)','Tip-force components (N)']):
    ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvline(21,color='k',ls=':',alpha=.3);ax.legend(fontsize=8)
axs[2].set_xlabel('Frame');fig.suptitle('Insertion / withdrawal friction reversal and force redistribution');fig.tight_layout();fig.savefig(PUB/'force_trajectories.png',dpi=170,bbox_inches='tight');plt.close(fig)
print(json.dumps(summary,indent=2))
