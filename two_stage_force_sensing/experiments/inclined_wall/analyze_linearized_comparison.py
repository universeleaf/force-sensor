"""Compare independently propagated full and affine-rod trajectories."""
from pathlib import Path
from collections import Counter
import csv,json,hashlib,sys
import numpy as np
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
STAMP=sys.argv[1] if len(sys.argv)>1 else '20261011_v2'
PUB=ROOT/'results/published'/f'linearized_modes_{STAMP}';PUB.mkdir(exist_ok=True,parents=True)
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def v(rows,key):return np.array([float(r[key]) for r in rows])
def rms(a):return float(np.sqrt(np.mean(a*a)))
summary={};allrows=[];validation={}
for scenario,T,truthpath in [('insertion',41,ROOT/'results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth.mat'),('rotation',25,ROOT/'results/runs/inclined_wall_rotation_20261010/truth.mat')]:
 data={}
 with h5py.File(truthpath) as f:truth={k:f['truth/'+k][()] for k in ['p','tipForce','contactForce']}
 for variant in ['full','linearized']:
  tag=f'inclined_{scenario}_{variant}_{STAMP}';folder=ROOT/'results/published'/tag
  rows=read(folder/'estimates.csv');branches=read(folder/'branch_attempts.csv');assert len(rows)==2*T
  data[variant]=rows
  for method in [1,2]:
   whole=[r for r in rows if int(r['method'])==method];assert list(v(whole,'frame'))==list(range(1,T+1))
   r=[x for x in whole if not int(x['restoredSeed'])];b=[x for x in branches if int(x['method'])==method]
   for x in r:
    a=[q for q in b if q['frame']==x['frame']];assert len(a)==int(x['assignments'])
    assert sum(int(q['ivps']) for q in a)==int(x['ivps'])
    assert sum(int(q['iterations']) for q in a)==int(x['iterations'])
    assert a[0]['modes']==x['previousMode'];accepted=[int(q['acceptable']) for q in a];assert not any(accepted[:-1])
    if any(accepted):assert int(x['selectedAttempt'])==len(a) and int(x['modeAccepted'])==1
    else:assert len(a)==int(x['possibleAssignments']) and int(x['modeAccepted'])==0
    assert abs(float(x['curvatureRmsPerMm'])-1e-4*float(x['fbgRms']))<1e-12
   with h5py.File(ROOT/'results/runs'/tag/'comparison.mat') as f:
    for k,x in enumerate(whole):
     sh=f[f['runs'][k,method-1]]['mechanics/evaluation/shape']
     assert float(sh['rhsEvaluations'][0,0])>0,'Published affine state instead of nonlinear IVP'
     err=sh['p'][()]-truth['p'][k];assert np.isclose(np.sqrt(np.mean(np.sum(err*err,axis=1))),float(x['shapeRmsMm']))
     assert np.isclose(np.linalg.norm(sh['x'][()].ravel()[3:6]-truth['tipForce'][k]),float(x['tipErrorN']))
     if variant=='linearized' and not int(x['restoredSeed']):
      sol=f[f['runs'][k,method-1]]['mechanics'];assert int(sol['counts/affineEvaluations'][0,0])==int(x['affineEvaluations'])
   s=dict(scenario=scenario,variant=variant,mu=float(r[0]['mu']),frames=len(r),converged=int(sum(v(r,'converged'))),feasible=int(sum(v(r,'modeFeasible'))),solverSuccess=int(sum(v(r,'solverSuccess'))),fitAccepted=int(sum(v(r,'modeAccepted'))),
    meanSeconds=float(np.mean(v(r,'seconds'))),medianSeconds=float(np.median(v(r,'seconds'))),maxSeconds=float(max(v(r,'seconds'))),totalSeconds=float(sum(v(r,'seconds'))),meanMechanicsSeconds=float(np.mean(v(r,'mechanicsSeconds'))),meanGeometrySeconds=float(np.mean(v(r,'geometrySeconds'))),
    meanIterations=float(np.mean(v(r,'iterations'))),meanIVPs=float(np.mean(v(r,'ivps'))),totalIVPs=int(sum(v(r,'ivps'))),failedIVPs=int(sum(v(r,'failedIVPs'))),meanOuterIterations=float(np.mean(v(r,'outerIterations'))),meanAffineEvaluations=float(np.mean(v(r,'affineEvaluations'))),
    tipRMSE_N=rms(v(r,'tipErrorN')),contactRMSE_N=rms(v(r,'contactErrorN')),shapeRMS_mm=rms(v(r,'shapeRmsMm')),
    maxTipError_N=float(max(v(r,'tipErrorN'))),maxContactError_N=float(max(v(r,'contactErrorN'))),maxShapeRmsMm=float(max(v(r,'shapeRmsMm'))),maxRawFbgRms=float(max(v(r,'curvatureRmsPerMm'))),
    maxModeViolation=float(max(v(r,'modeViolation'))),maxProjectionN=float(max(v(r,'projectionN'))),maxPenetrationMm=float(max(v(r,'penetrationMm'))),maxTipMomentNmm=float(max(v(r,'tipMomentNmm'))),
    modeCounts=dict(Counter(x['mode'] for x in r)),flags=dict(Counter(flag for x in r for flag in x['flags'].split(';') if flag)),
    failedSolveFrames=[int(x['frame']) for x in r if x['solverSuccess']=='0'],rejectedFitFrames=[int(x['frame']) for x in r if x['modeAccepted']=='0'])
   summary[f'{scenario}_{variant}_mu{r[0]["mu"]}']=s;allrows.append(s)
 for method in [1,2]:
  full=[x for x in data['full'] if int(x['method'])==method];lin=[x for x in data['linearized'] if int(x['method'])==method]
  for key in ['estimatedArcMm','normalErrorDeg','stage1ShapeRmsMm']:
   assert np.allclose(v(full,key),v(lin,key),equal_nan=True),key
  if scenario=='rotation':
   for key in ['tipFx','tipFy','tipFz','contactFx','contactFy','contactFz','shapeRmsMm']:
    assert float(full[0][key])==float(lin[0][key]),key
 validation[scenario]={'allSavedErrorsChecked':True,'sameStage1':True,'sameRestoredSeed':scenario=='rotation'}
 fig,axs=plt.subplots(2,3,figsize=(15,8),sharex=True)
 keys=[('seconds','Step time [s]'),('ivps','Nonlinear IVP evaluations'),('iterations','All inner SQP iterations'),('tipErrorN','Tip force error [N]'),('contactErrorN','Contact force error [N]'),('shapeRmsMm','Shape RMS error [mm]')]
 for ax,(key,label) in zip(axs.flat,keys):
  for method,color in [(1,'#087f8c'),(2,'#d26927')]:
   for variant,ls in [('full','--'),('linearized','-')]:
    r=[x for x in data[variant] if int(x['method'])==method and not int(x['restoredSeed'])]
    ax.semilogy(v(r,'frame'),np.maximum(v(r,key),1e-12),ls,color=color,label=f'{variant}, mu={r[0]["mu"]}')
    failed=[x for x in r if x['solverSuccess']=='0']
    if failed:ax.scatter(v(failed,'frame'),np.maximum(v(failed,key),1e-12),marker='x',s=60,color='#b72b38',zorder=5)
  if key=='iterations':ax.set_yscale('symlog',linthresh=1)
  ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvline(21 if scenario=='insertion' else 13,color='gray',ls=':')
 axs[0,0].legend(fontsize=8)
 for ax in axs[1,:]:ax.set_xlabel('Frame')
 fig.suptitle(f'{scenario.capitalize()}: full nonlinear versus successive rod linearization\nSame full circular friction, observations and thresholds; all fallbacks retained; red x = nonconverged return')
 fig.tight_layout();fig.savefig(PUB/f'{scenario}_comparison.png',dpi=160);plt.close(fig)
summary['validation']=validation
summary['sourceHashes']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'matlab/+tsfs'/name for name in ['mode_branch.m','mode_solve.m','linearized_evaluate.m','linearized_model.m','linearized_mode_branch.m','mode_residual.m','evaluate.m','ivp.m']]}
(PUB/'summary.json').write_text(json.dumps(summary,indent=2))
scalar_keys=[k for k,x in allrows[0].items() if not isinstance(x,(dict,list))]
with (PUB/'aggregate_metrics.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=scalar_keys);writer.writeheader();writer.writerows([{k:r[k] for k in scalar_keys} for r in allrows])
for r in allrows:
 print(r['scenario'],r['variant'],r['mu'],'time',round(r['meanSeconds'],4),'IVP',round(r['meanIVPs'],2),'it',round(r['meanIterations'],2),'outer',round(r['meanOuterIterations'],2),'tip/contact/shape',r['tipRMSE_N'],r['contactRMSE_N'],r['shapeRMS_mm'],'success',r['solverSuccess'],'fit',r['fitAccepted'],'failframes',r['failedSolveFrames'])
