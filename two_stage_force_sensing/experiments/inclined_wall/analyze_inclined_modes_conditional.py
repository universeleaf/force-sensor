"""Compare conditional and exhaustive mode trajectories; enforce search semantics."""
from pathlib import Path
import csv,json,hashlib,sys
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
tag=sys.argv[1] if len(sys.argv)>1 else 'inclined_wall_modes_conditional_1e6_20261010'
PUB=ROOT/'results/published'/tag
OLD=ROOT/'results/published/inclined_wall_modes_20261010'
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def v(r,k):return np.array([float(x[k]) for x in r])
def rms(a):return float(np.sqrt(np.mean(a*a)))
rows=read(PUB/'estimates.csv'); branches=read(PUB/'branch_attempts.csv'); old=read(OLD/'estimates.csv')
assert len(rows)==82
summary={}
for mu in [.3,0.]:
 r=[x for x in rows if float(x['mu'])==mu];b=[x for x in branches if float(x['mu'])==mu];prior=[x for x in old if float(x['mu'])==mu]
 assert [int(x['frame']) for x in r]==list(range(1,42))
 for x in r:
  a=[t for t in b if t['frame']==x['frame']]
  assert a[0]['modes']==x['previousMode']
  assert sum(int(t['ivps']) for t in a)==int(x['ivps'])
  assert sum(int(t['iterations']) for t in a)==int(x['iterations'])
  acceptable=[int(t['acceptable']) for t in a]
  assert not any(acceptable[:-1]),'Continued search after acceptable mode'
  if any(acceptable):assert int(x['selectedAttempt'])==len(a) and int(x['modeAccepted'])==1
  else:assert len(a)==int(x['possibleAssignments']) and int(x['modeAccepted'])==0
  assert int(x['objectiveTerms'])==48
 s=dict(frames=len(r),solverSuccess=int(sum(v(r,'solverSuccess'))),accepted=int(sum(v(r,'modeAccepted'))),
  optimalityExits=sum(x['reason']=='fmincon-exit-1' for x in r),warningFree=sum(not x['flags'] for x in r),
  totalSeconds=float(sum(v(r,'seconds'))),meanSeconds=float(np.mean(v(r,'seconds'))),medianSeconds=float(np.median(v(r,'seconds'))),maxSeconds=float(max(v(r,'seconds'))),
  meanIterations=float(np.mean(v(r,'iterations'))),meanIVPs=float(np.mean(v(r,'ivps'))),totalIVPs=int(sum(v(r,'ivps'))),
  tipRMSE=rms(v(r,'tipErrorN')),contactRMSE=rms(v(r,'contactErrorN')),shapeRMS=rms(v(r,'shapeRmsMm')),
  maxTipError=float(max(v(r,'tipErrorN'))),maxContactError=float(max(v(r,'contactErrorN'))),maxShapeRMS=float(max(v(r,'shapeRmsMm'))),
  maxFBGrms=float(max(v(r,'fbgRms'))),threshold=float(r[0]['fbgRmsAcceptance']),
  attemptedModes=len(b),stickingAttempts=sum(x['modes']=='1' for x in b),failedIVPs=int(sum(v(r,'failedIVPs'))),
  modeCounts=dict(Counter(x['mode'] for x in r)),
  matchedContactFrames=sum(int(x['activeCount'])>0 for x in r if 4<=int(x['frame'])<=38),
  multipleAttemptFrames=[int(x['frame']) for x in r if int(x['assignments'])>1],
  noAcceptableModeFrames=[int(x['frame']) for x in r if not int(x['modeAccepted'])],
  flags=dict(Counter(f for x in r for f in x['flags'].split(';') if f)),
  maxDensePenetration=float(max(v(r,'penetrationMm'))),maxProjection=float(max(v(r,'projectionN'))),
  previousMeanSeconds=float(np.mean(v(prior,'seconds'))),previousTipRMSE=rms(v(prior,'tipErrorN')),previousContactRMSE=rms(v(prior,'contactErrorN')),previousShapeRMS=rms(v(prior,'shapeRmsMm')),
  maxTipDifference=float(max(np.linalg.norm(np.column_stack([v(r,k)-v(prior,k) for k in ['tipFx','tipFy','tipFz']]),axis=1))),
  maxContactDifference=float(max(np.linalg.norm(np.column_stack([v(r,k)-v(prior,k) for k in ['contactFx','contactFy','contactFz']]),axis=1))))
 summary[f'mu_{mu:g}']=s
summary['sourceHashes']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'matlab/+tsfs/mode_solve.m',ROOT/'matlab/+tsfs/mode_branch.m',ROOT/'experiments/inclined_wall/run_inclined_modes.m',ROOT/'experiments/inclined_wall/run_inclined_modes_conditional.m']}
(PUB/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
fig,axs=plt.subplots(2,2,figsize=(12,8),sharex=True)
for ax,(key,label) in zip(axs.flat,[('seconds','Total time (s)'),('tipErrorN','Tip-force error (N)'),('contactErrorN','Contact-force error (N)'),('shapeRmsMm','Shape RMS error (mm)')]):
 for mu,color in [(.3,'#1769aa'),(0.,'#df761d')]:
  for data,ls,name in [(old,'--','Exhaustive'),(rows,'-','Conditional')]:
   r=[x for x in data if float(x['mu'])==mu]
   ax.semilogy(v(r,'frame'),np.maximum(v(r,key),1e-10),ls,color=color,label=f'{name}, mu={mu:g}')
 ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvline(21,color='gray',ls=':')
axs[0,0].legend(fontsize=8)
for ax in axs[1,:]:ax.set_xlabel('Frame')
fig.suptitle('Previous-mode-first conditional search: full insertion / withdrawal\nTiming includes every attempted branch; all original mechanics retained')
fig.tight_layout();fig.savefig(PUB/'conditional_comparison.png',dpi=170,bbox_inches='tight')
print(json.dumps(summary,indent=2))
