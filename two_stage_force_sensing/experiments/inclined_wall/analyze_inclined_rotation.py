"""Audit saved rotation replay, checkpoints, search semantics and aggregate errors."""
from pathlib import Path
import csv,json,hashlib,shutil
from collections import Counter
import numpy as np
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
RUN=ROOT/'results/runs/inclined_wall_rotation_20261010'
PUB=ROOT/'results/published/inclined_wall_rotation_20261010'
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def v(r,k):return np.array([float(x[k]) for x in r])
def rms(a):return float(np.sqrt(np.mean(a*a)))
rows=read(PUB/'estimates.csv');branches=read(PUB/'branch_attempts.csv');truth=read(RUN/'truth_frames.csv')
old=read(ROOT/'results/published/inclined_wall_modes_conditional_1e6_20261010/estimates.csv')
assert len(rows)==50 and len(truth)==25
assert all(int(x['passed']) for x in truth)
summary={'statisticsScope':'24 newly actuated frames per method; saved frame 1 excluded from aggregates'}
with h5py.File(RUN/'comparison.mat') as f,h5py.File(ROOT/'results/runs/inclined_wall_modes_conditional_1e6_20261010/comparison.mat') as prior,h5py.File(RUN/'truth.mat') as tf,h5py.File(RUN/'maximum_insertion_seed.mat') as sf:
 for j in range(2):
  a=f[f['runs'][0,j]];b=prior[prior['runs'][20,j]]
  for key in ['mechanics/evaluation/shape/p','mechanics/evaluation/shape/x','geometry/s','geometry/normal']:
   assert np.array_equal(a[key][()],b[key][()]),key
 for key in ['p','u','basePose','tipForce','contactForce','bishopW']:
  assert np.array_equal(np.squeeze(tf['truth/'+key][0]),np.squeeze(sf['seed/'+key][()])),key
 extent=np.concatenate([tf['truth/p'][()].reshape(-1,3)]+[f[f['runs'][k,j]]['mechanics/evaluation/shape/p'][()] for j in range(2) for k in range(25)])
 summary['shapeBoundsMm']={'minimum':extent.min(axis=0).tolist(),'maximum':extent.max(axis=0).tolist()}
for mu in [.3,0.]:
 allr=[x for x in rows if float(x['mu'])==mu];r=allr[1:];b=[x for x in branches if float(x['mu'])==mu]
 assert [int(x['frame']) for x in allr]==list(range(1,26))
 assert int(allr[0]['restoredSeed']) and float(allr[0]['seconds'])==0 and int(allr[0]['ivps'])==0
 for x in r:
  a=[t for t in b if t['frame']==x['frame']]
  assert a[0]['modes']==x['previousMode']
  assert sum(int(t['ivps']) for t in a)==int(x['ivps'])
  assert sum(int(t['iterations']) for t in a)==int(x['iterations'])
  accepted=[int(t['acceptable']) for t in a]
  assert not any(accepted[:-1])
  if any(accepted):assert int(x['selectedAttempt'])==len(a) and int(x['modeAccepted'])==1
  else:assert len(a)==int(x['possibleAssignments']) and int(x['modeAccepted'])==0
  assert int(x['objectiveTerms'])==48
  assert abs(float(x['curvatureRmsPerMm'])-1e-4*float(x['fbgRms']))<1e-12
 s=dict(solves=len(r),solverSuccess=int(sum(v(r,'solverSuccess'))),accepted=int(sum(v(r,'modeAccepted'))),
  totalSeconds=float(sum(v(r,'seconds'))),meanSeconds=float(np.mean(v(r,'seconds'))),medianSeconds=float(np.median(v(r,'seconds'))),maxSeconds=float(max(v(r,'seconds'))),
  meanIterations=float(np.mean(v(r,'iterations'))),meanIVPs=float(np.mean(v(r,'ivps'))),totalIVPs=int(sum(v(r,'ivps'))),
  tipRMSE_N=rms(v(r,'tipErrorN')),contactRMSE_N=rms(v(r,'contactErrorN')),shapeRMS_mm=rms(v(r,'shapeRmsMm')),
  maxTipError_N=float(max(v(r,'tipErrorN'))),maxContactError_N=float(max(v(r,'contactErrorN'))),maxShapeRMS_mm=float(max(v(r,'shapeRmsMm'))),
  maxFBGrmsPerMm=float(max(v(r,'curvatureRmsPerMm'))),failedIVPs=int(sum(v(r,'failedIVPs'))),attemptedModes=len(b),modeCounts=dict(Counter(x['mode'] for x in r)),
  noAcceptableModeFrames=[int(x['frame']) for x in r if not int(x['modeAccepted'])],
  flags=dict(Counter(flag for x in r for flag in x['flags'].split(';') if flag)),
  maxDensePenetrationMm=float(max(v(r,'penetrationMm'))),maxProjectionN=float(max(v(r,'projectionN'))),maxTangency=float(max(v(r,'tangency'))),
  stage1MaxArcErrorMm=float(max(abs(v(r,'arcErrorMm')))),stage1MaxNormalErrorDeg=float(max(v(r,'normalErrorDeg'))),stage1MaxShapeRmsMm=float(max(v(r,'stage1ShapeRmsMm'))),
  seed={k:float(allr[0][k]) for k in ['tipErrorN','contactErrorN','shapeRmsMm']})
 for phase,part in [('forward',r[:12]),('reverse',r[12:])]:
  s[phase]=dict(accepted=int(sum(v(part,'modeAccepted'))),tipRMSE_N=rms(v(part,'tipErrorN')),contactRMSE_N=rms(v(part,'contactErrorN')),shapeRMS_mm=rms(v(part,'shapeRmsMm')),meanSeconds=float(np.mean(v(part,'seconds'))))
 stick=[x for x in b if x['modes']=='1']
 s['stickingAttempts']=dict(count=len(stick),nativePositive=sum(int(x['nativePositive']) for x in stick),feasible=sum(int(x['feasible']) for x in stick),accepted=sum(int(x['acceptable']) for x in stick),seconds=sum(float(x['seconds']) for x in stick))
 summary[f'mu_{mu:g}']=s
summary['truth']=dict(passed=25,modeCounts=dict(Counter(x['mode'] for x in truth)),maxGapMm=float(max(abs(v(truth,'gapMm')))),maxTangency=float(max(abs(v(truth,'tangency')))),maxPenetrationMm=float(max(v(truth,'penetrationMm'))),maxProjectionN=float(max(v(truth,'projectionN'))),maxTipMomentNmm=float(max(v(truth,'terminalMomentNmm'))),maxIntegratedTwistDeg=float(max(abs(v(truth,'twistIntegralDeg')))),totalReferenceSeconds=float(sum(v(truth,'seconds'))))
summary['sourceHashes']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'matlab/+tsfs/mode_solve.m',ROOT/'matlab/+tsfs/mode_branch.m',ROOT/'experiments/inclined_wall/run_inclined_modes.m',ROOT/'experiments/inclined_wall/build_inclined_rotation_truth.m',ROOT/'experiments/inclined_wall/run_inclined_rotation_modes.m']}
(PUB/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');shutil.copy2(RUN/'truth_frames.csv',PUB/'truth_frames.csv')
fig,axs=plt.subplots(2,2,figsize=(12,8),sharex=True)
for ax,(key,label) in zip(axs.flat,[('seconds','Time per new solve [s]'),('tipErrorN','Tip-force error [N]'),('contactErrorN','Contact-force error [N]'),('shapeRmsMm','Shape RMS error [mm]')]):
 for mu,color in [(.3,'#087f8c'),(0.,'#d26927')]:
  r=[x for x in rows if float(x['mu'])==mu and int(x['frame'])>1];ax.semilogy(v(r,'frame'),np.maximum(v(r,key),1e-10),color=color,label=f'mu = {mu:g}')
  rejected=[x for x in r if x['modeAccepted']=='0'];ax.scatter(v(rejected,'frame'),np.maximum(v(rejected,key),1e-10),marker='x',color=color,s=25)
 ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvline(13,color='gray',ls=':')
axs[0,0].legend()
for ax in axs[1,:]:ax.set_xlabel('Frame (13 = 45 degrees, 25 = return to zero)')
fig.suptitle('Loaded-state rotation: 0 to 45 degrees and back\nCrosses mark rejected FBG fits; timing includes all attempted modes')
fig.tight_layout();fig.savefig(PUB/'rotation_comparison.png',dpi=160)
print(json.dumps(summary,indent=2))
