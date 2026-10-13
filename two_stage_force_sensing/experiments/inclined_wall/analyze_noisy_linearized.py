"""Audit and report noisy full trajectories; preserve rejected/failed modes."""
from pathlib import Path
from collections import Counter
import sys,csv,json,shutil,re,hashlib
import numpy as np
import h5py
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];STAMP=sys.argv[1] if len(sys.argv)>1 else '20261011'
OUT=ROOT/'results/published'/f'noisy_linearized_{STAMP}';OUT.mkdir(parents=True,exist_ok=True)
CAL=ROOT/'results/runs'/f'noisy_linearized_{STAMP}'
noise=loadmat(CAL/'noise_payload.mat');cal=json.loads((CAL/'noise_calibration.json').read_text());shutil.copy2(CAL/'noise_calibration.json',OUT/'noise_calibration.json')
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def val(r,k):return np.array([float(x[k]) for x in r])
def rms(v):return float(np.sqrt(np.mean(v*v)))
def mode(row):
 s=row['mode']
 if 'zeros' in s or s=='[]':return 'separation'
 ns=[int(v) for v in re.findall(r'-?\d+',s)]
 if len(ns)>1:return 'multiple:'+s
 return {0:'separation',1:'sticking',2:'sliding',3:'frictionless'}[ns[0]]
summary=[];framesOut=[];branchOut=[];outerOut=[];data={};validated=0
traceKeys=['iteration','radius','step','innerIterations','innerExitflag','innerViolation','trueViolation','predictedReduction','actualReduction','ratio','penalty','modelFbgRmsError','accepted','seconds','trueObjective','trueFbgRms','predictedObjective']
for target in [2,5]:
 sigma=cal['levels'][str(target)]['sigmaPerMm']
 for scenario,T in [('insertion',41),('rotation',25)]:
  tag=f'inclined_{scenario}_noisy{target}mm_{STAMP}';pub=ROOT/'results/published'/tag
  rows=read(pub/'estimates.csv');attempts=read(pub/'branch_attempts.csv');assert len(rows)==2*T
  conf=json.loads((pub/'configuration.json').read_text());assert np.isclose(conf['sensorStd'],sigma,rtol=1e-12,atol=0) and conf['fbgRmsAcceptance']==1.24
  truthPath=ROOT/('results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth_frames.csv' if scenario=='insertion' else 'results/runs/inclined_wall_rotation_20261010/truth_frames.csv')
  tr=read(truthPath)
  with h5py.File(CAL/f'{scenario}_{target}mm_truth.mat') as tf:
   truth={k:tf['truth/'+k][()] for k in ['p','tipForce','contactForce']}
  with h5py.File(ROOT/'results/runs'/tag/'comparison.mat') as f:
   for x in rows:
    method=int(x['method']);k=int(x['frame'])-1;ev=f[f['runs'][k,method-1]]['mechanics/evaluation'];sh=ev['shape']
    slip=ev['slip'];x['estimatedSlipNormMm']=0.0 if slip.attrs.get('MATLAB_empty',False) else float(np.linalg.norm(slip[()]));x['referenceSlipAbsMm']=abs(float(tr[k]['slipMm']))
    err=sh['p'][()]-truth['p'][k]
    assert np.isclose(np.sqrt(np.mean(np.sum(err*err,axis=1))),float(x['shapeRmsMm']))
    assert np.isclose(np.linalg.norm(sh['x'][()].ravel()[3:6]-truth['tipForce'][k]),float(x['tipErrorN']))
    assert float(sh['rhsEvaluations'][0,0])>0
    assert np.isclose(float(x['curvatureRmsPerMm']),sigma*float(x['fbgRms']),rtol=1e-8,atol=1e-12)
    if not int(x['restoredSeed']):
     aa=[a for a in attempts if a['method']==x['method'] and a['frame']==x['frame']]
     assert len(aa)==int(x['assignments']);assert sum(int(a['ivps']) for a in aa)==int(x['ivps']);assert sum(int(a['iterations']) for a in aa)==int(x['iterations'])
     assert aa[0]['modes']==x['previousMode'];assert not any(int(a['acceptable']) for a in aa[:-1])
     if int(x['modeAccepted']):assert int(x['selectedAttempt'])==len(aa) and float(x['fbgRms'])<=1.24 and int(x['solverSuccess'])
     else:assert len(aa)==int(x['possibleAssignments'])
    if not int(x['restoredSeed']):
     sol=f[f['runs'][k,method-1]]['mechanics']
     for ia,ref in enumerate(sol['attempts'][()].ravel()):
      branch=f[ref];trace=branch['outerTrace'];nt=int(branch['outerIterations'][0,0]);lastTime=0
      for it in range(nt):
       row=dict(targetMm=target,scenario=scenario,method=method,mu=float(x['mu']),frame=k+1,attempt=ia+1)
       for key in traceKeys:
        raw=trace[key][()];row[key]=float(f[raw.ravel()[it]][()].ravel()[0]) if raw.dtype==h5py.ref_dtype else float(raw.ravel()[it])
       row['iterationSeconds']=row['seconds']-lastTime;lastTime=row['seconds'];outerOut.append(row)
    trueMode=tr[k]['mode'];trueMode='separation' if trueMode=='free' else trueMode
    if trueMode=='saved-insertion-state':trueMode='sliding'
    active=np.linalg.norm(truth['contactForce'][k])>1e-3;estimatedActive=int(x['activeCount'])>0
    x.update(scenario=scenario,targetMm=target,estimatedModeName=mode(x),trueMode=trueMode,contactDetectionCorrect=int(active==estimatedActive),modeLabelCorrect=int(mode(x)==trueMode),modeCorrect=int(mode(x)==trueMode and active==estimatedActive),trueContactActive=int(active),trueSensorNormalizedRms=float(np.sqrt(np.mean(noise[scenario+'StandardNoise'][:,:,k]**2))))
    framesOut.append(x);validated+=1
  for a in attempts:a.update(scenario=scenario,targetMm=target);branchOut.append(a)
  for method in [1,2]:
   r=[x for x in rows if int(x['method'])==method and not int(x['restoredSeed'])];data[(target,scenario,method)]=r
   s=dict(targetMm=target,scenario=scenario,mu=float(r[0]['mu']),sigmaPerMm=sigma,frames=len(r),converged=int(sum(val(r,'converged'))),feasible=int(sum(val(r,'modeFeasible'))),solverSuccess=int(sum(val(r,'solverSuccess'))),fitAccepted=int(sum(val(r,'modeAccepted'))),geometryConverged=int(sum(val(r,'geometryExitflag')>0)),contactDetectionCorrect=int(sum(val(r,'contactDetectionCorrect'))),modeCorrect=int(sum(val(r,'modeCorrect'))))
   for key,label in [('seconds','Seconds'),('ivps','IVPs'),('iterations','Iterations'),('outerIterations','OuterIterations'),('assignments','Assignments'),('geometrySeconds','GeometrySeconds')]:
    a=val(r,key);s['mean'+label]=float(np.mean(a));s['max'+label]=float(max(a))
   s.update(medianSeconds=float(np.median(val(r,'seconds'))),totalSeconds=float(sum(val(r,'seconds'))),failedIVPs=int(sum(val(r,'failedIVPs'))),tipRMSE_N=rms(val(r,'tipErrorN')),contactRMSE_N=rms(val(r,'contactErrorN')),shapeRMS_mm=rms(val(r,'shapeRmsMm')),shapeTipRMS_mm=rms(val(r,'shapeTipErrorMm')),stage1TipRMS_mm=rms(val(r,'stage1TipErrorMm')),stage1ShapeRMS_mm=rms(val(r,'stage1ShapeRmsMm')),meanFbgRms=float(np.mean(val(r,'fbgRms'))),maxFbgRms=float(max(val(r,'fbgRms'))),maxModeViolation=float(max(val(r,'modeViolation'))),maxPenetrationMm=float(max(val(r,'penetrationMm'))),maxTrueWallPenetrationMm=float(max(val(r,'truthPlanePenetrationMm'))),maxTipErrorN=float(max(val(r,'tipErrorN'))),maxContactErrorN=float(max(val(r,'contactErrorN'))),maxNormalErrorDeg=float(max(val(r,'normalErrorDeg'))),arcErrorRMS_mm=rms(val([x for x in r if np.isfinite(float(x['arcErrorMm']))],'arcErrorMm')),modes=dict(Counter(x['estimatedModeName'] for x in r)),flags=dict(Counter(y for x in r for y in x['flags'].split(';') if y)),failedFrames=[int(x['frame']) for x in r if not int(x['solverSuccess'])],rejectedFrames=[int(x['frame']) for x in r if not int(x['modeAccepted'])],wrongContactFrames=[int(x['frame']) for x in r if not x['contactDetectionCorrect']])
   summary.append(s)
# Rotation starts from its own noisy insertion state at frame21.
for target in [2,5]:
 for method in [1,2]:
  ins=next(x for x in framesOut if x['scenario']=='insertion' and x['targetMm']==target and int(x['method'])==method and int(x['frame'])==21)
  rot=next(x for x in framesOut if x['scenario']=='rotation' and x['targetMm']==target and int(x['method'])==method and int(x['frame'])==1)
  for k in ['tipFx','tipFy','tipFz','contactFx','contactFy','contactFz','shapeRmsMm']:assert float(ins[k])==float(rot[k])
for scenario in ['insertion','rotation']:
 fig,axs=plt.subplots(2,3,figsize=(15,8),sharex=True)
 for ax,(key,label) in zip(axs.flat,[('seconds','Time per step [s]'),('ivps','IVPs per step'),('fbgRms','Normalized FBG RMS'),('tipErrorN','Tip-force error [N]'),('contactErrorN','Contact-force error [N]'),('shapeTipErrorMm','Estimated tip-position error [mm]')]):
  for target,color in [(2,'#087f8c'),(5,'#c36b20')]:
   for method,ls in [(1,'-'),(2,'--')]:
    r=data[(target,scenario,method)];ax.plot(val(r,'frame'),val(r,key),ls,color=color,label=f'{target} mm noise, mu={r[0]["mu"]}')
    bad=[x for x in r if not int(x['solverSuccess'])]
    if bad:ax.scatter(val(bad,'frame'),val(bad,key),marker='x',color='crimson',s=40,zorder=5)
  if key in ['seconds','ivps']:ax.set_yscale('symlog',linthresh=1)
  if key=='fbgRms':ax.axhline(1.24,color='gray',ls=':')
  ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvline(21 if scenario=='insertion' else 13,color='gray',ls=':')
 axs[0,0].legend(fontsize=8)
 for ax in axs[1,:]:ax.set_xlabel('Frame')
 fig.suptitle(f'{scenario.capitalize()}: Gaussian Bishop noise, linearized rod, full friction cone\nAll attempted-mode costs included; red x = nonconverged or infeasible selected state')
 fig.tight_layout();fig.savefig(OUT/f'{scenario}_noise_comparison.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(13,7),sharey=True)
for row,target in enumerate([2,5]):
 for col,scenario in enumerate(['insertion','rotation']):
  ax=axs[row,col];r=data[(target,scenario,1)];mapping={'separation':0,'sticking':1,'sliding':2}
  ax.step(val(r,'frame'),[mapping.get(x['trueMode'],3) for x in r],where='mid',color='black',ls='--',label='Reference mode')
  ax.step(val(r,'frame'),[mapping.get(x['estimatedModeName'],3) for x in r],where='mid',color='#087f8c',label='Selected branch')
  inactive=[x for x in r if x['estimatedModeName']!='separation' and not int(x['activeCount'])]
  if inactive:ax.scatter(val(inactive,'frame'),[mapping.get(x['estimatedModeName'],3) for x in inactive],facecolors='none',edgecolors='#c36b20',s=60,zorder=6)
  bad=[x for x in r if not int(x['solverSuccess'])]
  if bad:ax.scatter(val(bad,'frame'),[mapping.get(x['estimatedModeName'],3) for x in bad],marker='x',color='crimson',s=55,zorder=5)
  ax.set_title(f'{scenario}, {target} mm noise');ax.set_yticks([0,1,2],['Separation','Sticking','Sliding']);ax.set_ylim(-.3,2.3);ax.set_xlabel('Frame');ax.grid(alpha=.2)
axs[0,0].legend(fontsize=9);fig.suptitle('Selected mode branches, mu=0.3: passing the fit gate does not imply the correct mode\nOrange circles: inactive contact despite branch label; red crosses: nonconverged')
fig.tight_layout();fig.savefig(OUT/'mode_identification.png',dpi=160);plt.close(fig)
def writecsv(path,rows):
 keys=[k for k,v in rows[0].items() if not isinstance(v,(dict,list))]
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows([{k:r[k] for k in keys} for r in rows])
writecsv(OUT/'outer_iterations.csv',outerOut);writecsv(OUT/'aggregate_metrics.csv',summary);writecsv(OUT/'all_frames.csv',framesOut);writecsv(OUT/'all_branch_attempts.csv',branchOut)
report={'calibration':cal,'metrics':summary,'validatedSavedStates':validated,'exportedOuterIterations':len(outerOut),'covarianceAndGateChecked':True,'nonlinearShapeErrorsChecked':True,'noisyRotationCheckpointsChecked':True,'sourceHashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'matlab/+tsfs').glob('*mode*.m')}}
(OUT/'summary.json').write_text(json.dumps(report,indent=2))
for s in summary:print(s['targetMm'],s['scenario'],s['mu'],'success',s['solverSuccess'],'/',s['frames'],'fit',s['fitAccepted'],'mode',s['modeCorrect'],'time',round(s['meanSeconds'],3),'IVP',round(s['meanIVPs'],2),'tip',round(s['tipRMSE_N'],3),'contact',round(s['contactRMSE_N'],3),'failed',s['failedFrames'],flush=True)
print('Validated',validated,'saved states; outputs',OUT)
