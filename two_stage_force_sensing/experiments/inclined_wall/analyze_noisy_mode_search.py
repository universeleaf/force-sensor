"""Matched exhaustive versus first-acceptable trajectories; offline budget audit.
Truth is used exclusively to score saved states, never to select runtime modes.
"""
from pathlib import Path
from collections import Counter,defaultdict
import csv,json,sys,re,hashlib
import numpy as np
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];STAMP=sys.argv[1] if len(sys.argv)>1 else '20261011'
OUT=ROOT/'results/published'/f'noisy_mode_search_{STAMP}';OUT.mkdir(parents=True,exist_ok=True)
CAL=ROOT/'results/runs/noisy_linearized_20261011'
cal=json.loads((CAL/'noise_calibration.json').read_text())
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def vec(r,k):return np.array([float(x[k]) for x in r])
def rms(a):return float(np.sqrt(np.mean(np.asarray(a)**2)))
def scalar(f,g,k,i=0):
 d=g[k][()];return float(f[d.ravel()[i]][()].ravel()[0]) if d.dtype==h5py.ref_dtype else float(d.ravel()[i])
def array(f,g,k,i=0):
 d=g[k][()];return f[d.ravel()[i]][()].ravel() if d.dtype==h5py.ref_dtype else d.ravel()
def mode_label(g):
 if g['mode'].attrs.get('MATLAB_empty',False):return '0'
 return ','.join(str(int(a)) for a in g['mode'][()].ravel())
def writecsv(path,rows):
 keys=list(rows[0])
 with path.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
hashes=json.loads((ROOT/'results/published/noisy_linearized_20261011/summary.json').read_text())['sourceHashes']
for name,expected in hashes.items():assert hashlib.sha256((ROOT/'matlab/+tsfs'/name).read_bytes()).hexdigest()==expected
allframes=[];allbranches=[];alltrace=[];metrics=[];data={};valid=0
traceKeys=['iteration','radius','step','innerIterations','innerExitflag','innerViolation','trueViolation','predictedReduction','actualReduction','ratio','penalty','modelFbgRmsError','accepted','seconds','trueObjective','trueFbgRms','predictedObjective']
for target in [2,5]:
 for scenario,T in [('insertion',41),('rotation',25)]:
  tp=ROOT/('results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth_frames.csv' if scenario=='insertion' else 'results/runs/inclined_wall_rotation_20261010/truth_frames.csv');tr=read(tp)
  with h5py.File(CAL/f'{scenario}_{target}mm_truth.mat') as tf:truth={k:tf['truth/'+k][()] for k in ['p','tipForce','contactForce']}
  for policy in ['conditional','exhaustive']:
   tag=f'inclined_{scenario}_noisy{target}mm_'+('20261011' if policy=='conditional' else f'exhaustive_{STAMP}')
   pub=ROOT/'results/published'/tag;rows=[r for r in read(pub/'estimates.csv') if r['method']=='1'];assert len(rows)==T
   cfg=json.loads((pub/'configuration.json').read_text());assert cfg['linearization']['maxOuterIterations']==40 and cfg['fbgRmsAcceptance']==1.24 and cfg['constraintTolerance']==1e-6
   sigma=cal['levels'][str(target)]['sigmaPerMm'];assert np.isclose(cfg['sensorStd'],sigma)
   with h5py.File(ROOT/'results/runs'/tag/'comparison.mat') as f:
    for k,r in enumerate(rows):
     out=f[f['runs'][k,0]];sol=out['mechanics'];ev=sol['evaluation'];sh=ev['shape'];valid+=1
     assert float(sh['rhsEvaluations'][0,0])>0
     assert np.isclose(np.linalg.norm(sh['x'][()].ravel()[3:6]-truth['tipForce'][k]),float(r['tipErrorN']))
     assert np.isclose(rms(np.linalg.norm(sh['p'][()]-truth['p'][k],axis=1)),float(r['shapeRmsMm']))
     assert np.isclose(sigma*float(r['fbgRms']),float(r['curvatureRmsPerMm']),atol=1e-12)
     trueMode={'free':'0','sliding':'2','sticking':'1','saved-insertion-state':'2'}[tr[k]['mode']]
     refActive=np.linalg.norm(truth['contactForce'][k])>1e-3
     r.update(targetMm=target,scenario=scenario,policy=policy,trueMode=trueMode,modeCorrect=int(mode_label(sol)==trueMode and (int(r['activeCount'])>0)==refActive),contactCorrect=int((int(r['activeCount'])>0)==refActive))
     allframes.append(r)
     if int(r['restoredSeed']):continue
     candidates=[]
     for ia,ref in enumerate(sol['attempts'][()].ravel()):
      b=f[ref];be=b['evaluation'];bs=be['shape'];nt=int(scalar(f,b,'outerIterations'));trace=b['outerTrace'];seen={};cycle=None;cycleMatch=0.;maxReject=0;streak=0;lastTime=0;running=[];zHistory=[]
      for it in range(nt):
       z=array(f,trace,'z',it);zHistory.append(z.copy());t=dict(targetMm=target,scenario=scenario,policy=policy,frame=k+1,attempt=ia+1)
       for key in traceKeys:t[key]=scalar(f,trace,key,it)
       rad=t['radius']
       if cycle is None and rad in seen:
        cycle=it;cycleMatch=float(np.max(np.abs(z-seen[rad][1])))
       seen[rad]=(it,z.copy())
       if t['accepted']:seen={};streak=0
       else:streak+=1;maxReject=max(maxReject,streak)
       t['iterationSeconds']=t['seconds']-lastTime;lastTime=t['seconds'];running.append(t);alltrace.append(t)
      bx=bs['x'][()].ravel();F=np.zeros(3) if bs['force'].attrs.get('MATLAB_empty',False) else bs['force'][()].sum(axis=0)
      q=dict(targetMm=target,scenario=scenario,policy=policy,frame=k+1,attempt=ia+1,mode=mode_label(b),selected=int(ia+1==int(r['selectedAttempt'])),converged=int(scalar(f,b,'converged')),feasible=int(scalar(f,b,'modeFeasible')),violation=scalar(f,b,'modeViolation'),objective=scalar(f,be,'objective'),fbgRms=scalar(f,be,'fbgRms'),seconds=scalar(f,b,'seconds'),iterations=int(scalar(f,b,'iterations')),outerIterations=nt,ivps=int(scalar(f,b['counts'],'augmentedIVPs')),rhsEvaluations=int(scalar(f,b['counts'],'rhsEvaluations')),affineEvaluations=int(scalar(f,b['counts'],'affineEvaluations')),tipErrorN=float(np.linalg.norm(bx[3:6]-truth['tipForce'][k])),contactErrorN=float(np.linalg.norm(F-truth['contactForce'][k])),modeCorrect=int(mode_label(b)==trueMode and np.any(bx[6::3]>1e-3)==refActive),activeCount=int(sum(bx[6::3]>1e-3)),cycleBeforeIteration=0 if cycle is None else cycle+1,cycleCandidateMaxDifference=cycleMatch,maxRejectedStreak=maxReject)
      q['acceptable']=int(q['converged'] and q['feasible'] and q['fbgRms']<=1.24)
      q['cyclePrefixSeconds']=q['seconds'] if cycle is None else (running[cycle-1]['seconds'] if cycle else 0)+q['seconds']-running[-1]['seconds']
      prefix=running if cycle is None else running[:cycle]
      feasibleTrace=[x for x in prefix if x['trueViolation']<=1e-6]
      q['cyclePrefixBestObjective']=min((x['trueObjective'] for x in feasibleTrace),default=float('nan'))
      q['cyclePrefixBestViolation']=min(x['trueViolation'] for x in prefix)
      q['cyclePrefixHasReturnedZ']=int(any(np.array_equal(array(f,b,'z'),z) for z in (zHistory if cycle is None else zHistory[:cycle])))
      candidates.append(q);allbranches.append(q)
     assert len(candidates)==int(r['assignments']);assert sum(x['iterations'] for x in candidates)==int(r['iterations']);assert sum(x['ivps'] for x in candidates)==int(r['ivps'])
     feasible=[x for x in candidates if x['feasible']];conv=[x for x in feasible if x['converged']];good=[x for x in candidates if x['acceptable']]
     selected=candidates[int(r['selectedAttempt'])-1]
     assert np.isclose(selected['tipErrorN'],float(r['tipErrorN']))
     if policy=='exhaustive':
      assert len(candidates)==int(r['possibleAssignments'])
      if feasible:assert np.isclose(selected['objective'],min(x['objective'] for x in feasible))
      first=good[0] if good else selected
      r['sameHistoryFirstAcceptedMode']=first['mode']
      r['sameHistoryFirstAcceptedFbgRms']=first['fbgRms']
      ranked=sorted(feasible,key=lambda x:x['objective'])
      r['secondFeasibleFbgRmsGap']=ranked[1]['fbgRms']-ranked[0]['fbgRms'] if len(ranked)>1 else float('nan')
      r['sameHistoryFirstAcceptedTipErrorN']=first['tipErrorN'];r['sameHistoryFirstAcceptedContactErrorN']=first['contactErrorN']
      r['bestConvergedTipErrorN']=min(conv,key=lambda x:x['objective'])['tipErrorN'] if conv else float('nan')
      r['bestConvergedContactErrorN']=min(conv,key=lambda x:x['objective'])['contactErrorN'] if conv else float('nan')
      r['oracleCandidateTipErrorN']=min((x['tipErrorN'] for x in feasible),default=float('nan'))
      r['oracleCandidateContactErrorN']=min((x['contactErrorN'] for x in feasible),default=float('nan'))
      correct=[x for x in feasible if x['modeCorrect']]
      r['trueModeCandidateTipErrorN']=min(correct,key=lambda x:x['objective'])['tipErrorN'] if correct else float('nan')
      r['trueModeCandidateAvailable']=int(bool(correct))
      r['trueModeCandidateContactErrorN']=min(correct,key=lambda x:x['objective'])['contactErrorN'] if correct else float('nan')
      if refActive and int(r['candidateCount'])==0:r['outcomeCategory']='Stage 1 missed contact'
      elif not selected['converged']:r['outcomeCategory']='Selected candidate not converged'
      elif not r['modeCorrect'] and not correct:r['outcomeCategory']='Wrong mode; no feasible correct-mode candidate returned'
      elif not r['modeCorrect']:r['outcomeCategory']='Wrong mode despite feasible correct-mode candidate'
      else:r['outcomeCategory']='Correct mode and converged'
     else:
      if good:assert int(r['selectedAttempt'])==good[0]['attempt'] and len(good)==1
   rr=[r for r in rows if not int(r['restoredSeed'])];data[(target,scenario,policy)]=rr
   s=dict(targetMm=target,scenario=scenario,policy=policy,frames=len(rr),converged=int(sum(vec(rr,'solverSuccess'))),feasible=int(sum(vec(rr,'modeFeasible'))),fitAccepted=int(sum(vec(rr,'modeAccepted'))),modeCorrect=int(sum(vec(rr,'modeCorrect'))),contactCorrect=int(sum(vec(rr,'contactCorrect'))),meanSeconds=float(np.mean(vec(rr,'seconds'))),medianSeconds=float(np.median(vec(rr,'seconds'))),maxSeconds=float(max(vec(rr,'seconds'))),totalSeconds=float(sum(vec(rr,'seconds'))),meanIVPs=float(np.mean(vec(rr,'ivps'))),meanIterations=float(np.mean(vec(rr,'iterations'))),meanOuterIterations=float(np.mean(vec(rr,'outerIterations'))),tipRMSE_N=rms(vec(rr,'tipErrorN')),contactRMSE_N=rms(vec(rr,'contactErrorN')),shapeRMS_mm=rms(vec(rr,'shapeRmsMm')),shapeTipRMS_mm=rms(vec(rr,'shapeTipErrorMm')),maxViolation=float(max(vec(rr,'modeViolation'))))
   if policy=='exhaustive':
    s.update(sameHistoryFirstAcceptedTipRMSE_N=rms(vec(rr,'sameHistoryFirstAcceptedTipErrorN')),sameHistoryFirstAcceptedContactRMSE_N=rms(vec(rr,'sameHistoryFirstAcceptedContactErrorN')),oracleCandidateTipRMSE_N=rms(vec(rr,'oracleCandidateTipErrorN')),oracleCandidateContactRMSE_N=rms(vec(rr,'oracleCandidateContactErrorN')),bestConvergedTipRMSE_N=rms(vec(rr,'bestConvergedTipErrorN')),bestConvergedContactRMSE_N=rms(vec(rr,'bestConvergedContactErrorN')),trueModeAvailable=int(sum(vec(rr,'trueModeCandidateAvailable'))),trueModeAvailableTipRMSE_N=rms([float(x['trueModeCandidateTipErrorN']) for x in rr if int(x['trueModeCandidateAvailable'])]),trueModeAvailableContactRMSE_N=rms([float(x['trueModeCandidateContactErrorN']) for x in rr if int(x['trueModeCandidateAvailable'])]))
   metrics.append(s)
  for key in ['estimatedArcMm','normalErrorDeg','stage1ShapeRmsMm']:
   assert np.allclose(vec(data[(target,scenario,'conditional')],key),vec(data[(target,scenario,'exhaustive')],key),equal_nan=True)
outcomeGroups=[]
for target in [2,5]:
 for scenario in ['insertion','rotation']:
  rr=data.get((target,scenario,'exhaustive'),[])
  for category in sorted({x['outcomeCategory'] for x in rr}):
   subset=[x for x in rr if x['outcomeCategory']==category]
   outcomeGroups.append(dict(targetMm=target,scenario=scenario,category=category,frames=len(subset),tipSSEFraction=float(sum(vec(subset,'tipErrorN')**2)/sum(vec(rr,'tipErrorN')**2)),contactSSEFraction=float(sum(vec(subset,'contactErrorN')**2)/sum(vec(rr,'contactErrorN')**2)),tipRMSE_N=rms(vec(subset,'tipErrorN')),contactRMSE_N=rms(vec(subset,'contactErrorN'))))
# Budget metrics use observed prefixes only; they are not reruns under new limits.
groups=defaultdict(list)
for t in alltrace:groups[(t['targetMm'],t['scenario'],t['policy'],t['frame'],t['attempt'])].append(t)
budgets=[]
for policy in ['conditional','exhaustive']:
 for typ in ['converged','selected-converged','accepted','failed']:
  bs=[x for x in allbranches if x['policy']==policy and ((x['converged'] and x['feasible']) if typ!='failed' else not (x['converged'] and x['feasible']))]
  if typ=='selected-converged':bs=[x for x in bs if x['selected']]
  if typ=='accepted':bs=[x for x in bs if x['acceptable']]
  for cap in [5,8,10,15,20,25,30,35,40]:
   prefix=[]
   for b in bs:
    t=groups[(b['targetMm'],b['scenario'],b['policy'],b['frame'],b['attempt'])];prefix.append(t[min(cap,len(t))-1]['seconds']+b['seconds']-t[-1]['seconds'])
   budgets.append(dict(policy=policy,category=typ,cap=cap,branches=len(bs),completedWithinCap=sum(x['outerIterations']<=cap for x in bs),originalSeconds=sum(x['seconds'] for x in bs),prefixSeconds=sum(prefix)))
innerBudgets=[]
for policy in ['conditional','exhaustive']:
 bs=[b for b in allbranches if b['policy']==policy and b['converged'] and b['feasible']]
 for cap in [100,150,200,250,300]:
  affected=[b for b in bs if max(t['innerIterations'] for t in groups[(b['targetMm'],b['scenario'],b['policy'],b['frame'],b['attempt'])])>cap]
  innerBudgets.append(dict(policy=policy,cap=cap,convergedBranches=len(bs),affectedBranches=len(affected),affectedSelected=sum(b['selected'] for b in affected)))
cycles=[]
for policy in ['conditional','exhaustive']:
 bs=[x for x in allbranches if x['policy']==policy];cy=[x for x in bs if x['cycleBeforeIteration']];success=[x for x in cy if x['converged'] and x['feasible']]
 cycles.append(dict(policy=policy,branches=len(bs),cyclingBranches=len(cy),cyclingConverged=len(success),cyclingSelected=sum(x['selected'] for x in cy),cyclingReturnedStatesPreserved=sum(x['cyclePrefixHasReturnedZ'] for x in cy),observedTimeSavedSeconds=sum(x['seconds']-x['cyclePrefixSeconds'] for x in cy),maxRepeatedCandidateDifference=max((x['cycleCandidateMaxDifference'] for x in cy),default=0),maxFeasibleObjectiveDifference=max((abs(x['cyclePrefixBestObjective']-x['objective']) for x in cy if x['feasible']),default=0)))
for target in [2,5]:
 fig,axs=plt.subplots(2,3,figsize=(15,8))
 for row,scenario in enumerate(['insertion','rotation']):
  for ax,key,label in zip(axs[row],['tipErrorN','contactErrorN','seconds'],['Tip force error [N]','Contact force error [N]','All-mode time [s]']):
   for policy,color,ls in [('conditional','#c36b20','--'),('exhaustive','#087f8c','-')]:
    r=data[(target,scenario,policy)];ax.plot(vec(r,'frame'),vec(r,key),ls,color=color,label=policy)
    bad=[x for x in r if not int(x['solverSuccess'])]
    if bad:ax.scatter(vec(bad,'frame'),vec(bad,key),marker='x',color='crimson',s=40,zorder=5)
   if key=='seconds':ax.set_yscale('symlog',linthresh=1)
   ax.set_xlabel(f'{scenario} frame');ax.set_ylabel(label);ax.grid(alpha=.2)
 axs[0,0].legend();fig.suptitle(f'{target} mm sensor tip noise: first acceptable versus exhaustive modes, mu=0.3\nSame observations and solver limits; each trajectory propagates its own estimates; red x = nonconverged/infeasible')
 fig.tight_layout();fig.savefig(OUT/f'comparison_{target}mm.png',dpi=160);plt.close(fig)
# Plot the known long failure against a slow but converged branch.
fig,axs=plt.subplots(1,3,figsize=(15,4))
examples=[(2,'rotation','conditional',3,1,'Failed sliding, 430 s'),(5,'rotation','conditional',15,1,'Converged sliding, 70 s')]
for key,label in [(x[:5],x[5]) for x in examples]:
 t=groups[key]
 for ax,field,ylabel in zip(axs,['trueViolation','trueFbgRms','seconds'],['Actual constraint violation','Normalized FBG RMS','Cumulative branch time [s]']):
  ax.plot(vec(t,'iteration'),vec(t,field),'-o',ms=3,label=label);ax.set_xlabel('Outer iteration');ax.set_ylabel(ylabel);ax.grid(alpha=.2)
axs[0].set_yscale('symlog',linthresh=1e-7);axs[0].axhline(1e-6,color='gray',ls=':');axs[1].axhline(1.24,color='gray',ls=':');axs[0].legend(fontsize=8)
fig.tight_layout();fig.savefig(OUT/'long_vs_converged.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,3,figsize=(13,7))
for row,ex in enumerate(examples):
 t=groups[ex[:5]]
 for ax,field,label in zip(axs[row],['radius','trueViolation','trueFbgRms'],['Trust radius','Nonlinear constraint violation','Normalized FBG RMS']):
  ax.plot(vec(t,'iteration'),vec(t,field),'-o',ms=3);ax.set_xlabel('Outer iteration');ax.set_ylabel(label);ax.grid(alpha=.2)
 axs[row,0].set_title(ex[5]);axs[row,1].set_yscale('symlog',linthresh=1e-7);axs[row,1].axhline(1e-6,color='gray',ls=':');axs[row,2].axhline(1.24,color='gray',ls=':')
fig.tight_layout();fig.savefig(OUT/'cycle_example.png',dpi=150);plt.close(fig)
# Union keys accommodate counterfactual fields on exhaustive rows only.
keys=list(dict.fromkeys(k for r in allframes for k in r))
with (OUT/'all_frames.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(allframes)
writecsv(OUT/'all_branches.csv',allbranches);writecsv(OUT/'outer_iterations.csv',alltrace);writecsv(OUT/'budget_prefixes.csv',budgets)
keys=list(dict.fromkeys(k for r in metrics for k in r))
with (OUT/'aggregate_metrics.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(metrics)
(OUT/'summary.json').write_text(json.dumps(dict(metrics=metrics,cycleAudit=cycles,validatedStates=valid,outerIterations=len(alltrace),sourceHashes=hashes,outcomeGroups=outcomeGroups,innerBudgetAudit=innerBudgets,budgetAudit='Retrospective prefix censorship, not new solver runs; iteration changes may alter later states.'),indent=2))
for r in metrics:print(r['targetMm'],r['scenario'],r['policy'],'tip/contact',r['tipRMSE_N'],r['contactRMSE_N'],'success',r['converged'],'fit',r['fitAccepted'],'time',r['meanSeconds'],flush=True)
print('CYCLES',cycles);print('Validated states',valid,'outputs',OUT)
