"""Compare matched one-linearization/full-cone, octagonal QP, and outer-loop runs."""
from pathlib import Path
import csv,json,sys,hashlib
import numpy as np,h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];STAMP=sys.argv[1] if len(sys.argv)>1 else '20261012';OUT=ROOT/'results/published'/f'singlepass_comparison_{STAMP}';OUT.mkdir(parents=True,exist_ok=True)
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def arr(g,k):return np.array([]) if g[k].attrs.get('MATLAB_empty',False) else g[k][()].ravel()
def num(g,k):return float(arr(g,k)[0])
def rms(v):return float(np.sqrt(np.mean(np.asarray(v)**2)))
def write(p,r):
 if not r:return
 with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
rows=[];stats=[];branches=[];missing=[];polygonChecks=[]
for method in ['current','direct','polygon']:
 for level in [0,.5,1,2]:
  lev=str(level).replace('.','p') if level==.5 else str(int(level))
  for sc,T in [('insertion',41),('rotation',25)]:
   tag=f'singlepass_{method}_{sc}_{lev}mm_{STAMP}'
   if method=='current' and level==2:tag=f'inclined_{sc}_noisy2mm_exhaustive_20261011'
   pub=ROOT/'results/published'/tag;path=ROOT/'results/runs'/tag/'comparison.mat'
   if not (pub/'estimates.csv').exists():missing.append(tag);continue
   rr=read(pub/'estimates.csv')
   if len(rr)!=T:missing.append(tag);continue
   inp=ROOT/f'results/runs/single_pass_inputs_{STAMP}/{sc}_{lev}mm.mat'
   with h5py.File(inp) as tf:truth={key:tf['truth/'+key][()] for key in ['tipForce','contactForce','p']}
   tr=read(ROOT/('results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth_frames.csv' if sc=='insertion' else 'results/runs/inclined_wall_rotation_20261010/truth_frames.csv'))
   with h5py.File(path) as f:
    for k,r in enumerate(rr):
     if int(r['restoredSeed']):continue
     o=f[f['runs'][k,0]];m=o['mechanics'];ev=m['evaluation'];g=o['geometry'];sh=ev['shape'];ft=arr(sh,'x')[3:6];fc=sh['force'][()].sum(axis=0) if arr(sh,'force').size else np.zeros(3)
     assert np.isclose(np.linalg.norm(ft-truth['tipForce'][k]),float(r['tipErrorN']))
     trueMode={'free':0,'sliding':2,'sticking':1,'saved-insertion-state':2}[tr[k]['mode']];mode=int(arr(m,'mode')[0]) if arr(m,'mode').size else 0
     q={key:float(r[key]) for key in ['frame','seconds','geometrySeconds','mechanicsSeconds','iterations','ivps','rhsEvaluations','assignments','tipErrorN','contactErrorN','shapeRmsMm','shapeTipErrorMm','solverSuccess','converged','modeFeasible','modeViolation','fbgRms','curvatureRmsPerMm','gapMm','penetrationMm','tipMomentNmm','projectionN','coneExcessN','outerIterations','workNmm','estimatedArcMm','normalErrorDeg','stage1ShapeRmsMm','tangency']}
     q['diagnosticFlags']=r['flags']
     q.update(method=method,targetMm=level,scenario=sc,mode=mode,modeCorrect=int(mode==trueMode and (np.linalg.norm(fc)>1e-3)==(np.linalg.norm(truth['contactForce'][k])>1e-3)),truthContactN=float(np.linalg.norm(truth['contactForce'][k])),affineConverged=num(m,'affineConverged') if method!='current' else num(m,'converged'),affineFeasible=num(m,'affineFeasible') if method!='current' else num(m,'modeFeasible'),affineViolation=num(m,'affineViolation') if method!='current' else num(m,'modeViolation'),affineFbgRms=num(m,'affineFbgRms') if method!='current' else num(ev,'fbgRms'),trustBoundary=num(m,'atTrustBoundary') if method!='current' else 0.,auditSeconds=num(m,'auditSeconds') if method!='current' else 0.,linearizeSeconds=num(m,'linearizeSeconds') if method!='current' else 0.,auditFailed=num(m,'nonlinearAuditFailed') if method!='current' else 0.,circularViolation=num(m,'circularTrueViolation') if method!='current' else num(m,'modeViolation'))
     q['affineShapeRmsMm']=rms(np.linalg.norm(m['affineEvaluation/shape/p'][()]-truth['p'][k],axis=1)) if method!='current' else q['shapeRmsMm'];q['shapeLinearizationErrorMm']=rms(np.linalg.norm(m['affineEvaluation/shape/p'][()]-sh['p'][()],axis=1)) if method!='current' else 0.
     q['circularSlidingErrorN']=float('nan');q['affineCircularSlidingErrorN']=float('nan')
     if mode==2 and len(arr(sh,'x'))==9:
      xx=arr(sh,'x');vv=arr(ev,'slip');mu=float(arr(g,'mu')[0])
      if np.linalg.norm(vv)>1e-8:q['circularSlidingErrorN']=float(np.linalg.norm(xx[7:9]+mu*xx[6]*vv/np.linalg.norm(vv)))
      if method!='current':
       vv=arr(m['affineEvaluation'],'slip')
       if np.linalg.norm(vv)>1e-8:q['affineCircularSlidingErrorN']=float(np.linalg.norm(xx[7:9]+mu*xx[6]*vv/np.linalg.norm(vv)))
     q['activeGapMm']=q['gapMm'] if mode>0 else 0.
     q['circularFeasible']=int(q['circularViolation']<=1e-6)
     rows.append(q)
     for j,ref in enumerate(m['attempts'][()].ravel()):
      a=f[ref];branches.append(dict(method=method,targetMm=level,scenario=sc,frame=k+1,attempt=j+1,selected=int(j+1==int(r['selectedAttempt'])),iterations=num(a,'iterations'),converged=num(a,'converged'),feasible=num(a,'modeFeasible'),violation=num(a,'modeViolation'),seconds=num(a,'seconds')))
      if method=='polygon' and num(a,'modeFeasible'):
       ae=a['affineEvaluation'];ax=arr(ae['shape'],'x');codes=arr(a,'enumeratedMode').astype(int);v=ae['slip'][()];mu=arr(g,'mu')
       if len(codes):
        for ic,code in enumerate(codes):
         lam=ax[6+3*ic];tau=ax[7+3*ic:9+3*ic];slip=v[ic];vertices=mu[ic]*lam*np.column_stack([np.cos(np.arange(8)*np.pi/4),np.sin(np.arange(8)*np.pi/4)]);diss=float(tau@slip-np.min(vertices@slip));excess=float(np.linalg.norm(tau)-mu[ic]*lam)
         polygonChecks.append(dict(scenario=sc,targetMm=level,frame=k+1,attempt=j+1,code=int(code),dissipationGapNmm=diss,circularConeExcessN=excess));assert excess<5e-5;assert abs(diss)<5e-4
   sub=[x for x in rows if x['method']==method and x['targetMm']==level and x['scenario']==sc];n=len(sub)
   s=dict(method=method,targetMm=level,scenario=sc,frames=n,tipRMSE_N=rms([x['tipErrorN'] for x in sub]),contactRMSE_N=rms([x['contactErrorN'] for x in sub]),contactTruthRMS_N=rms([x['truthContactN'] for x in sub]),shapeRMS_mm=rms([x['shapeRmsMm'] for x in sub]),affineShapeRMS_mm=rms([x['affineShapeRmsMm'] for x in sub]),shapeTipRMS_mm=rms([x['shapeTipErrorMm'] for x in sub]),meanSeconds=float(np.mean([x['seconds'] for x in sub])),medianSeconds=float(np.median([x['seconds'] for x in sub])),maxSeconds=max(x['seconds'] for x in sub),meanIterations=float(np.mean([x['iterations'] for x in sub])),meanIVPs=float(np.mean([x['ivps'] for x in sub])),meanAssignments=float(np.mean([x['assignments'] for x in sub])),nativeConverged=sum(x['converged'] for x in sub),affineSuccess=sum(x['affineConverged'] and x['affineFeasible'] for x in sub),nonlinearSuccess=sum(x['solverSuccess'] for x in sub),modeCorrect=sum(x['modeCorrect'] for x in sub),maxGapMm=max(x['gapMm'] for x in sub),maxActiveGapMm=max(x['activeGapMm'] for x in sub),medianActiveGapMm=float(np.median([x['activeGapMm'] for x in sub if x['mode']>0])) if any(x['mode']>0 for x in sub) else 0.,circularFeasible=sum(x['circularFeasible'] for x in sub),maxPenetrationMm=max(x['penetrationMm'] for x in sub),maxTipMomentNmm=max(x['tipMomentNmm'] for x in sub),maxCircularViolation=max(x['circularViolation'] for x in sub),trustBoundaryFrames=sum(x['trustBoundary'] for x in sub),auditFailed=sum(x['auditFailed'] for x in sub),meanAuditSeconds=float(np.mean([x['auditSeconds'] for x in sub])))
   s['relativeContactErrorPercent']=100*s['contactRMSE_N']/s['contactTruthRMS_N'];stats.append(s)
write(OUT/'summary.csv',stats);write(OUT/'frames.csv',rows);write(OUT/'branches.csv',branches);write(OUT/'polygon_law_checks.csv',polygonChecks)
(OUT/'summary.json').write_text(json.dumps(dict(summary=stats,missing=missing,newMethodsComplete=sum(s['method']!='current' for s in stats)==16,referenceScope='User approved reporting completed tests against existing baselines; optional 0.5/1 mm controls stopped',polygonLawChecks=len(polygonChecks)),indent=2))
fig,axs=plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
for i,sc in enumerate(['insertion','rotation']):
 for j,(key,label) in enumerate([('tipRMSE_N','Tip force RMS error [N]'),('contactRMSE_N','Contact force RMS error [N]'),('meanSeconds','Mean seconds / frame')]):
  for method,color in [('current','#222222'),('direct','#007c91'),('polygon','#cc6c20')]:
   ss=sorted([s for s in stats if s['scenario']==sc and s['method']==method],key=lambda s:s['targetMm']);axs[i,j].plot([s['targetMm'] for s in ss],[s[key] for s in ss],('x' if method=='current' and len(ss)<4 else '-o'),label={'current':'Outer-loop circular','direct':'One-pass circular','polygon':'One-pass octagon'}[method],color=color)
  axs[i,j].set(xlabel='Geometric tip-noise RMS [mm]',ylabel=label,title=sc.capitalize());axs[i,j].legend();axs[i,j].grid(alpha=.2)
  if j==2:axs[i,j].set_yscale('log')
fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)

if sum(s['method']!='current' for s in stats)==16:
 assert len([s for s in stats if s['method']!='current'])==16
 for lv in [0,.5,1,2]:
  for sc in ['insertion','rotation']:
   ref=[r for r in rows if r['method']=='direct' and r['targetMm']==lv and r['scenario']==sc]
   for mt in ['current','polygon']:
    other=[r for r in rows if r['method']==mt and r['targetMm']==lv and r['scenario']==sc]
    for key in ['estimatedArcMm','normalErrorDeg','stage1ShapeRmsMm','tangency']:
     assert np.allclose([r[key] for r in ref],[r[key] for r in other],rtol=1e-6,atol=1e-7,equal_nan=True),(lv,sc,mt,key)
 assert all(s['frames']==(41 if s['scenario']=='insertion' else 24) for s in stats)
 assert all(s['meanIVPs']==2 for s in stats if s['method']!='current')
 # Nonlinear audit failures are results to report, not a reason to discard a run.
print(json.dumps(dict(summary=stats,missing=missing,polygonLawChecks=len(polygonChecks)),indent=2))
