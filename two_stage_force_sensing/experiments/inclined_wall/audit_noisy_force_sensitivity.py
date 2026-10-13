"""Offline local force observability of saved exact nonlinear IVP Jacobians.
No estimator changes. Covariance is conditional on fixed geometry/history/mode.
"""
from pathlib import Path
import csv,json
import h5py,numpy as np
from scipy.linalg import null_space,block_diag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'results/published/noisy_force_sensitivity_20261012';OUT.mkdir(parents=True,exist_ok=True)
SIG=3.7072566884243936e-4

def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def mat(g,k):return g[k][()].T
def scalar(g,k):return float(g[k][()].ravel()[0])
def array(g,k):return np.array([]) if g[k].attrs.get('MATLAB_empty',False) else g[k][()].ravel()
def readcsv(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def writecsv(p,rows):
 with p.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def gain(G,C):
 C=np.asarray(C);C=C/np.maximum(np.linalg.norm(C,axis=1,keepdims=True),1e-30)
 N=null_space(C) if len(C) else np.eye(G.shape[1]);H=G@N
 K=N@np.linalg.pinv(H);cov=SIG**2*K@K.T
 assert np.linalg.norm(C@K)*SIG<1e-7
 return K,cov,np.linalg.svd(H,compute_uv=False)
def profile(G,C,axis):
 e=np.eye(G.shape[1])[axis];D=np.vstack([C,e]);b=np.r_[np.zeros(len(C)),1]
 Dscale=np.maximum(np.linalg.norm(D,axis=1),1e-30);D=D/Dscale[:,None];b=b/Dscale
 d=np.linalg.pinv(D)@b;N=null_space(D);d+=N@np.linalg.lstsq(G@N,-G@d,rcond=None)[0]
 assert np.linalg.norm(D@d-b)<1e-7
 return d

rows=[];axes=[];examples=[];checks=[]
for sc in ['insertion','rotation']:
 cleanPath=ROOT/('results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth.mat' if sc=='insertion' else 'results/runs/inclined_wall_rotation_20261010/truth.mat')
 with h5py.File(cleanPath) as f:
  clean=f['input/packet/bishopCurvaturePerMm'][()];arcs=array(f['input/packet'],'sFbgMm');trueft=f['truth/tipForce'][()];truefc=f['truth/contactForce'][()];truep=f['truth/p'][()]
 with h5py.File(ROOT/f'results/runs/noisy_linearized_20261011/{sc}_2mm_truth.mat') as f:noisy=f['input/packet/bishopCurvaturePerMm'][()]
 truthRows=readcsv(cleanPath.with_name('truth_frames.csv'))
 score=[r for r in readcsv(ROOT/'results/published/noisy_mode_search_20261011/all_frames.csv') if r['scenario']==sc and r['targetMm']=='2' and r['policy']=='exhaustive']
 for source in ['clean','noisy']:
  tag=f'inclined_{sc}_linearized_20261011_v2' if source=='clean' else f'inclined_{sc}_noisy2mm_exhaustive_20261011'
  with h5py.File(ROOT/f'results/runs/{tag}/comparison.mat') as f:
   for k in range(len(clean)):
    if sc=='rotation' and k==0:continue
    out=f[f['runs'][k,0]];m=out['mechanics'];ev=m['evaluation'];sh=ev['shape'];g=out['geometry'];scale=array(m,'scale');x=array(sh,'x');mode=int(array(m,'mode')[0]) if array(m,'mode').size else 0
    J=mat(sh,'Jprediction')/scale;E=mat(sh,'JtipMoment')/scale;M=np.linalg.solve(E[:,:3],E[:,3:]);A=np.eye(3)
    if len(x)==9:A=np.column_stack([array(g,'normal'),mat(g,'B')])
    T=block_diag(np.eye(3),A) if len(x)==9 else np.eye(3)
    G=(J[:,3:]-J[:,:3]@M)@T.T
    Jq=sh['JcollisionP'][()].transpose(1,2,0).reshape(-1,len(x))/scale
    Q=(Jq[:,3:]-Jq[:,:3]@M)@T.T
    C=np.zeros((0,G.shape[1]));Cg=C.copy();P=np.zeros((3,G.shape[1]));slip=np.nan;cs=np.nan;margin=np.nan
    if len(x)==9:
     n=A[:,0];B=A[:,1:];JP=sh['JcontactP'][()][:,0,:].T/scale;P=(JP[:,3:]-JP[:,:3]@M)@T.T
     cs=scalar(g,'s');Cg=n[None,:]@P;v=array(ev,'slip');slip=np.linalg.norm(v);mu=scalar(g,'mu');lam=x[6];margin=mu*lam-np.linalg.norm(x[7:9])
     if mode==2 and slip>1e-9:
      d=v/slip;C=np.vstack([Cg,np.column_stack([np.zeros((2,3)),B.T+mu*d[:,None]*n])+mu*lam/slip*(np.eye(2)-np.outer(d,d))@B.T@P])
     elif mode==1:C=P.copy()
     else:C=np.column_stack([np.zeros((3,3)),np.eye(3)])
    K,V,sv=gain(G,C);Kg,Vg,sg=gain(G,Cg);Kfree,Vfree,sfree=gain(G,np.zeros((0,G.shape[1])))
    Vknown=SIG**2*np.linalg.pinv(G[:,:3].T@G[:,:3]);eps=(noisy[k]-clean[k]).ravel();pred=K@eps
    err=x[3:6]-trueft[k];fc=A@x[6:9] if len(x)==9 else np.zeros(3)
    r=dict(scenario=sc,source=source,frame=k+1,mode=mode,contactS=cs,distalSensors=int(sum(arcs>cs)) if np.isfinite(cs) else 24,slipMm=slip,coneMarginN=margin,converged=int(scalar(m,'converged')),feasible=int(scalar(m,'modeFeasible')),conditionMoment=float(np.linalg.cond(E[:,:3])),freeTipRmsN=float(np.sqrt(np.trace(Vfree[:3,:3]))),gapTipRmsN=float(np.sqrt(np.trace(Vg[:3,:3]))),modeTipRmsN=float(np.sqrt(np.trace(V[:3,:3]))),knownContactTipRmsN=float(np.sqrt(np.trace(Vknown))),predictedDrawTipErrorN=float(np.linalg.norm(pred[:3])),observedTipErrorN=float(np.linalg.norm(err)),observedContactErrorN=float(np.linalg.norm(fc-truefc[k])),shapeRmsMm=rms(np.linalg.norm(sh['p'][()]-truep[k],axis=1)),cleanObservationRms=rms(sh['prediction'][()]-clean[k]),correctMode=int(mode=={'free':0,'sliding':2,'sticking':1,'saved-insertion-state':2}[truthRows[k]['mode']] and (len(x)==9 and x[6]>1e-3)==(np.linalg.norm(truefc[k])>1e-3)),fbgRms=scalar(ev,'fbgRms'))
    for j,label in enumerate('XYZ'):r['tipStd'+label]=float(np.sqrt(V[j,j]));r['tipError'+label]=float(err[j]);r['predictedDraw'+label]=float(pred[j])
    rows.append(r)
    for j,label in enumerate('XYZ'):
     d=profile(G,C,j);signal=G@d;tiponly=np.eye(G.shape[1])[j]
     a=dict(scenario=sc,source=source,frame=k+1,mode=mode,axis=label,profileCurvatureRmsPerN=rms(signal),profileSignalNormPerN=float(np.linalg.norm(signal)),oneSigmaForceN=SIG/np.linalg.norm(signal),profileShapeRmsMmPerN=rms(np.linalg.norm((Q@d).reshape(-1,3),axis=1)),tipOnlyCurvatureRmsPerN=rms(G@tiponly),sameAxisTipContactCorrelation=float(np.dot(G[:,j],G[:,j+3])/(np.linalg.norm(G[:,j])*np.linalg.norm(G[:,j+3]))) if len(x)==9 else np.nan)
     for ix,nm in enumerate(['tipX','tipY','tipZ','contactX','contactY','contactZ'][:len(d)]):a[nm]=float(d[ix])
     for nm in ['contactX','contactY','contactZ']:a.setdefault(nm,0.)
     axes.append(a)
     if source=='clean' and j==0 and ((sc=='insertion' and k==23) or (sc=='rotation' and k==12)):
      physical=T.T@d;dz=np.r_[-M@physical,physical]/scale
      checks.append(dict(scenario=sc,frame=k+1,dz=dz.tolist(),Jdz=signal.tolist(),profile=a))
    if source=='noisy' and ((sc=='insertion' and k==23) or (sc=='rotation' and k in [2,12,20])):
     examples.append(dict(**r,tipEstimate=x[3:6].tolist(),contactEstimate=fc.tolist(),contactTrue=truefc[k].tolist(),noiseRms=rms(eps),cleanToEstimateSNR=float(np.linalg.norm(sh['prediction'][()].ravel()-clean[k].ravel())/SIG)))
(OUT/'derivative_inputs.json').write_text(json.dumps(checks,indent=2))
writecsv(OUT/'frame_sensitivity.csv',rows);writecsv(OUT/'direction_sensitivity.csv',axes)
summ=[]
for sc in ['insertion','rotation']:
 for source in ['clean','noisy']:
  rr=[r for r in rows if r['scenario']==sc and r['source']==source]
  for group in ['all','active','correct_converged']:
   sub=[r for r in rr if group=='all' or (group=='active' and r['mode']>0) or (group=='correct_converged' and r['correctMode'] and r['converged'] and r['feasible'])]
   s=dict(scenario=sc,source=source,group=group,frames=len(sub))
   for name in ['modeTipRmsN','gapTipRmsN','freeTipRmsN','knownContactTipRmsN','predictedDrawTipErrorN','observedTipErrorN']:s[name]=rms([r[name] for r in sub])
   for name in ['modeTipRmsN','tipStdX','tipStdY','tipStdZ']:s[name+'Median']=float(np.median([r[name] for r in sub]));s[name+'Max']=max(r[name] for r in sub)
   summ.append(s)
(OUT/'summary.json').write_text(json.dumps(dict(sigma=SIG,summary=summ,examples=examples),indent=2))
fig,axs=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
for i,sc in enumerate(['insertion','rotation']):
 rr=[r for r in rows if r['scenario']==sc and r['source']=='clean'];nn=[r for r in rows if r['scenario']==sc and r['source']=='noisy'];xx=[r['frame'] for r in rr]
 for key,label in [('modeTipRmsN','Local noise amplification: fixed mode'),('predictedDrawTipErrorN','Linear response to actual noise')]:axs[i,0].plot(xx,[r[key] for r in rr],label=label)
 axs[i,0].plot(xx,[r['observedTipErrorN'] for r in nn],'k--',label='Observed noisy estimate')
 axs[i,0].set(title=sc.capitalize(),ylabel='Tip force error / predicted RMS [N]',xlabel='Frame');axs[i,0].legend(fontsize=8)
 for label in 'XYZ':axs[i,1].plot(xx,[r['tipStd'+label] for r in rr],label=label)
 axs[i,1].set(title='Conditional component standard deviation',ylabel='Force standard deviation [N]',xlabel='Frame');axs[i,1].legend()
fig.savefig(OUT/'sensitivity.png',dpi=160);plt.close(fig)
print(json.dumps(dict(summary=summ,examples=examples),indent=2))
