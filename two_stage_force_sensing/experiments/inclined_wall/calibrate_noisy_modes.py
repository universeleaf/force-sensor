"""Calibrate iid Bishop-channel Gaussian noise using the current kinematic integrator.
No mechanical inverse solve; no files outside this package. MATLAB cross-checks tips.
"""
from pathlib import Path
import sys,json
import numpy as np
import h5py
from scipy.interpolate import PchipInterpolator
from scipy.io import savemat
ROOT=Path(__file__).resolve().parents[2]
STAMP=sys.argv[1] if len(sys.argv)>1 else '20261011'
OUT=ROOT/'results/runs'/f'noisy_linearized_{STAMP}'
OUT.mkdir(parents=True,exist_ok=True)
assert not (OUT/'noise_payload.mat').exists(), 'Choose a fresh stamp; preserve existing noise.'
SOURCES={'insertion':ROOT/'results/runs/inclined_wall_bidirectional_20261008/inclined_wall/truth.mat','rotation':ROOT/'results/runs/inclined_wall_rotation_20261010/truth.mat'}

def tips(obs,arcs):
    # Exact algebra of reconstruct_bishop: segmented PCHIP, SE(3) midpoint steps.
    mids=np.arange(200)+.5; wm=np.empty((len(obs),200,2))
    for lo,hi in [(0,80),(80,201)]:
        nodes=(arcs>=lo)&(arcs<hi); take=(mids>=lo)&(mids<hi)
        wm[:,take,:]=PchipInterpolator(arcs[nodes],obs[:,nodes,:],axis=1,extrapolate=True)(mids[take])
    R=np.broadcast_to(np.eye(3),(len(obs),3,3)).copy(); pos=np.zeros((len(obs),3)); eye=np.eye(3)
    for j in range(200):
        v=np.zeros((len(obs),3)); v[:,0]=-wm[:,j,1];v[:,1]=wm[:,j,0];a=np.linalg.norm(v,axis=1)
        K=np.zeros_like(R);K[:,0,1]=-v[:,2];K[:,0,2]=v[:,1];K[:,1,0]=v[:,2];K[:,1,2]=-v[:,0];K[:,2,0]=-v[:,1];K[:,2,1]=v[:,0];K2=K@K
        A=np.sinc(a/np.pi);D=.5*np.sinc(a/(2*np.pi))**2;E=np.full_like(a,1/6);mask=a>1e-5
        E[mask]=(a[mask]-np.sin(a[mask]))/a[mask]**3;E[~mask]=1/6-a[~mask]**2/120
        V=eye+D[:,None,None]*K+E[:,None,None]*K2
        pos+=np.einsum('nij,nj->ni',R,V[:,:,2]);R=R@(eye+A[:,None,None]*K+D[:,None,None]*K2)
    return pos

D={}
for name,path in SOURCES.items():
    with h5py.File(path) as f:
        p=f['input/packet'];D[name]=(p['bishopCurvaturePerMm'][()],p['sFbgMm'][()].ravel())
base={name:tips(w,arcs) for name,(w,arcs) in D.items()}
N=256
mc={name:np.random.default_rng(12001+i).normal(size=(len(w),N,24,2)) for i,(name,(w,a)) in enumerate(D.items())}
def measure(sigma,draws):
    results={}
    for name,(w,arcs) in D.items():
        sq=[]
        for k in range(len(w)):
            e=tips(w[k][None]+sigma*draws[name][k],arcs)-base[name][k]
            sq.append(float(np.mean(np.sum(e*e,axis=1))))
        results[name]={'tipRmsMm':float(np.sqrt(np.mean(sq))),'perFrameTipRmsMm':np.sqrt(sq).tolist()}
    pooled=np.sqrt(np.mean([v['tipRmsMm']**2 for v in results.values()]))
    return float(pooled),results
ref,_=measure(1e-4,mc);print('REFERENCE pooled tip RMS for sigma=1e-4:',ref,flush=True)
validation={name:np.random.default_rng(22001+i).normal(size=(len(w),512,24,2)) for i,(name,(w,a)) in enumerate(D.items())}
rng=np.random.default_rng(33001);real={name:rng.normal(size=w.shape) for name,(w,a) in D.items()}
real['rotation'][0]=real['insertion'][20]
payload={name+'StandardNoise':v.transpose(2,1,0) for name,v in real.items()}
report={'interpretation':'RMS 3D tip displacement relative to noise-free geometric reconstruction; iid zero-mean Gaussian w1/w2 node noise; exact base pose; unchanged environment prior, no environment noise injected','calibrationSamplesPerFrame':N,'validationSamplesPerFrame':512,'calibrationWeighting':'Equal scenario weight, uniform frame weight within scenario','trajectorySeed':33001,'calibrationSeeds':[12001,12002],'validationSeeds':[22001,22002],'levels':{}}
for target in [2,5]:
    sigma=1e-4*target/ref
    got,_=measure(sigma,mc);sigma*=target/got
    got,details=measure(sigma,validation)
    report['levels'][str(target)]={'sigmaPerMm':sigma,'variancePerMm2':sigma*sigma,'validationPooledTipRmsMm':got,'scenarios':details}
    payload['sigma'+str(target)]=sigma
    for name,(w,arcs) in D.items():
        pred=tips(w+sigma*real[name],arcs);err=pred-base[name]
        payload[name+'LocalTips'+str(target)]=pred.T
        details[name]['realizationTipRmsMm']=float(np.sqrt(np.mean(np.sum(err*err,axis=1))))
        details[name]['realizationPerFrameTipErrorMm']=np.linalg.norm(err,axis=1).tolist()
    print('TARGET',target,'SIGMA',sigma,'VALIDATION',got,'SCENARIOS',[(n,d['tipRmsMm'],d['realizationTipRmsMm']) for n,d in details.items()],flush=True)
savemat(OUT/'noise_payload.mat',payload)
(OUT/'noise_calibration.json').write_text(json.dumps(report,indent=2))
print('SAVED',OUT,flush=True)
