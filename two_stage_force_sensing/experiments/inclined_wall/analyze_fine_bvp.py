"""Compare five-times-finer one-pass/BVP trajectories with original step sizes."""
from pathlib import Path
import csv,json,sys
import numpy as np,h5py
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];STAMP=sys.argv[1] if len(sys.argv)>1 else '20261012_step5';OUT=ROOT/'results/published'/f'fine_bvp_comparison_{STAMP}';OUT.mkdir(parents=True,exist_ok=True)
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def arr(g,k):return np.array([]) if g[k].attrs.get('MATLAB_empty',False) else g[k][()].ravel()
def num(g,k):return float(arr(g,k)[0])
def txt(g,k):return ''.join(chr(int(c)) for c in arr(g,k))
def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def write(p,rows):
 if rows:
  with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def table(h,rows):return '\n'.join(['| '+' | '.join(h)+' |','|'+'|'.join(['---']*len(h))+'|']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def fm(x):return f'{x:.4g}'
old=json.loads((ROOT/'results/published/singlepass_bvp_comparison_20261012_fixedload_v2/summary.json').read_text())['summary'];old={(s['scenario'],s['targetMm']):s for s in old}
noise=loadmat(ROOT/'results/runs'/f'fine_bvp_inputs_{STAMP}/noise_payload.mat');coarseNoise=loadmat(ROOT/'results/runs/noisy_linearized_20261011/noise_payload.mat')
for sc in ['insertion','rotation']:assert np.array_equal(noise[sc+'StandardNoise'][:,:,::5],coarseNoise[sc+'StandardNoise'])
rows=[];stats=[];branches=[];missing=[];cleanTips={};truthAudit=[]
for sc,T in [('insertion',201),('rotation',121)]:
 tf=ROOT/'results/runs'/f'fine_bvp_{sc}_truth_{STAMP}'/'truth_frames.csv'
 if tf.exists():
  tr=read(tf);truthAudit.append(dict(scenario=sc,frames=len(tr),expected=T,totalSeconds=sum(float(r['seconds']) for r in tr),maxPenetrationMm=max(float(r.get('penetrationMm',0)) for r in tr),maxProjectionN=max(float(r.get('projectionN',0)) for r in tr),maxPositiveWorkNmm=max([0]+[float(r.get('workNmm',0)) for r in tr])))
for target in [0,.5,1,2]:
 lev=str(target).replace('.','p') if target==.5 else str(int(target))
 for sc,T in [('insertion',201),('rotation',121)]:
  tag=f'fine_bvp_{sc}_{lev}mm_{STAMP}';pub=ROOT/'results/published'/tag;raw=ROOT/'results/runs'/tag/'comparison.mat'
  if not (pub/'estimates.csv').exists() or not raw.exists():missing.append(tag);continue
  rr=read(pub/'estimates.csv')
  if len(rr)!=T:missing.append(tag);continue
  with h5py.File(raw) as f:
   if not isinstance(f[f['runs'][-1,0]],h5py.Group):missing.append(tag);continue
   oldrows=read(ROOT/'results/published'/f'singlepass_bvp_{sc}_{lev}mm_20261012_fixedload_v2'/'estimates.csv')
   with h5py.File(ROOT/'results/runs'/f'fine_bvp_inputs_{STAMP}'/f'{sc}_{lev}mm.mat') as tf:
    truth={k:tf['truth/'+k][()] for k in ['p','contactForce','tipForce']};obs=tf['input/packet/bishopCurvaturePerMm'][()];sigma=float(tf['sigma'][()].item())
   with h5py.File(ROOT/'results/runs'/f'fine_bvp_{sc}_truth_{STAMP}'/'truth.mat') as tf:clean=tf['input/packet/bishopCurvaturePerMm'][()]
   assert np.allclose(obs-clean,sigma*noise[sc+'StandardNoise'].transpose(2,1,0),rtol=1e-12,atol=1e-16)
   geoTips=np.empty((T,3))
   for k,r in enumerate(rr):
    o=f[f['runs'][k,0]];m=o['mechanics'];g=o['geometry'];geoTips[k]=g['kinematicShape/p'][()][-1]
    if int(r['restoredSeed']):continue
    assert float(r['assignments'])==float(r['possibleAssignments']) and float(r['outerIterations'])==0
    e=m['evaluation'];sh=e['shape'];b=m['bvpCorrection'];pre=m['beforeBvp'];mode=arr(m,'mode');active=np.any(mode>0)
    assert np.array_equal(arr(pre,'z')[3:],arr(m,'z')[3:]),(tag,k,'forces changed')
    assert np.isclose(np.linalg.norm(arr(sh,'x')[3:6]-truth['tipForce'][k]),float(r['tipErrorN']))
    q={key:float(r[key]) for key in ['frame','seconds','geometrySeconds','mechanicsSeconds','iterations','ivps','rhsEvaluations','assignments','tipErrorN','contactErrorN','shapeRmsMm','shapeTipErrorMm','modeViolation','modeFeasible','converged','solverSuccess','fbgRms','curvatureRmsPerMm','tipMomentNmm','penetrationMm','workNmm','coneExcessN','stage1TipErrorMm','stage1ShapeRmsMm','normalErrorDeg','arcErrorMm','selectedIterations']}
    q.update(scenario=sc,targetMm=target,commonFrame=int(k%5==0),mode=str(mode.astype(int).tolist()),flags=r['flags'],activeGapMm=float(r['gapMm']) if active else 0,preMomentNmm=num(pre,'tipMomentNmm'),preActiveGapMm=float(np.max(np.abs(arr(pre,'gapMm')))) if active else 0,preFbgRms=num(pre,'fbgRms'),preShapeRmsMm=rms(np.linalg.norm(pre['p'][()]-truth['p'][k],axis=1)),slipMm=float(np.linalg.norm(arr(e,'slip'))),affineSuccess=int(num(m,'affineConverged') and num(m,'affineFeasible')),bvpSuccess=num(b,'converged'),bvpFailed=num(b,'failed'),bvpSeconds=num(b,'seconds'),bvpRhs=num(b,'rhsEvaluations'),bvpNodes=num(b,'finalNodes'),bvpIvPDisagreementMm=num(b,'ivpBvpShapeRmsMm') if 'ivpBvpShapeRmsMm' in b else float('nan'),bvpException=txt(b,'exception'),bvpWarning=txt(b,'warning'),sameFrameForceChangeN=num(b,'forceChangeN'),oldTipErrorN=float(oldrows[k//5]['tipErrorN']) if k%5==0 else float('nan'),oldContactErrorN=float(oldrows[k//5]['contactErrorN']) if k%5==0 else float('nan'))
    rows.append(q)
    for j,ref in enumerate(m['attempts'][()].ravel()):
     a=f[ref];branches.append(dict(scenario=sc,targetMm=target,frame=k+1,attempt=j+1,selected=int(j+1==int(r['selectedAttempt'])),iterations=num(a,'iterations'),converged=num(a,'converged'),feasible=num(a,'modeFeasible'),seconds=num(a,'seconds')))
  if target==0:cleanTips[sc]=geoTips
  noiseTip=rms(np.linalg.norm(geoTips-cleanTips[sc],axis=1)) if sc in cleanTips else float('nan')
  sub=[r for r in rows if r['scenario']==sc and r['targetMm']==target];common=[r for r in sub if r['commonFrame']];prev=old[(sc,target)]
  summary=dict(scenario=sc,targetMm=target,frames=len(sub),tipRMSE_N=rms([r['tipErrorN'] for r in sub]),contactRMSE_N=rms([r['contactErrorN'] for r in sub]),shapeRMS_mm=rms([r['shapeRmsMm'] for r in sub]),shapeTipRMS_mm=rms([r['shapeTipErrorMm'] for r in sub]),maxTipErrorN=max(r['tipErrorN'] for r in sub),maxContactErrorN=max(r['contactErrorN'] for r in sub),maxShapeRmsMm=max(r['shapeRmsMm'] for r in sub),oldMaxTipErrorN=max(float(r['tipErrorN']) for r in oldrows if not int(r['restoredSeed'])),oldMaxContactErrorN=max(float(r['contactErrorN']) for r in oldrows if not int(r['restoredSeed'])),oldMaxShapeRmsMm=max(float(r['shapeRmsMm']) for r in oldrows if not int(r['restoredSeed'])),meanSeconds=float(np.mean([r['seconds'] for r in sub])),medianSeconds=float(np.median([r['seconds'] for r in sub])),maxSeconds=max(r['seconds'] for r in sub),totalSolveSeconds=sum(r['seconds'] for r in sub),meanBvpSeconds=float(np.mean([r['bvpSeconds'] for r in sub])),meanIVPs=float(np.mean([r['ivps'] for r in sub])),totalIVPs=sum(r['ivps'] for r in sub),meanIterations=float(np.mean([r['iterations'] for r in sub])),affineSuccess=sum(r['affineSuccess'] for r in sub),bvpSuccess=sum(r['bvpSuccess'] for r in sub),bvpFailures=sum(r['bvpFailed'] for r in sub),fbgAboveAcceptance=sum(r['fbgRms']>1.24 for r in sub),maxFbgRms=max(r['fbgRms'] for r in sub),fullModeSuccess=sum(r['solverSuccess'] for r in sub),maxSelectedIterations=max(r['selectedIterations'] for r in sub),selectedAtCap=sum(r['selectedIterations']>=100 for r in sub),maxTipMomentNmm=max(r['tipMomentNmm'] for r in sub),maxPreMomentNmm=max(r['preMomentNmm'] for r in sub),maxActiveGapMm=max(r['activeGapMm'] for r in sub),maxPreActiveGapMm=max(r['preActiveGapMm'] for r in sub),maxPenetrationMm=max(r['penetrationMm'] for r in sub),positiveWorkFrames=sum(r['workNmm']>1e-8 for r in sub),maxPositiveWorkNmm=max([0]+[r['workNmm'] for r in sub]),maxBvpIvPDisagreementMm=max(r['bvpIvPDisagreementMm'] for r in sub),maxBvpNodes=max(r['bvpNodes'] for r in sub),realizedGeometricTipNoiseRMS_mm=noiseTip,stage1ShapeRMS_mm=rms([r['stage1ShapeRmsMm'] for r in sub]),stage1NormalRMS_deg=rms([r['normalErrorDeg'] for r in sub]),commonFrames=len(common),commonTipRMSE_N=rms([r['tipErrorN'] for r in common]),commonContactRMSE_N=rms([r['contactErrorN'] for r in common]),oldTipRMSE_N=prev['tipRMSE_N'],oldContactRMSE_N=prev['contactRMSE_N'],oldShapeRMS_mm=prev['shapeRMS_mm'],oldMeanSeconds=prev['meanSeconds'],oldTotalSolveSeconds=prev['meanSeconds']*prev['frames'],oldMaxTipMomentNmm=prev['maxTipMomentNmm'],oldMaxActiveGapMm=prev['maxActiveGapMm'],oldFullModeSuccess=prev['nonlinearContactSuccess'],oldFrames=prev['frames'],oldPositiveWorkFrames=prev['positiveFrictionWorkFrames'])
  stats.append(summary)
write(OUT/'summary.csv',stats);write(OUT/'frames.csv',rows);write(OUT/'branches.csv',branches);write(OUT/'truth_audit.csv',truthAudit)
complete=len(stats)==8;info=dict(summary=stats,missing=missing,complete=complete,totalFrames=len(rows),truthAudit=truthAudit);(OUT/'summary.json').write_text(json.dumps(info,indent=2))
if not complete:print('Complete trajectories:',len(stats),'/8; frame solves:',len(rows));sys.exit(0)
assert len(rows)==1284
assert all(s['commonFrames']==s['oldFrames'] for s in stats)
fig,axs=plt.subplots(2,4,figsize=(16,8),constrained_layout=True)
for i,sc in enumerate(['insertion','rotation']):
 ss=sorted([s for s in stats if s['scenario']==sc],key=lambda s:s['targetMm']);lv=[s['targetMm'] for s in ss]
 for j,(key,oldkey,label) in enumerate([('tipRMSE_N','oldTipRMSE_N','Tip force RMSE [N]'),('contactRMSE_N','oldContactRMSE_N','Contact force RMSE [N]'),('shapeRMS_mm','oldShapeRMS_mm','Shape RMS [mm]'),('meanSeconds','oldMeanSeconds','Mean seconds/frame')]):
  ax=axs[i,j];ax.plot(lv,[s[oldkey] for s in ss],'-o',label='Original steps + BVP');ax.plot(lv,[s[key] for s in ss],'-o',label='5x finer steps + BVP');ax.set(xlabel='Tip-noise RMS target [mm]',ylabel=label,title=sc.capitalize());ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
report=['# Five-times-finer single-pass rod plus fixed-load BVP','','Completed report date: 2026-10-13 (run stamp preserves the start date). Branch codex/two-stage-force-sensing, base b376013. Eight complete trajectories / 1284 new solves. All outputs are separate; no commit or push.','',
'Insertion distance step is 0.4 mm (previously 2 mm), producing 201 states over 80->40->80 mm. Rotation step is 0.75 degrees (previously 3.75), producing 121 states over 0->45->0 degrees; its first restored insertion state is excluded from scores. Both reference trajectories were regenerated with their actual finer-step friction histories using the existing collaborator algorithms read-only. No interpolation of ground truth was used. Finer-step friction history can change the physical reference itself, especially around reversal.','',
'The inverse method is unchanged: one rod linearization; exhaustive full circular separation/sticking/sliding search; maximum 100 SQP iterations per branch; one fixed-load bvp4c call; independent nonlinear IVP validation; corrected history passed forward. No outer relinearization/force-correction loop was added. Constraint tolerance 1e-6; BVP scaled RelTol 1e-7, AbsTol 1e-9; no force priors or trust box. Geometry/collision spatial discretization remains unchanged. Timestamps remain 0.02 s apart as quasistatic history bookkeeping; this test changes actuation increment, not a validated dynamic model.','',
'Gaussian Bishop curvature standard deviations remain 0, 9.26814e-5, 1.85363e-4, 3.70726e-4 /mm for 0/0.5/1/2 mm targets. Zero injected noise retains measurement covariance std 1e-4 /mm. At common actuation samples the original seed33001 draws are reused exactly; additional samples use independent seed33005 Gaussian draws. Restored rotation noise equals insertion noise at maximum insertion. No averaging or interpolation of noise was performed; environment and mu noise remain absent.','',
'Checkpoint serialization occurs every ten frames and at completion, retaining all frame states, branches and diagnostic values. Redundant dense IVP interpolation objects are omitted after diagnostics, as the existing runner already did for final shapes. This affects storage, not numerical execution. Reported solve times exclude checkpoint serialization and forward truth generation.','',
'## Accuracy and runtime','',
'Errors are RMS vector norms; shape error pools centerline sample points and frames. Coarse values refer to the previous fixed-load BVP method. Fine values use all fine frames.','']
report+=[table(['Motion','Noise mm','Coarse tip N','Fine tip N','Coarse contact N','Fine contact N','Coarse shape mm','Fine shape mm','Coarse s/frame','Fine s/frame'],[[s['scenario'],s['targetMm'],*[fm(s[k]) for k in ['oldTipRMSE_N','tipRMSE_N','oldContactRMSE_N','contactRMSE_N','oldShapeRMS_mm','shapeRMS_mm','oldMeanSeconds','meanSeconds']]] for s in stats]),'',
'## Matched actuation samples and noise realization','',
'This table evaluates the fine trajectory only at every fifth sample, corresponding to the coarse actuation positions and identical noise draws. Histories and reference contact states may still differ. Realized geometric noise is endpoint displacement between noisy and clean geometric reconstructions on the fine trajectory.','',
table(['Motion','Noise mm','Common samples','Coarse tip N','Fine common tip N','Coarse contact N','Fine common contact N','Realized tip-noise RMS mm'],[[s['scenario'],s['targetMm'],s['commonFrames'],*[fm(s[k]) for k in ['oldTipRMSE_N','commonTipRMSE_N','oldContactRMSE_N','commonContactRMSE_N','realizedGeometricTipNoiseRMS_mm']]] for s in stats]),'',
'## Peak errors','',
table(['Motion','Noise mm','Coarse peak tip N','Fine peak tip N','Coarse peak contact N','Fine peak contact N','Coarse peak shape mm','Fine peak shape mm'],[[s['scenario'],s['targetMm'],*[fm(s[k]) for k in ['oldMaxTipErrorN','maxTipErrorN','oldMaxContactErrorN','maxContactErrorN','oldMaxShapeRmsMm','maxShapeRmsMm']]] for s in stats]),'',
'Peak errors are reported because a smaller step does not necessarily remove a friction-force jump at reversal; averaging over more samples can reduce trajectory RMS without eliminating the peak. Noise maxima also depend on sample count.','',
'## Convergence and physical consistency','',
table(['Motion','Noise mm','Affine success','BVP success','Full mode success','Max terminal moment N mm','Max active gap mm','Max penetration mm','Positive-work frames'],[[s['scenario'],s['targetMm'],f"{int(s['affineSuccess'])}/{s['frames']}",f"{int(s['bvpSuccess'])}/{s['frames']}",f"{int(s['fullModeSuccess'])}/{s['frames']}",fm(s['maxTipMomentNmm']),fm(s['maxActiveGapMm']),fm(s['maxPenetrationMm']),s['positiveWorkFrames']] for s in stats]),'',
'The normalized FBG acceptance level remains 1.24. Exhaustive search still returns the lowest-objective affine-feasible candidate when no candidate meets all checks; diagnostics do not stop the step. FBG threshold exceedances are recorded in summary.csv. Full mode success requires native convergence and original nonlinear selected-mode constraints at tolerance 1e-6. BVP convergence alone enforces the prescribed-load equilibrium boundary problem. Gap/slip/friction remain diagnostic after correction. Positive-work counts use the existing displacement-based 1e-8 N mm diagnostic threshold. All exceptions and warnings are preserved in frames.csv.','',
'BVP nonconvergence is counted separately from thrown exceptions: a bvp4c mesh-limit warning can return an approximate state without raising an exception. The current wrapper still audits and passes that state into history, while marking convergence false. Such frames are retained in every accuracy statistic; no fallback or retry was introduced in this experiment.','',
'## Cost and iteration cap','',
table(['Motion','Noise mm','Fine total solve s','Coarse total solve s','IVPs/frame','SQP iterations/frame','Selected cap frames','Mean BVP s'],[[s['scenario'],s['targetMm'],fm(s['totalSolveSeconds']),fm(s['oldTotalSolveSeconds']),fm(s['meanIVPs']),fm(s['meanIterations']),s['selectedAtCap'],fm(s['meanBvpSeconds'])] for s in stats]),'',
f"Total branch solves: {len(branches)}; all branches at iteration cap: {sum(b['iterations']>=100 for b in branches)}. Exhaustive enumeration with capped or locally solved branches is not a global certificate. All same-frame force corrections preserve loads exactly. Independent IVP/BVP shape agreement and Stage 1 accuracy are included in summary.csv.",'',
'## Interpretation limits','',
'Smaller actuation steps reduce deterministic state changes, but iid sensor noise retains its per-sample magnitude. Consequently noisy changes between consecutive observations do not shrink in proportion to the actuation step, and their effect on estimated slip may grow relative to actual motion. This experiment retains that noise model deliberately. One trajectory realization per noise level cannot establish a statistical ranking. Joint contact/force correction and temporal smoothing were not added.','',
'At common actuation points, regenerated insertion truth forces match the coarse reference exactly. Rotation truth differs by 0.205 N vector RMS (0.373 N maximum), due to the finer friction history. Error metrics always use the corresponding physical reference. For iid channel noise, the covariance of successive noise differences remains twice the per-frame covariance; reducing actuation increments does not reduce this component.','',
'A diagnostic example at 0.5 mm noise: fine rotation frame91 (22.5 degrees on return) selects the correct sticking mode but has 12.675 N tip error, versus 1.636 N at the shared coarse sample with the same noise. Its normalized FBG RMS is already 5.623 in the affine solve, 5.669 before BVP, and 5.476 after BVP. Thus this outlier is not explained solely by BVP nonlinearity or wrong mode classification; retained history constraints and the existing capped/local mode solves also matter.','',
'## Failed corrections retained in the results','',
table(['Motion','Noise mm','Frame','Shape RMS mm','Terminal moment N mm','Failure'],[[r['scenario'],r['targetMm'],int(r['frame']),fm(r['shapeRmsMm']),fm(r['tipMomentNmm']),r['bvpException'] or r['bvpWarning'].replace('\n',' ')] for r in rows if not r['bvpSuccess']]),'',
'## Findings','',
'Noiseless rotation improves strongly with smaller steps; insertion contact/shape errors also improve. Noiseless insertion tip RMSE on all fine positions rises slightly because a new intermediate position has an isolated error spike, while matched-position tip RMSE improves. At 0.5/1 mm noise, matched-position insertion force errors remain almost unchanged; shape improves. Noisy rotation is mixed, and 2 mm runs exhibit serious failures. A smaller actuation increment is therefore useful for deterministic linearization accuracy but is not a sufficient noise-robustness measure for this recurrent estimator.','',
'At 2 mm noise, insertion frame173 (68.8 mm wall distance on return) and rotation frame72 (36.75 degrees on return) reach the BVP mesh limit. Rotation frame78 (32.25 degrees) throws a singular-collocation-Jacobian exception and retains the pre-BVP estimate. Warning-returned states are passed onward by the unchanged wrapper, so error propagation is possible. These failures are included rather than removed. No statistical generalization is claimed from a single realization.','',
'## Videos and plots','',
f'All eight cases: [media index](../../results/published/fine_bvp_comparison_{STAMP}/media/README.md). Each case includes an MP4 with XYZ force components and correction status, force plots, errors/timing/physics plots, Stage 1 plots, representative frames and a compact rendering source. Every saved frame is included; failures remain visible.','',
'## Reproduction and outputs','',
"Run setup_tsfs; addpath('experiments/inclined_wall'); run_fine_bvp_experiment('20261012_step5'). Then python experiments/inclined_wall/analyze_fine_bvp.py 20261012_step5. The runner skips completed outputs and rejects partial inverse trajectories. To rerun under a fresh stamp, copy noise_payload.mat/noise_protocol.json from the published comparison reproduction folder to the corresponding fresh input folder; numerical reference and inverse outputs must remain separate.",'',
f'Combined output: results/published/fine_bvp_comparison_{STAMP}/. Raw trajectories: results/runs/fine_bvp_*_{STAMP}/comparison.mat. Truth folders: results/runs/fine_bvp_{{insertion,rotation}}_truth_{STAMP}/. Inputs: results/runs/fine_bvp_inputs_{STAMP}/. Main log: fine_bvp_{STAMP}.log.','']
(ROOT/'docs/reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md').write_text('\n'.join(report));print(json.dumps(info,indent=2))
