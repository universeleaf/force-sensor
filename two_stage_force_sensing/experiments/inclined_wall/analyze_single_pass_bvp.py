"""Score separate fixed-load BVP trajectories against preserved single-pass results."""
from pathlib import Path
import csv,json,sys
import numpy as np,h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
STAMP=sys.argv[1] if len(sys.argv)>1 else '20261012_fixedload_v2'
OUT=ROOT/'results/published'/f'singlepass_bvp_comparison_{STAMP}';OUT.mkdir(parents=True,exist_ok=True)
def read(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def arr(g,k):return np.array([]) if g[k].attrs.get('MATLAB_empty',False) else g[k][()].ravel()
def num(g,k):return float(arr(g,k)[0])
def text(g,k):return ''.join(chr(int(c)) for c in arr(g,k))
def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def write(p,rows):
 if not rows:return
 with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','|'+'|'.join(['---']*len(headers))+'|']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def fmt(x):return f'{x:.4g}'
old=json.loads((ROOT/'results/published/singlepass_comparison_20261012_unbounded/summary.json').read_text())['summary'];old={(s['scenario'],s['targetMm']):s for s in old if s['method']=='direct'}
frames=[];stats=[];branches=[];missing=[]
for target in [0,.5,1,2]:
 lev=str(target).replace('.','p') if target==.5 else str(int(target))
 for scenario,T in [('insertion',41),('rotation',25)]:
  tag=f'singlepass_bvp_{scenario}_{lev}mm_{STAMP}';pub=ROOT/'results/published'/tag;raw=ROOT/'results/runs'/tag/'comparison.mat'
  if not (pub/'estimates.csv').exists():missing.append(tag);continue
  rr=read(pub/'estimates.csv')
  if len(rr)!=T:missing.append(tag);continue
  original=read(ROOT/'results/published'/f'singlepass_direct_{scenario}_{lev}mm_20261012_unbounded'/'estimates.csv')
  with h5py.File(ROOT/'results/runs/single_pass_inputs_20261012_unbounded'/f'{scenario}_{lev}mm.mat') as tf:truth={key:tf['truth/'+key][()] for key in ['p','tipForce','contactForce']}
  with h5py.File(raw) as f:
   for k,r in enumerate(rr):
    if int(r['restoredSeed']):continue
    out=f[f['runs'][k,0]];m=out['mechanics'];b=m['bvpCorrection'];pre=m['beforeBvp'];e=m['evaluation'];sh=e['shape'];mode=arr(m,'mode');active=np.any(mode>0)
    assert np.array_equal(arr(pre,'z')[3:],arr(m,'z')[3:]),(tag,k,'loads changed')
    assert num(b,'forceChangeN')==0
    assert np.isclose(np.linalg.norm(arr(sh,'x')[3:6]-truth['tipForce'][k]),float(r['tipErrorN']))
    for key in ['estimatedArcMm','normalErrorDeg','stage1ShapeRmsMm','tangency']:
     if key!='tangency':assert np.isclose(float(r[key]),float(original[k][key]),rtol=1e-6,atol=1e-7,equal_nan=True),(tag,k,key)
    q={key:float(r[key]) for key in ['frame','seconds','geometrySeconds','mechanicsSeconds','iterations','ivps','rhsEvaluations','assignments','tipErrorN','contactErrorN','shapeRmsMm','shapeTipErrorMm','converged','solverSuccess','modeFeasible','modeViolation','fbgRms','curvatureRmsPerMm','penetrationMm','tipMomentNmm','projectionN','workNmm','coneExcessN']}
    q.update(scenario=scenario,targetMm=target,mode=str(mode.astype(int).tolist()),modeChangedVsOld=int(r['mode']!=original[k]['mode']),flags=r['flags'],activeGapMm=float(r['gapMm']) if active else 0,preShapeRmsMm=rms(np.linalg.norm(pre['p'][()]-truth['p'][k],axis=1)),preTipPositionErrorMm=float(np.linalg.norm(pre['p'][()][-1]-truth['p'][k][-1])),preTipMomentNmm=num(pre,'tipMomentNmm'),preActiveGapMm=float(np.max(np.abs(arr(pre,'gapMm')))) if active else 0,preFbgRms=num(pre,'fbgRms'),preModeViolation=num(pre,'modeViolation'),bvpSuccess=num(b,'converged'),bvpFailed=num(b,'failed'),bvpSeconds=num(b,'seconds'),bvpRhsEvaluations=num(b,'rhsEvaluations'),bvpJacobianEvaluations=num(b,'jacobianEvaluations'),bvpNodes=num(b,'finalNodes'),bvpRegions=num(b,'regions'),bvpBoundaryResidual=num(b,'boundaryResidual'),bvpCollocationMomentNmm=num(b,'collocationTipMomentNmm'),bvpShapeChangeRmsMm=num(b,'shapeChangeRmsMm'),bvpOrthogonality=num(b,'rotationOrthogonality'),ivpBvpShapeRmsMm=num(b,'ivpBvpShapeRmsMm') if 'ivpBvpShapeRmsMm' in b else float('nan'),forceChangeN=num(b,'forceChangeN'),affineSuccess=int(num(m,'affineConverged') and num(m,'affineFeasible')),bvpException=text(b,'exception'),bvpWarning=text(b,'warning'),exactSlidingErrorN=float('nan'))
    if len(mode)==1 and mode[0]==2:
     xx=arr(sh,'x');v=arr(e,'slip');mu=arr(out['geometry'],'mu')[0]
     if np.linalg.norm(v)>1e-8:q['exactSlidingErrorN']=float(np.linalg.norm(xx[7:9]+mu*xx[6]*v/np.linalg.norm(v)))
    xx=arr(sh,'x');preSlip=arr(pre,'slipMm');q['preWorkNmm']=float(np.sum(xx[6:].reshape(-1,3)[:,1:]*preSlip.reshape(-1,2))) if preSlip.size else 0.
    frames.append(q)
    for j,ref in enumerate(m['attempts'][()].ravel()):
     a=f[ref];branches.append(dict(scenario=scenario,targetMm=target,frame=k+1,attempt=j+1,selected=int(j+1==int(r['selectedAttempt'])),iterations=num(a,'iterations'),nativeConverged=num(a,'converged'),affineFeasible=num(a,'modeFeasible'),seconds=num(a,'seconds')))
  sub=[r for r in frames if r['scenario']==scenario and r['targetMm']==target];o=old[(scenario,target)]
  s=dict(scenario=scenario,targetMm=target,frames=len(sub),tipRMSE_N=rms([r['tipErrorN'] for r in sub]),contactRMSE_N=rms([r['contactErrorN'] for r in sub]),shapeRMS_mm=rms([r['shapeRmsMm'] for r in sub]),shapeTipRMS_mm=rms([r['shapeTipErrorMm'] for r in sub]),preCorrectionShapeRMS_mm=rms([r['preShapeRmsMm'] for r in sub]),meanSeconds=float(np.mean([r['seconds'] for r in sub])),medianSeconds=float(np.median([r['seconds'] for r in sub])),maxSeconds=max(r['seconds'] for r in sub),meanBvpSeconds=float(np.mean([r['bvpSeconds'] for r in sub])),meanIVPs=float(np.mean([r['ivps'] for r in sub])),meanBvpRhsEvaluations=float(np.mean([r['bvpRhsEvaluations'] for r in sub])),meanIterations=float(np.mean([r['iterations'] for r in sub])),meanAssignments=float(np.mean([r['assignments'] for r in sub])),affineSuccess=sum(r['affineSuccess'] for r in sub),bvpSuccess=sum(r['bvpSuccess'] for r in sub),bvpFailed=sum(r['bvpFailed'] for r in sub),nonlinearContactSuccess=sum(r['solverSuccess'] for r in sub),maxPreTipMomentNmm=max(r['preTipMomentNmm'] for r in sub),maxTipMomentNmm=max(r['tipMomentNmm'] for r in sub),maxActiveGapMm=max(r['activeGapMm'] for r in sub),maxPreActiveGapMm=max(r['preActiveGapMm'] for r in sub),maxPenetrationMm=max(r['penetrationMm'] for r in sub),maxBvpNodes=max(r['bvpNodes'] for r in sub),maxBvpShapeDisagreementMm=max(r['ivpBvpShapeRmsMm'] for r in sub),maxBvpOrthogonality=max(r['bvpOrthogonality'] for r in sub),maxExactSlidingErrorN=max([r['exactSlidingErrorN'] for r in sub if np.isfinite(r['exactSlidingErrorN'])],default=0),terminalMomentReduced=sum(r['tipMomentNmm']<r['preTipMomentNmm'] for r in sub),fbgFitWorsened=sum(r['fbgRms']>r['preFbgRms']+1e-8 for r in sub),shapeErrorWorsened=sum(r['shapeRmsMm']>r['preShapeRmsMm']+1e-8 for r in sub),prePositiveFrictionWorkFrames=sum(r['preWorkNmm']>1e-8 for r in sub),positiveFrictionWorkFrames=sum(r['workNmm']>1e-8 for r in sub),maxPositiveFrictionWorkNmm=max([0]+[r['workNmm'] for r in sub]),maxNormalizedFbgRms=max(r['fbgRms'] for r in sub),fbgAcceptancePass=sum(r['fbgRms']<=1.24 for r in sub),maxForceChangeN=max(r['forceChangeN'] for r in sub),modesChangedVsOld=sum(r['modeChangedVsOld'] for r in sub),oldTipRMSE_N=o['tipRMSE_N'],oldContactRMSE_N=o['contactRMSE_N'],oldShapeRMS_mm=o['shapeRMS_mm'],oldMeanSeconds=o['meanSeconds'],oldMaxTipMomentNmm=o['maxTipMomentNmm'],oldMaxActiveGapMm=o['maxActiveGapMm'])
  stats.append(s)
write(OUT/'summary.csv',stats);write(OUT/'frames.csv',frames);write(OUT/'branches.csv',branches)
complete=len(stats)==8
info=dict(summary=stats,missing=missing,complete=complete,totalFrames=len(frames));(OUT/'summary.json').write_text(json.dumps(info,indent=2))
if not complete:print(json.dumps(info,indent=2));sys.exit(0)
assert len(frames)==260 and all(s['frames']==(41 if s['scenario']=='insertion' else 24) for s in stats)
fig,axes=plt.subplots(2,4,figsize=(16,8),constrained_layout=True)
for i,sc in enumerate(['insertion','rotation']):
 ss=sorted([s for s in stats if s['scenario']==sc],key=lambda s:s['targetMm'])
 for j,(key,oldkey,label) in enumerate([('tipRMSE_N','oldTipRMSE_N','Tip force RMSE [N]'),('contactRMSE_N','oldContactRMSE_N','Contact force RMSE [N]'),('shapeRMS_mm','oldShapeRMS_mm','Nonlinear shape RMS [mm]'),('meanSeconds','oldMeanSeconds','Mean seconds/frame')]):
  ax=axes[i,j];levels=[s['targetMm'] for s in ss];ax.plot(levels,[s[oldkey] for s in ss],'-o',label='Single pass');ax.plot(levels,[s[key] for s in ss],'-o',label='Single pass + fixed-load BVP');ax.set(xlabel='Geometric tip-noise RMS target [mm]',ylabel=label,title=sc.capitalize());ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
report=['# Single-pass full-cone estimator with fixed-load BVP correction','','Date: 2026-10-12. Branch codex/two-stage-force-sensing, base b376013. Eight complete trajectories / 260 new solves; no commit or push. User explicitly authorized this correction and exhaustive search. All outputs are separate from prior trials.','',
'The existing full-cone method uses fmincon SQP, not a pure QP: circular sliding retains a nonlinear direction variable. Each frame exhausts separation/sticking/sliding on one affine rod model (100 SQP iterations per branch), selects the lowest affine-feasible objective, then calls MATLAB bvp4c once with all tip/contact forces fixed. No contact-mode reselection or force optimization occurs after that selection. Only base moment and nonlinear shape change. The corrected state feeds the next frame, so full-trajectory force differences reflect history feedback even though every individual correction leaves forces exactly unchanged.','',
'Configuration and measurements match the prior single-pass test: insertion 80->40->80 mm distance, 41 frames; rotation 0->45->0 degrees by 3.75 degrees at maximum insertion, 24 new solves excluding restored seed. Rod 200 mm, 80 straight +120 precurved 90 degrees; true tip [1,0,0] N, mu=0.3. Gaussian Bishop noise targets 0/.5/1/2 mm use identical prior packets; no environment or mu noise. Zero injected noise retains covariance std 1e-4/mm. Stage 1 geometry matches the prior trajectories.','',
'BVP unknowns are scaled position, nine rotation entries, moment and (for Bishop sensing) material-to-Bishop roll. Base position/orientation/roll and zero terminal moment are enforced, with continuity at intrinsic-curvature/stiffness/contact interfaces. Interior contact arclengths and loads stay fixed; wall gap/tangency and friction slip constraints are not part of this BVP. The initial guess is the optimized nonlinear IVP shape and base moment. bvp4c uses RelTol=1e-7, AbsTol=1e-9 on scaled states, NMax=4000, an initial 5 mm mesh, and an analytic ODE Jacobian. The existing SQP/diagnostic tolerances are unchanged.','',
'After the BVP call an independent augmented IVP with the corrected base moment recomputes the full estimator diagnostics. Reported state is this IVP representation of the BVP solution. Successful frames thus use three IVPs plus the BVP collocation solve; BVP RHS evaluations are reported separately and are not counted as IVPs. One BVP call can contain multiple internal Newton/mesh iterations. Warnings/failures remain recorded, and contact diagnostics do not stop or change the step.','',
'## Force, shape and timing comparison','',
'All errors are trajectory RMS vector norms; shape RMS includes all sampled centerline nodes. Old columns refer to the preserved no-correction single-pass full-cone trajectories, not outer-loop controls.','']
rows=[]
for s in stats:rows.append([s['scenario'],s['targetMm'],fmt(s['oldTipRMSE_N']),fmt(s['tipRMSE_N']),fmt(s['oldContactRMSE_N']),fmt(s['contactRMSE_N']),fmt(s['oldShapeRMS_mm']),fmt(s['shapeRMS_mm']),fmt(s['oldMeanSeconds']),fmt(s['meanSeconds'])])
report+=[table(['Motion','Noise mm','Old tip N','BVP tip N','Old contact N','BVP contact N','Old shape mm','BVP shape mm','Old mean s','BVP mean s'],rows),'','## Correction effects and physical consistency','']
rows=[]
for s in stats:rows.append([s['scenario'],s['targetMm'],fmt(s['maxPreTipMomentNmm']),fmt(s['maxTipMomentNmm']),fmt(s['maxPreActiveGapMm']),fmt(s['maxActiveGapMm']),fmt(s['maxPenetrationMm']),f"{int(s['bvpSuccess'])}/{s['frames']}",f"{int(s['nonlinearContactSuccess'])}/{s['frames']}"])
report+=[table(['Motion','Noise mm','Max moment before N mm','Max moment after N mm','Max active gap before mm','Max active gap after mm','Max penetration after mm','BVP numerical success','Full mode success'],rows),'','Before/after columns compare the same frame with identical forces; the first table compares complete recurrent trajectories. Positive separation gaps are excluded from active-gap errors. BVP convergence certifies its fixed-load equilibrium problem, while full mode success also requires contact geometry and friction consistency at 1e-6.','', '## Computational cost and validation','']
rows=[]
for s in stats:rows.append([s['scenario'],s['targetMm'],fmt(s['meanBvpSeconds']),fmt(s['meanIVPs']),fmt(s['meanBvpRhsEvaluations']),fmt(s['meanIterations']),fmt(s['meanAssignments']),int(s['maxBvpNodes']),fmt(s['maxBvpShapeDisagreementMm'])])
report+=[table(['Motion','Noise mm','Mean BVP s','IVPs/frame','Mean BVP RHS calls','Total SQP iterations/frame','Modes/frame','Max mesh nodes','Max IVP-BVP shape RMS mm'],rows),'']
selected=[b for b in branches if b['selected']];report+=[f"Branch solves: {len(branches)}; maximum selected SQP iterations: {int(max(b['iterations'] for b in selected))}; selected at the 100-iteration cap: {sum(b['iterations']>=100 for b in selected)}; all alternatives at the cap: {sum(b['iterations']>=100 for b in branches)}. A capped alternative prevents an unconditional global-optimality certificate.",'',f"Positive friction work appears on {sum(s['positiveFrictionWorkFrames'] for s in stats)} corrected frames (before correction: {sum(s['prePositiveFrictionWorkFrames'] for s in stats)} frames); this is explicitly flagged in the saved diagnostics. Fixed-load BVP convergence does not establish dissipative contact behavior.",'',f"Selected modes differ from the preserved uncorrected trajectories on {sum(s['modesChangedVsOld'] for s in stats)}/{len(frames)} frames. This is a recurrent-history comparison, not a same-state force-refit experiment.",'',f"All {len(frames)} corrections preserve optimized forces exactly (max difference {max(s['maxForceChangeN'] for s in stats):.3g} N). Terminal moment decreased in {sum(s['terminalMomentReduced'] for s in stats)}/{len(frames)} frames. FBG fit worsened in {sum(s['fbgFitWorsened'] for s in stats)} frames and shape error worsened in {sum(s['shapeErrorWorsened'] for s in stats)} frames. These within-frame comparisons measure correction effects directly. Native affine success: {int(sum(s['affineSuccess'] for s in stats))}/{len(frames)}; BVP numerical success: {int(sum(s['bvpSuccess'] for s in stats))}/{len(frames)}. Exceptions/warnings, exact circular sliding errors, and all frame residuals are in frames.csv.",'',
'## Interpretation','',
'A fixed-load BVP addresses terminal equilibrium, but it cannot simultaneously adjust erroneous force estimates to preserve the measured shape or enforce wall contact. Physical contact constraints can worsen even when the BVP converges. This experiment measures that tradeoff without silently changing the force estimates or imposing an unapproved correction rule. A contact-consistent force correction would be a different formulation. No general speed or accuracy claim follows from one noise seed and these two trajectories.','',
'In noiseless rotation, force RMS increases from 0.046/0.143 N to 0.424/1.970 N (tip/contact), while eight frames change from sliding to sticking. At 2 mm insertion, maximum corrected active gap reaches 1.887 mm. Consequently this fixed-load BVP is useful as an equilibrium audit, but the measured results do not support adopting it as a contact-consistent estimator correction. A future correction should allow forces to change while enforcing contact and FBG fit together; that was not implemented here.','',
'## Reproduction and artifacts','',
"From package root: setup_tsfs; addpath('experiments/inclined_wall'); run_single_pass_bvp('FRESH_STAMP'). Then python experiments/inclined_wall/analyze_single_pass_bvp.py FRESH_STAMP. Use a fresh stamp; raw checkpoints reject accidental overwrites. Historical 20261012_unbounded input packets and baseline results are read-only dependencies.",'',f'Combined output: results/published/singlepass_bvp_comparison_{STAMP}/. Per-trajectory CSVs: results/published/singlepass_bvp_*_{STAMP}/. Raw MATs: results/runs/singlepass_bvp_*_{STAMP}/comparison.mat. Main log: singlepass_bvp_{STAMP}.log. Preliminary smoke interface failures, successful smoke_v2 validation, and the interrupted first batch (20261012_fixedload) are preserved and excluded from scores. The first batch exposed floating-point interface evaluation outside a dense-ODE segment by roundoff; initial guesses now clamp to exact segment endpoints.','']
(ROOT/'docs/reports/SINGLE_PASS_FIXED_LOAD_BVP_RESULTS.md').write_text('\n'.join(report))
print(json.dumps(info,indent=2))
