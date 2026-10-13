"""Publish the Bishop rotation comparison, retaining the failed cold-start replay."""
from pathlib import Path
import csv,json,shutil,hashlib
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
RUN='base_rotation_bishop_20261007'
def read(p):
    with p.open(encoding='utf-8-sig',newline='') as f:r=list(csv.DictReader(f))
    for row in r:
        for k,v in row.items():
            try:row[k]=float(v)
            except ValueError:pass
    return r
def arr(r,k):return np.array([x[k] for x in r])
def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def summarize(rows):
    result={k:rms(arr(rows,k)) for k in ['tipErrorN','contactErrorN','totalErrorN','shapeRmsMm']}
    result.update({k:float(arr(rows,k).mean()) for k in ['seconds','geometrySeconds','mechanicsSeconds','sqpIterations','totalIVPs','stateIVPs','augmentedIVPs','odeSegments','rhsEvaluations']})
    result['converged']=int(sum(arr(rows,'converged')));result['failedAngles']=arr(rows,'angleDeg')[arr(rows,'converged')==0].tolist()
    result['flags']=dict(Counter(f for row in rows for f in row['flags'].split(';') if f));result['final']=rows[-1]
    return result
def main():
    run=ROOT/'results/runs'/RUN;src=run/'rotation';dst=ROOT/'results/published'/RUN;dst.mkdir(parents=True,exist_ok=True)
    allr=read(src/'ours_frames.csv');r=[x for x in allr if x['mu']==.3 and x['repeat']==2];z=[x for x in allr if x['mu']==0 and x['repeat']==2];c=[x for x in read(src/'cold_frames.csv') if x['mu']==.3 and x['repeat']==2]
    a=read(src/'aloi_frames.csv');g=read(src/'stage1_accuracy.csv');ph=read(src/'physics.csv');old=json.loads((ROOT/'results/published/base_rotation_20261006/summary.json').read_text())
    timing=json.loads((src/'aloi_timing.json').read_text());profile=json.loads((src/'aloi_profile.json').read_text());test=json.loads((run/'bishop_tests.json').read_text());obs=json.loads((src/'observation_validation.json').read_text())
    A,Z,C=summarize(r),summarize(z),summarize(c);B={k:rms(arr(a,k)) for k in ['totalErrorN','shapeRmsMm']};B.update(timing);B['positiveExitFlags']=int(sum(arr(a,'exitFlag')>0));B['equilibriumConverged']=int(sum(arr(a,'equilibriumConverged')));B['iterationsWinningStart']=float(arr(a,'iterationsWinningStart').mean());B['shapeIntegrationsMean']=next(x['calls'] for x in profile['functions'] if x['name']=='solveShape')/len(a)
    repeat=0.
    for row in allr:
        if row['repeat']!=1:continue
        match=next(x for x in allr if x['repeat']==2 and x['mu']==row['mu'] and x['frame']==row['frame'])
        for key in ['tipErrorN','contactErrorN','shapeRmsMm','totalIVPs','sqpIterations']:repeat=max(repeat,abs(row[key]-match[key]))
    assert repeat<1e-9 and obs['sameBaselineInputMaxMm']==0 and max(arr(g,'muGeometryDifference'))==0
    manifest=json.loads((run/'preservation_before.json').read_text());changed=[name for name,digest in manifest.items() if hashlib.sha256((ROOT.parent/name).read_bytes()).hexdigest()!=digest];assert not changed,changed
    G={'shapeRmsAcrossFramesMm':rms(arr(g,'shapeRmsMm')),'maxFrameShapeRmsMm':float(max(arr(g,'shapeRmsMm'))),'maxArcErrorMm':float(max(abs(arr(g,'arcErrorMm')))),'maxNormalErrorDeg':float(max(arr(g,'normalErrorDeg'))),'maxContactPointErrorMm':float(max(arr(g,'contactPointErrorMm'))),'final':g[-1]}
    result={'oursMu03':A,'oursMu0':Z,'coldStartMu03':C,'aloi':B,'stage1':G,'repeatMaxDifference':repeat,'preservedFileCount':len(manifest),'validation':test,'observation':obs}
    (dst/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    for file in src.iterdir():
        if file.suffix in ['.csv','.json']:shutil.copy2(file,dst/file.name)
    for name in ['bishop_tests.json','unit_tests.json','spatial_derivative_tests.json']:shutil.copy2(run/name,dst/name)
    sources=list((ROOT/'matlab/+tsfs').glob('*.m'))+[ROOT/'tests/test_bishop.m']+list((ROOT/'experiments/base_rotation').glob('*bishop*'))
    (dst/'source_hashes.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
    x=arr(r,'angleDeg');plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False});fig,axs=plt.subplots(2,3,figsize=(13,7.4),constrained_layout=True)
    ax=axs[0,0];ax.semilogy(x,arr(g,'shapeRmsMm'),'o-',label='Bishop Stage 1');oldr=read(ROOT/'results/published/base_rotation_20261006/ours_frames.csv');oldr=[v for v in oldr if v['mu']==.3 and v['repeat']==2];ax.semilogy(x,arr(oldr,'stage1ShapeRmsMm'),'--',label='Previous material u1,u2 + u3=0');ax.set(title='Stage 1 shape RMS',ylabel='mm');ax.legend(fontsize=8)
    ax=axs[0,1];ax.plot(x,arr(g,'arcErrorMm')*1000,'o-');ax.set(title='Stage 1 contact arc error',ylabel='micrometers')
    ax=axs[0,2];ax.plot(x,arr(g,'normalErrorDeg'),'o-');ax.set(title='Stage 1 normal error',ylabel='degrees')
    names=['Ours, mu=0.3','Ours, mu=0','Aloi'];cols=['#177e89','#d57827','#7655a6']
    for row,name,col in zip([r,z],names,cols):axs[1,0].semilogy(x,arr(row,'tipErrorN'),'o-',label=name,color=col)
    axs[1,0].semilogy(x,arr(c,'tipErrorN'),':',color='gray',label='Bishop cold start, mu=0.3');axs[1,0].set(title='Tip-force error',ylabel='N');axs[1,0].legend(fontsize=8)
    for row,name,col in zip([r,z,a],names,cols):axs[1,1].semilogy(x,arr(row,'shapeRmsMm'),'o-',label=name,color=col)
    axs[1,1].set(title='Mechanical shape RMS',ylabel='mm');axs[1,1].legend(fontsize=8)
    for row,name,col in zip([r,z],names,cols):axs[1,2].plot(x,arr(row,'seconds'),'o-',label=name,color=col)
    axs[1,2].set(title='Estimator time per frame',ylabel='seconds');axs[1,2].legend(fontsize=8)
    for ax in axs.flat:ax.set(xlabel='Base rotation (degrees)',xticks=[0,15,30,45]);ax.grid(alpha=.2)
    fig.suptitle('Noiseless rotation with Bishop observations: 24 sensor stations');fig.savefig(dst/'comparison.png',dpi=170);plt.close(fig)
    physr=[v for v in ph if v['mu']==.3];final=A['final'];oldA=old['oursMu03'];geo=G['final'];opts=json.loads((src/'metadata.json').read_text())['options']
    report=f'''# Bishop-observation base-rotation comparison

Date: 2026-10-07. Branch: codex/two-stage-force-sensing. Local changes only; no commit or push.

## Scope and observation contract

The physical reference is the preserved, independently solved 0-45 degree axial base-rotation trajectory, at 18 mm insertion, with world-fixed plane and tip force [1,0,-1] N. True friction is 0.3. There are 13 quasistatic frames and 24 noiseless geometric sensor stations. No finite-radius contact torque, tip contact, or edge/corner geometry was added. This is an ideal geometric sensor benchmark, not a validation of the experimental FBGS software's twist compensation.

The sensor outputs w1,w2 defined by t'=w1*b1+w2*b2, with B(base)=R(base). Full material strain u is converted outside the estimator using psi'=u3 and w=R2(psi)[u2,-u1]. Reference conversion uses the unchanged upstream forward map at 0.1 mm output spacing, retaining the same physical forces, contact arcs and actuation. The estimator receives only sampled w, calibrated rod/environment inputs and actuation. It never receives true twist, forces, contact arcs or dense truth positions. The saved MAT contains input and truth separately for scoring.

Stage 1 integrates Bishop angular strain [-w2,w1,0]. Piecewise PCHIP is confined to calibrated intrinsic/stiffness segments, preserving the known 120 mm intrinsic-curvature transition without using the unknown contact arc. No material intrinsic vector is incorrectly subtracted from a Bishop vector. Each calibrated segment currently requires at least two sensor stations. This adapter targets the existing piecewise calibrated rod model; it should not treat an arbitrary smoothly varying calibration as a collection of sparse discontinuities.

Stage 2 retains full material mechanics, adds a scalar psi state and its analytic sensitivities, and compares predicted Bishop curvature with measured Bishop curvature. The likelihood, force priors, complementarity projections and semismooth SQP retain the previous tolerances. The zero-mu run changes assumed friction only. Aloi's original algorithm is unchanged and receives sparse positions reconstructed from the exact same Bishop measurements.

## Results, including every frame

Force entries are RMS Euclidean force errors across frames; shape entries combine all rod nodes and frames. Timing is the second complete deterministic replay for our methods.

| Quantity | Ours mu=0.3 | Ours mu=0 | Aloi |
|---|---:|---:|---:|
| Contact force RMSE (N) | {A['contactErrorN']:.6f} | {Z['contactErrorN']:.6f} | Not separated |
| Tip force RMSE (N) | {A['tipErrorN']:.6f} | {Z['tipErrorN']:.6f} | Not separated |
| Total force RMSE (N) | {A['totalErrorN']:.6f} | {Z['totalErrorN']:.6f} | {B['totalErrorN']:.6f} |
| Shape RMS (mm) | {A['shapeRmsMm']:.6f} | {Z['shapeRmsMm']:.6f} | {B['shapeRmsMm']:.6f} |
| Seconds/frame | {A['seconds']:.4f} | {Z['seconds']:.4f} | {B['meanFrameSeconds']:.4f} |
| Stage 1 seconds/frame | {A['geometrySeconds']:.4f} | {Z['geometrySeconds']:.4f} | Shared reconstruction preparation |
| Mechanics seconds/frame | {A['mechanicsSeconds']:.4f} | {Z['mechanicsSeconds']:.4f} | Included above |
| SQP iterations/frame | {A['sqpIterations']:.2f} | {Z['sqpIterations']:.2f} | Different solver |
| Full-rod IVPs/frame | {A['totalIVPs']:.2f} | {Z['totalIVPs']:.2f} | No shooting IVP |
| Segmented ODE calls/frame | {A['odeSegments']:.2f} | {Z['odeSegments']:.2f} | Different integration |
| RHS evaluations/frame | {A['rhsEvaluations']:.1f} | {Z['rhsEvaluations']:.1f} | Different integration |
| Converged optimizer frames | {A['converged']}/13 | {Z['converged']}/13 | {B['positiveExitFlags']}/13 positive exits |

At 45 degrees, correct-mu tip error is {final['tipErrorN']:.6f} N and contact error {final['contactErrorN']:.6f} N. Estimated tip force is [{final['tipFx']:.6f}, {final['tipFy']:.6f}, {final['tipFz']:.6f}] N.

![Comparison](../../results/published/{RUN}/comparison.png)

## Stage 1 accuracy

Across all frames, Stage 1 shape RMS is {G['shapeRmsAcrossFramesMm']:.8f} mm; maximum per-frame RMS is {G['maxFrameShapeRmsMm']:.8f} mm. Maximum absolute contact-arc error is {G['maxArcErrorMm']:.8f} mm, maximum normal error {G['maxNormalErrorDeg']:.8f} degrees, and maximum contact-point position error {G['maxContactPointErrorMm']:.8f} mm. Stage 1 results are identical for both assumed friction coefficients.

At 45 degrees, shape RMS is {geo['shapeRmsMm']:.8f} mm, tip position error {geo['shapeTipMm']:.8f} mm, arc error {geo['arcErrorMm']:.8f} mm, and normal error {geo['normalErrorDeg']:.8f} degrees. Previous Stage 1 shape RMS was {old['finalOurs']['stage1ShapeRmsMm']:.6f} mm. The integration refinement table holds the same 24 measurements fixed; remaining error includes sparse interpolation and kinematic discretization. No contact truth is used to correct it.

## Initialization ablation and failures

Simply changing the observations/residual while retaining the old per-frame linear seed gives {C['converged']}/13 converged frames, tip RMSE {C['tipErrorN']:.6f} N and shape RMS {C['shapeRmsMm']:.6f} mm. Failed angles: {C['failedAngles']}. This replay is preserved as cold_frames.csv and cold.mat; its 41.25-degree failure occurs despite accurate Stage 1 geometry and propagates into the next frame's mechanical history.

The final run uses the preceding converged material state as a numerical start when contacts match. This avoids repeatedly interpreting a Bishop frame as a measured material frame. First-frame/unmatched initialization remains the old approximate linear seed. This is a starting-point correction: it does not change the MAP objective, tolerances, measurements or diagnostics, and is controlled by bishopWarmStart. The no-extra-within-step-loop design is retained. Final failed angles: mu=0.3 {A['failedAngles']}; mu=0 {Z['failedAngles']}.

A same-history diagnostic isolates the remaining outlier: at 22.5 degrees the recurrent seed returns objective 404.06 and tip error 0.71048 N, whereas the linear seed reaches objective 16.814 and tip error 0.002391 N using the same observations, geometry and preceding state. At 41.25 degrees the recurrent seed succeeds, while the linear seed fails. This establishes initialization/globalization sensitivity, not an irreducible geometric error or proof of a second physical solution. The 22.5-degree outlier remains in all headline metrics; no automatic restart or result substitution was introduced. See same_history_initialization.csv. A more reliable initialization/globalization policy remains future work.

Correct-mu flag counts: {A['flags']}. Zero-mu flag counts: {Z['flags']}. Maximum correct-mu inferred-plane penetration: {max(arr(r,'penetrationMm')):.6g} mm; maximum projection residual: {max(arr(r,'projectionN')):.6g} N. Maximum positive modeled friction work: {max(0,max(arr(physr,'frictionWorkNmm'))):.6g} N mm. Flags are preserved, not suppressed: the work warning uses a stricter near-zero threshold than the force projection tolerance and must be interpreted with the reported slip/projection values. The reconstructed environment differs slightly from the true plane: true-plane penetration is recorded separately in ours_frames.csv and must not be confused with feasibility against the inferred plane. All detailed physics, solver failures and iteration traces are published. A converged flag does not certify accuracy under the deliberately incorrect mu=0 model.

The previous material-observation run had tip/contact RMSE {oldA['tipErrorN']:.6f}/{oldA['contactErrorN']:.6f} N and {oldA['converged']}/13 converged frames. Improvements combine the intended geometric observation model and the explicitly reported initialization correction; the cold replay isolates the former.

## Solver settings and validation

MATLAB geometry MAP: lsqnonlin trust-region-reflective, max 30 iterations, function/step tolerance 1e-8, optimality 1e-6. Mechanics: existing semismooth SQP with active-set quadprog, max {opts['maxIterations']} iterations, stationarity {opts['stationarityTolerance']}, moment {opts['momentTolerance']} N mm, projection {opts['projectionTolerance']} N, gap {opts['gapTolerance']} mm. ODE relative/absolute coefficients: {opts['odeRelativeTolerance']}/{opts['odeAbsoluteTolerance']}, with the existing state scaling. No injected sensor or environment noise; the curvature likelihood retains the declared standard deviation {obs['likelihoodStdPerMm']:.3g}/mm. Environment priors retain 0.03 mm point-component and 0.001 normal-component standard deviations, despite noiseless mean observations.

The analytic Bishop observation Jacobian agrees with finite differences to relative error {test['relativeObservationJacobianError']:.3g}. An independent analytic helix reconstructs within {test['analyticHelixPositionMaxMm']:.3g} mm on the test grid. The independent forward truth predicts sensor measurements within {test['independentTruthPredictionMaxPerMm']:.3g}/mm and positions within {test['independentTruthPositionMaxMm']:.3g} mm. Rigid-world-rotation invariance passes. Existing core and spatial derivative tests pass. Repeated numerical outcomes agree to {repeat:.3g}. All {len(manifest)} hashed collaborator-source/previous-published files are unchanged.

Aloi mean winning-start iterations: {B['iterationsWinningStart']:.2f}; separate profiling counts {B['shapeIntegrationsMean']:.2f} shape integrations/frame. Final equilibrium checks pass {B['equilibriumConverged']}/13. Unprofiled sequence time is {B['sequenceSeconds']:.4f} s plus {B['preparationSeconds']:.4f} s preparation; profiling overhead is excluded. Comparisons use different integration schemes and warmup histories, so timings are descriptive rather than a controlled hardware speedup claim.

## Reproduction

From the package directory, with the previous physical reference available:

```matlab
setenv('TSFS_RUN_ID','{RUN}'); setup_tsfs;
prepare_bishop_rotation;
test_bishop; test_tsfs; test_spatial_derivatives;
run_bishop_rotation('cold');
run_bishop_rotation('ours');
run_bishop_rotation('aloi');
run_bishop_rotation('profile');
analyze_bishop_rotation;
diagnose_bishop_initialization;
```

Then run `python experiments/base_rotation/summarize_bishop_rotation.py`. The previous base_rotation_20261006 benchmark and single_contact_mu_20261006 benchmark are preserved. The forward physical friction-sensitivity results remain applicable to the unchanged truth; the old inverse-sensitivity results belong to the old observation model and are not reused here.
'''
    (ROOT/'docs/reports/BASE_ROTATION_BISHOP_RESULTS.md').write_text(report,encoding='utf-8');(dst/'REPORT.md').write_text(report.replace(f'../../results/published/{RUN}/',''),encoding='utf-8')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
