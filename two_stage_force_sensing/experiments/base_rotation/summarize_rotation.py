"""Publish the isolated base-rotation experiment from saved MATLAB outputs."""
from pathlib import Path
import argparse, csv, json, shutil
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]

def read(path):
    with path.open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
    for r in rows:
        for k,v in r.items():
            try:r[k]=float(v)
            except ValueError:pass
    return rows

def values(rows,key):return np.array([r[key] for r in rows],float)
def rms(v):return float(np.sqrt(np.mean(np.array(v)**2)))
def summary(rows,ours=True):
    d={k:rms(values(rows,k)) for k in ['totalErrorN','shapeRmsMm']}
    if ours:
        d.update({k:rms(values(rows,k)) for k in ['tipErrorN','contactErrorN','twistRmsErrorPerMm']})
        d.update({k:float(values(rows,k).mean()) for k in ['seconds','geometrySeconds','mechanicsSeconds','sqpIterations','totalIVPs','odeSegments','rhsEvaluations']})
        d['converged']=int(sum(values(rows,'converged')))
        d['flagCounts']=dict(Counter(f for r in rows for f in r['flags'].split(';') if f))
        d['failedAngles']=[r['angleDeg'] for r in rows if not r['converged']]
    return d

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-id',default='base_rotation_20261006');args=parser.parse_args()
    assert args.run_id.startswith('base_rotation_') and all(c.isalnum() or c in '_-' for c in args.run_id)
    src=ROOT/'results/runs'/args.run_id/'rotation';dst=ROOT/'results/published'/args.run_id;dst.mkdir(parents=True,exist_ok=True)
    all_ours=read(src/'ours_frames.csv');ours=[r for r in all_ours if r['mu']==.3 and r['repeat']==2];zero=[r for r in all_ours if r['mu']==0 and r['repeat']==2]
    aloi=read(src/'aloi_frames.csv');sens=read(src/'sensitivity.csv');physics=read(src/'physics.csv');shapes=read(src/'shape_profiles.csv')
    truth={mu:read(src/f'truth_mu_{round(mu*1000):03d}_audit.csv') for mu in [0,.29,.3,.31]}
    local=json.loads((src/'local_sensitivity.json').read_text());validation=json.loads((src/'reference_validation.json').read_text())
    timing=json.loads((src/'aloi_timing.json').read_text());profile=json.loads((src/'aloi_profile.json').read_text())
    assert all(len(r)==13 for r in [ours,zero,aloi,sens,*truth.values()])
    assert all(r['passed']==1 for seq in truth.values() for r in seq)
    assert validation['noiseMaximumPerMm']==0
    assert all(r['stage1ArcDifferenceMm']==r['stage1NormalDifference']==0 for r in local)
    difference=0.
    for r in all_ours:
        if r['repeat']!=1:continue
        p=next(q for q in all_ours if q['repeat']==2 and q['mu']==r['mu'] and q['frame']==r['frame'])
        for key in ['tipErrorN','contactErrorN','shapeRmsMm','sqpIterations','totalIVPs']:difference=max(difference,abs(r[key]-p[key]))
    assert difference<1e-9
    a,z,b=summary(ours),summary(zero),summary(aloi,False);b.update(timing)
    b['winningStartIterations']=float(values(aloi,'iterationsWinningStart').mean());b['equilibriumConverged']=int(sum(values(aloi,'equilibriumConverged')))
    b['positiveExitFlags']=int(sum(values(aloi,'exitFlag')>0))
    calls={r['name']:r['calls'] for r in profile['functions']};b['shapeIntegrationsTotal']=calls['solveShape'];b['shapeIntegrationsMean']=calls['solveShape']/13
    transitions={str(mu):next((r['angleDeg'] for r in seq if r['slipMm']>1e-5),None) for mu,seq in truth.items()}
    result={'oursMu03':a,'oursMu0':z,'aloi':b,'firstSampleWithSlipDeg':transitions,'pairedRepeatDifference':difference,'finalTruth':truth[.3][-1],
            'finalSensitivity':sens[-1],'finalOurs':ours[-1],'finalOursMu0':zero[-1],'referenceValidation':validation}
    (dst/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    for name in ['metadata.json','ours_frames.csv','inverse_frames.csv','aloi_frames.csv','aloi_timing.json','aloi_profile.json','sensitivity.csv','physics.csv','shape_profiles.csv','local_sensitivity.json','iteration_trace.csv','reference_validation.json']+[f'truth_mu_{round(mu*1000):03d}_audit.csv' for mu in truth]:shutil.copy2(src/name,dst/name)
    plt.rcParams.update({'font.size':10,'axes.spines.right':False,'axes.spines.top':False});colors=['#147e9c','#d36a1b','#7952a2'];x=values(ours,'angleDeg')
    fig,axs=plt.subplots(3,2,figsize=(11,10),constrained_layout=True)
    for ax,key,label in zip(axs.flat[:4],['contactFx','contactFy','contactFz','tipFy'],['Contact X','Contact Y','Contact Z','Tip Y']):
        target=values(truth[.3],key) if key.startswith('contact') else np.zeros_like(x)
        ax.plot(x,target,'k--',lw=2,label='Truth')
        for rows,c,name in zip([ours,zero],colors,['Ours, mu = 0.3','Ours, mu = 0']):ax.plot(x,values(rows,key),'o-',ms=3,color=c,label=name)
        for seq in [ours,zero]:
            bad=values(seq,'converged')==0;ax.scatter(x[bad],values(seq,key)[bad],marker='x',s=45,color='black',zorder=4)
        ax.set(title=label,ylabel='Force (N)')
    for ax,key,title,unit in zip(axs.flat[4:],['shapeRmsMm','totalErrorN'],['Shape error','Total force error'],['mm','N']):
        for rows,c,name in zip([ours,zero,aloi],colors,['Ours, mu = 0.3','Ours, mu = 0','Aloi (Tongyu)']):ax.semilogy(x,values(rows,key),'o-',ms=3,color=c,label=name)
        for seq in [ours,zero,aloi]:
            bad=values(seq,'converged')==0 if 'converged' in seq[0] else values(seq,'exitFlag')<=0
            ax.scatter(x[bad],values(seq,key)[bad],marker='x',s=45,color='black',zorder=4)
        ax.set(title=title,ylabel=unit)
    for ax in axs.flat:ax.set(xlabel='Base rotation (degrees)',xticks=[0,15,30,45]);ax.grid(alpha=.2)
    axs[0,0].plot([],[],'kx',label='Solver not converged');axs[0,0].legend(fontsize=8);axs[2,0].legend(fontsize=8);fig.suptitle('Noiseless base rotation: current estimators, including flagged outputs',fontsize=14)
    fig.savefig(dst/'comparison.png',dpi=170);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
    for mu,c in zip([0,.29,.3,.31],['#777777','#a492ca','#147e9c','#48975b']):
        axs[0,0].plot(x,values(truth[mu],'frictionN'),'o-',ms=3,color=c,label=f'True mu = {mu}')
        axs[0,1].plot(x,values(truth[mu],'twistIntegralDeg'),'o-',ms=3,color=c,label=f'True mu = {mu}')
    axs[0,0].set(title='Physical tangential contact force',ylabel='N');axs[0,1].set(title='Accumulated material twist',ylabel='Degrees')
    axs[1,0].plot(x,values(sens,'frictionInducedShapeRmsMm'),'o-',color=colors[0]);axs[1,0].set(title='Physical shape change: true mu 0.3 versus 0',ylabel='RMS position difference (mm)')
    axs[1,1].semilogy(x,values(ours,'stage1ShapeRmsMm'),'o-',color=colors[1],label='Current Stage 1: intrinsic twist')
    axs[1,1].semilogy(x,values(sens,'offlineTrueTwistReconstructionRmsMm'),'o-',color=colors[0],label='Offline diagnostic: restore true twist')
    axs[1,1].set(title='Attribution of reconstruction error',ylabel='RMS position error (mm)')
    for ax in axs.flat:ax.set(xlabel='Base rotation (degrees)',xticks=[0,15,30,45]);ax.grid(alpha=.2)
    axs[0,0].legend(fontsize=8);axs[1,1].legend(fontsize=8);fig.suptitle('Forward physics and missing-twist diagnosis',fontsize=14)
    fig.savefig(dst/'physics_and_twist.png',dpi=170);plt.close(fig)
    end=[r for r in shapes if r['angleDeg']==45]
    fig=plt.figure(figsize=(12,5),layout='constrained');ax=fig.add_subplot(121,projection='3d');ax2=fig.add_subplot(122)
    for prefix,c,label in zip(['true','ours','zero','aloi','zeroTruth'],['black',*colors,'#888888'],['Frictional truth','Ours, mu = 0.3','Ours, mu = 0','Aloi','Frictionless truth']):
        ax.plot(values(end,prefix+'X'),values(end,prefix+'Y'),values(end,prefix+'Z'),color=c,label=label,lw=1.8)
    ax.set_box_aspect([np.ptp(ax.get_xlim()),np.ptp(ax.get_ylim()),np.ptp(ax.get_zlim())]);ax.set_xticks([0,30,60]);ax.set_yticks([0,20,40])
    ax.set(xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z (mm)',title='Shapes at 45 degrees');ax.legend(fontsize=7,loc='upper left')
    for key,c,label in zip(['trueTwistPerMm','oursTwistPerMm','zeroTwistPerMm','zeroTruthTwistPerMm'],['black',*colors[:2],'#888888'],['Frictional truth','Ours, mu = 0.3','Ours, mu = 0','Frictionless truth']):
        ax2.plot(values(end,'arcMm'),values(end,key),color=c,label=label)
    ax2.set(xlabel='Arc length (mm)',ylabel='Material twist u3 (1/mm)',title='Twist along the rod at 45 degrees');ax2.legend(fontsize=8);ax2.grid(alpha=.2)
    fig.savefig(dst/'final_shape_twist.png',dpi=170);plt.close(fig)
    last=ours[-1];ls=sens[-1];ft=truth[.3][-1];fzero=truth[0][-1]
    truth_audit={key:max(abs(r[key]) for seq in truth.values() for r in seq) for key in ['gapMm','tangency','penetrationMm','projectionN','terminalMomentNmm']}
    cond=[r['forceCondition'] for r in local if 'forceCondition' in r]
    report=f'''# Base rotation with centerline friction: current-method comparison

Date: 2026-10-06. Run `{args.run_id}`. Approved setup implemented separately from the earlier sliding experiment. No estimator physics or settings were changed; no commit or push was made.

## Main finding

Axial rotation of this precurved rod builds sticking friction and elastic material twist, then produces sliding. The correct friction model substantially improves contact/tip separation over mu=0, but the current two-stage estimator is less accurate and less reliable than in the planar sliding experiment. Stage 1 assumes intrinsic twist in an unmeasured channel, producing geometry bias even with perfect bending measurements. Correct friction alone cannot fix that reconstruction limitation.

| Method, all 13 frames including failures | Contact RMSE (N) | Tip RMSE (N) | Total-force RMSE (N) | Shape RMS (mm) |
|---|---:|---:|---:|---:|
| Ours, assumed mu=0.3 | {a['contactErrorN']:.5f} | {a['tipErrorN']:.5f} | {a['totalErrorN']:.5f} | {a['shapeRmsMm']:.5f} |
| Ours, assumed mu=0 | {z['contactErrorN']:.5f} | {z['tipErrorN']:.5f} | {z['totalErrorN']:.5f} | {z['shapeRmsMm']:.5f} |
| Tongyu Aloi | Not separated | Not separated | {b['totalErrorN']:.5f} | {b['shapeRmsMm']:.5f} |

Force RMSE pools squared three-dimensional error norm across frames; shape RMS pools squared position norm over all common nodes and frames. No failed or flagged estimate is excluded. Aloi estimates a single distributed Gaussian resultant, not separate contact and tip forces. Its position input is reconstructed from the same two bending channels, also with intrinsic twist assumed.

![Comparison](../../results/published/{args.run_id}/comparison.png)

## Fixture, actuation and observations

The old sliding case had fixed base orientation, insertion 18 to 22 mm, X translation -0.5 mm per recorded frame and independently prescribed 0.02 mm sliding history pairs. This new case holds the base at [0,0,18] mm and rotates its material frame about world/local-base Z through 0:3.75:45 degrees. The plane remains at z=160 mm with normal [0,0,-1]; the applied tip force remains [1,0,-1] N in world coordinates. The 200 mm rod retains its 120 mm straight segment, 30 mm intrinsic bend radius, stiffness and 24 FBG locations.

Initialization is a rest equilibrium at 0 degrees with zero tangential preload, unlike the old sliding initial state. Frame-zero history repeats that same resting shape at an earlier timestamp. Subsequent history is the actual preceding state. The timestamps retain the current 0.02 s estimator reference interval; these are quasistatic equilibria, not an inertial simulation or validation at the implied actuation speed. Force application is at the centerline: friction opposes motion in the plane and is not constrained to the rod tangent. No finite-radius surface friction torque or contact-couple jump was added. Elastic twist follows u3 = u03 + (R-transpose m)3 / Kt; a spatial force can therefore induce twist through its moment arm without an applied contact couple.

Only two bending components are observed. No FBG/environment noise is injected, while the previous covariance floors and priors are retained (FBG std 1e-6/mm, environment position std 0.03 mm and normal component std 0.001). The third packet entry is intrinsic twist, not measured true twist. Our observation residual still selects only axes 1 and 2. Neither estimator receives reference forces, contact locations or true twist.

Current settings remain Stage 1 trust-region-reflective lsqnonlin (30 iterations, function/optimality/step tolerances 1e-8/1e-6/1e-8), and Stage 2 semismooth SQP (60 iterations, 300 trials, stationarity 1e-5; moment 1e-5 N mm, projection 1e-6 N, gap 1e-5 mm). IVP relative/absolute settings remain 2e-8/2e-10, with component scaling of absolute tolerances. Diagnostic sampling is 0.1 mm, optimization collision sampling 0.5 mm. Full options and source hashes are in metadata.json. Diagnosis remains reporting-only.

## Independent reference and actual friction behavior

The new contact wrapper directly calls Tongyu's unmodified solve_cosserat_multi_contact_map; it does not use our inverse IVP or optimizer to generate reference data. The wrapper solves contact arc and reaction using sticking/sliding branches, and validates the full disk projection, friction work, tangency, gap, nonpenetration and terminal moment. The initial planar solve only initializes the zero-angle equilibrium. Subsequent shapes are genuinely spatial, not rigid rotations of planar truth.

The first numerical attempts found nonphysical sliding roots with positive friction work; those were rejected. The final forward wrapper uses multiple angular starting guesses and accepts only roots passing the dissipative projection law and physical audit. Rejected attempts are retained in local logs/debug outputs and are not benchmark truth. Valid quasistatic roots do not establish global uniqueness or dynamic stability.

For true mu=0.3, the contact sticks through 33.75 degrees; first sampled sliding occurs at {transitions['0.3']:.2f} degrees. The exact onset lies between these samples. At 45 degrees:

- Tangential friction magnitude: **{ft['frictionN']:.5f} N**; normal force: **{ft['normalN']:.5f} N**.
- Contact force: [{ft['contactFx']:.5f}, {ft['contactFy']:.5f}, {ft['contactFz']:.5f}] N.
- Accumulated material twist, integral u3 ds: **{ft['twistIntegralDeg']:.5f} degrees**, versus **{fzero['twistIntegralDeg']:.5f} degrees** for independently regenerated frictionless truth.
- Friction changes the true shape by **{ls['frictionInducedShapeRmsMm']:.5f} mm RMS** and true tip position by [{ls['frictionInducedTipXmm']:.5f}, {ls['frictionInducedTipYmm']:.5f}, {ls['frictionInducedTipZmm']:.5f}] mm, compared with frictionless physics at the same base angle.

The fixed world-frame tip force already generates some torsion when the rod rotates. The difference between physical mu=0.3 and mu=0 isolates friction's additional effect; total twist must not all be attributed to friction. Material twist is also distinct from geometric centerline torsion and is not simply base-angle minus tip-angle.

All 52 forward states (mu=0, 0.29, 0.3, 0.31) passed the audit. Maximum absolute gaps/tangency/penetration/projection/terminal moment are {truth_audit['gapMm']:.3g} mm, {truth_audit['tangency']:.3g}, {truth_audit['penetrationMm']:.3g} mm, {truth_audit['projectionN']:.3g} N and {truth_audit['terminalMomentNmm']:.3g} N mm. Tightening reference ODE tolerance tenfold at the final state changes position by at most {validation['finalReferenceTighterPositionMaxMm']:.3g} mm and curvature by {validation['finalReferenceTighterCurvatureMaxPerMm']:.3g}/mm. Full truth audits and reference_validation.json are retained.

![Physics and twist attribution](../../results/published/{args.run_id}/physics_and_twist.png)

## Estimator failure and geometry attribution

At 45 degrees, current Stage 1 position RMS error is **{last['stage1ShapeRmsMm']:.5f} mm**, contact-arc error is **{last['arcMm']-last['trueArcMm']:.5f} mm**, and normal-angle error is **{np.rad2deg(last['normalErrorRad']):.5f} degrees**. The inferred Stage 1 geometry is identical for mu=0 and mu=0.3.

An offline diagnostic takes the same interpolated bending measurements and restores reference u3 only during reconstruction. Its final shape RMS error is **{ls['offlineTrueTwistReconstructionRmsMm']:.6f} mm**. This is a truth-assisted attribution check, not another estimator or a reported deployable result. It shows how much reconstruction error is due to missing twist, while preserving bending interpolation error. No reference twist was used in either comparison run.

At 45 degrees our correct-mu tip estimate is [{last['tipFx']:.5f}, {last['tipFy']:.5f}, {last['tipFz']:.5f}] N, with {last['tipErrorN']:.5f} N error. The zero-mu tip estimate is [{zero[-1]['tipFx']:.5f}, {zero[-1]['tipFy']:.5f}, {zero[-1]['tipFz']:.5f}] N, with {zero[-1]['tipErrorN']:.5f} N error. Tip Y is a prominent compensation channel when the tangential contact load is removed. Nonzero world X/Y contact components at assumed mu=0 can arise from the inferred normal tilt; physics.csv separately reports tangential force in the inferred contact basis.

Our converged counts are {a['converged']}/13 at mu=0.3 and {z['converged']}/13 at mu=0. Failed angles are {a['failedAngles']} and {z['failedAngles']} degrees respectively. A convergence flag is not a clean physical diagnostic: finer diagnosis detects penetration or tangency mismatch in some returned converged states. Maximum dense inferred-plane penetration is {max(values(ours,'penetrationMm')):.6f}/{max(values(zero,'penetrationMm')):.6f} mm (mu=0.3/0). Final correct-mu flags: `{last['flags']}`. Per-frame flags, model friction work, inferred contact work against true tangential motion, and all SQP iterations are preserved. The latter uses the inferred total contact force projected onto the true plane; a nonzero value at assumed mu=0 reflects the tilted estimated normal, not an added tangential force in the estimated contact basis. In all three truly sliding frames, the mu=0.3 contact load does negative work against the actual motion; the positive-friction-work flags at 30/33.75 degrees concern failed solves and their inferred history.

Correct-mu flag counts: {a['flagCounts']}. Zero-mu flag counts: {z['flagCounts']}. Aloi returns {b['positiveExitFlags']}/13 positive optimizer exit flags and {b['equilibriumConverged']}/13 successful final equilibrium checks. These statuses are not force-accuracy certificates.

![Final shapes and material twist](../../results/published/{args.run_id}/final_shape_twist.png)

## Friction and force/deflection sensitivity

Two different experiments are kept separate:

1. **Forward physical sensitivity:** regenerate reference trajectories at true mu=0.29 and 0.31 with unchanged actuation and tip load. On the sticking branch, friction magnitude is set by equilibrium and kinematic sticking; mu only sets the allowable bound. Small mu changes therefore have virtually no effect while both trajectories remain stuck. Their first sampled sliding angles are {transitions['0.29']} and {transitions['0.31']} degrees. Near different transition angles, central differences span branches and are finite perturbations, not smooth derivatives.
2. **Inverse coefficient sensitivity:** replay the fixed true-mu=0.3 observations with assumed mu=0.29 and 0.31. These measure estimator/model sensitivity, including current geometry bias, history and stopping behavior. At the final frame, the finite dFtip/dmu is [{ls['dInverseTipX_dMu']:.4f}, {ls['dInverseTipY_dMu']:.4f}, {ls['dInverseTipZ_dMu']:.4f}] N per unit mu. Do not interpret these values as calibrated uncertainty, especially when a contributing solve does not converge; sensitivity.csv records both-converged status.

At 45 degrees, forward dFc/dmu is [{ls['dForwardContactX_dMu']:.4f}, {ls['dForwardContactY_dMu']:.4f}, {ls['dForwardContactZ_dMu']:.4f}] N per unit mu. The forward shape-change norm is {ls['dForwardShapeRmsMm_dMu']:.4f} mm per unit mu, and accumulated-twist change is {ls['dForwardTwistDeg_dMu']:.4f} degrees per unit mu. These use the full sequential paths, not a single isolated equilibrium with its history reset.

The local observed-curvature Jacobian eliminates base moment through terminal equilibrium and uses physical world-force coordinates [tip XYZ, contact XYZ]. Its six-force condition numbers range {min(cond):.2f}-{max(cond):.2f}. This diagnoses local force separation, not global identifiability; entries evaluated at failed estimates remain labelled with their convergence status. Full singular values, tip-Y/contact-Y correlations and constraint-tangent dimensions are in local_sensitivity.json. Geometry bias and weak load separation coexist here; the offline twist-restoration result directly exposes the former.

## Timing and numerical work

| Mean per frame | Ours mu=0.3 | Ours mu=0 | Aloi |
|---|---:|---:|---:|
| Full estimator seconds | {a['seconds']:.4f} | {z['seconds']:.4f} | {b['meanFrameSeconds']:.4f} |
| Geometry seconds | {a['geometrySeconds']:.4f} | {z['geometrySeconds']:.4f} | Not separated |
| Mechanics seconds | {a['mechanicsSeconds']:.4f} | {z['mechanicsSeconds']:.4f} | Not separated |
| SQP iterations | {a['sqpIterations']:.2f} | {z['sqpIterations']:.2f} | Not SQP |
| Full-rod IVP evaluations | {a['totalIVPs']:.2f} | {z['totalIVPs']:.2f} | No shooting IVP |
| Segmented ODE calls | {a['odeSegments']:.2f} | {z['odeSegments']:.2f} | Not this scheme |
| ODE RHS evaluations | {a['rhsEvaluations']:.1f} | {z['rhsEvaluations']:.1f} | Not comparable |
| Winning-start iterations | Not applicable | Not applicable | {b['winningStartIterations']:.2f} |
| Shape integrations | Different scheme | Different scheme | {b['shapeIntegrationsMean']:.2f} |

Our timing is the second full replay; checked numerical outputs agree between repeats to {difference:.3g}. Aloi's unprofiled sequence takes {b['sequenceSeconds']:.4f} s plus {b['preparationSeconds']:.4f} s reconstruction preparation. Its API does not expose per-frame times or summed iterations across both starts. Separate profiling counts {b['shapeIntegrationsTotal']} shape integrations; profiled elapsed time {profile['profiledSeconds']:.4f} s is excluded from the main timing comparison. Integration operations and warmup protocols differ; this is not a paper-level speedup claim. Aloi keeps the previous comparison's settings, including two starts, 25 maximum iterations, 180 evaluations, and 15 fixed-point updates with 0.001 mm tolerance. None of these estimator timings includes reference generation or sensitivity analysis.

## Reproduce and hand off

From two_stage_force_sensing, with the existing baseline dependency configured:

```matlab
setenv('TSFS_RUN_ID','{args.run_id}'); setup_tsfs;
build_rotation_truth(0.3);
run_rotation_comparison('ours');
run_rotation_comparison('aloi');
run_rotation_comparison('inverse');
run_rotation_comparison('forward');
run_rotation_comparison('profile');
analyze_rotation;
```

Then run `python experiments/base_rotation/summarize_rotation.py --run-id {args.run_id}`. Reference generation caches each coefficient's completed MAT file in the run directory; use a fresh base_rotation_ run ID for a new experiment. Large MAT files/logs remain local under ignored results/runs; compact evidence and plots are under results/published/{args.run_id}. The earlier single_contact_mu_20261006 outputs are preserved. The next scientific decision is how to handle unobserved twist in Stage 1; this work does not silently add twist sensing, a new objective, or a within-step correction loop.
'''
    (ROOT/'docs/reports/BASE_ROTATION_RESULTS.md').write_text(report,encoding='utf-8');(dst/'REPORT.md').write_text(report.replace(f'../../results/published/{args.run_id}/',''),encoding='utf-8')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
