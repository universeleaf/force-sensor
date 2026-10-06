"""Summarize the saved MATLAB comparison; no estimator or truth generation here."""
from pathlib import Path
import argparse
import csv
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]

def read_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        for key, value in row.items():
            try:
                row[key] = float(value)
            except ValueError:
                pass
    return rows

def col(rows, key):
    return np.array([r[key] for r in rows], dtype=float)

def rms(values):
    return float(np.sqrt(np.mean(np.array(values)**2)))

def stats(rows, aloi=False):
    result = {k: rms(col(rows, k)) for k in ['totalErrorN', 'shapeRmsMm']}
    if not aloi:
        result.update({k: rms(col(rows, k)) for k in ['contactErrorN', 'tipErrorN']})
        for key in ['seconds', 'geometrySeconds', 'mechanicsSeconds', 'diagnosticSeconds',
                    'geometryIterations', 'sqpIterations', 'qpIterations', 'stateIVPs',
                    'augmentedIVPs', 'totalIVPs', 'rhsEvaluations']:
            result[key] = float(col(rows, key).mean())
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', default='single_contact_mu_20261006')
    args = parser.parse_args()
    if not args.run_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.run_id):
        parser.error('run-id must be a simple directory name')
    src = ROOT / 'results' / 'runs' / args.run_id / 'single_contact'
    dst = ROOT / 'results' / 'published' / args.run_id
    dst.mkdir(parents=True, exist_ok=True)
    ours_all = read_csv(src / 'ours_frames.csv')
    ours = [r for r in ours_all if r['mu'] == .3 and r['repeat'] == 2]
    zero = [r for r in ours_all if r['mu'] == 0 and r['repeat'] == 2]
    aloi = read_csv(src / 'aloi_frames.csv')
    physics = read_csv(src / 'physics.csv')
    local = json.loads((src / 'local_sensitivity.json').read_text())
    meta = json.loads((src / 'metadata.json').read_text())
    timing = json.loads((src / 'aloi_timing.json').read_text())
    profile = json.loads((src / 'aloi_profile.json').read_text())
    finite = read_csv(src / 'mu_sensitivity.csv')
    assert len(ours) == len(zero) == len(aloi) == len(local) == 12
    assert meta['noiseMaximumPerMm'] == meta['environmentPointMaximumErrorMm'] == meta['environmentNormalMaximumError'] == 0
    assert all(r['converged'] == 1 and r['flags'] == '' for r in ours)
    assert all(r['converged'] == 1 and r['flags'] == 'fbg-mismatch' for r in zero)
    assert all(r['exitFlag'] > 0 and r['equilibriumConverged'] == 1 for r in aloi)
    assert all(r['stage1ArcDifferenceMm'] == r['stage1NormalDifference'] == 0 for r in local)
    assert all(r['bothConverged'] == 1 for r in finite)
    assert all(r['frictionWorkNmm'] <= 1e-8 for r in physics)
    # Deterministic repeats must agree in numerical outcomes; timing is excluded.
    repeat_error = 0.
    for row in ours_all:
        if row['repeat'] != 1:
            continue
        peer = next(r for r in ours_all if r['repeat'] == 2 and r['mu'] == row['mu'] and r['frame'] == row['frame'])
        for key in ['contactErrorN','tipErrorN','shapeRmsMm','sqpIterations','totalIVPs']:
            repeat_error = max(repeat_error, abs(row[key] - peer[key]))
    assert repeat_error < 1e-10
    a, z, b = stats(ours), stats(zero), stats(aloi, True)
    for result, mu in [(a, .3), (z, 0)]:
        result['ode45SegmentCalls'] = float(np.mean([r['ode45SegmentCalls'] for r in physics if r['mu']==mu]))
    b.update(timing)
    b['winningStartIterationsMean'] = float(col(aloi, 'iterationsWinningStart').mean())
    calls = {r['name']: r['calls'] for r in profile['functions']}
    b['shapeIntegrationsTotal'] = calls['solveShape']
    b['shapeIntegrationsMean'] = calls['solveShape'] / 12
    prediction_error = max(float(np.linalg.norm(np.array(r['linearTipChangeN']) - r['actualTipChangeN'])) for r in local)
    summary = {'runId': args.run_id, 'frames': 12, 'oursMu03': a, 'oursMu0': z, 'aloi': b,
               'repeatNumericalMaximumDifference': repeat_error, 'linearTipPredictionMaximumErrorN': prediction_error,
               'conditionRange': [min(r['condition'] for r in local), max(r['condition'] for r in local)],
               'feasibleConditionRange': [min(r['feasibleForceCondition'] for r in local), max(r['feasibleForceCondition'] for r in local)],
               'timingScope': 'Ours: full second replay, mean per frame including stages and diagnosis; Aloi: one unprofiled sequence / 12, preparation separate. Profile used for counts only.'}
    (dst / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    for name in ['metadata.json','ours_frames.csv','aloi_frames.csv','aloi_timing.json','aloi_profile.json',
                 'local_sensitivity.json','mu_sensitivity.csv','physics.csv','shape_profiles.csv']:
        shutil.copy2(src / name, dst / name)
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    colors = ['#177c9c', '#d36a1b', '#7652a1']
    x = col(ours, 'frame')
    fig, axes = plt.subplots(3, 2, figsize=(11, 10), constrained_layout=True)
    for ax, key, label in zip(axes.flat[:4], ['contactFx','contactFz','tipFx','tipFz'],
                              ['Contact force X','Contact force Z','Tip force X','Tip force Z']):
        truth_key = 'true' + key[0].upper() + key[1:]
        ax.plot(x, col(ours, truth_key), 'k--', lw=2.2, label='Truth')
        ax.plot(x, col(ours, key), 'o-', color=colors[0], ms=3, label='Ours, mu = 0.3')
        ax.plot(x, col(zero, key), 's-', color=colors[1], ms=3, label='Ours, mu = 0')
        ax.set(title=label, ylabel='N')
    for ax, key, title, unit in zip(axes.flat[4:], ['shapeRmsMm','totalErrorN'],
                                   ['Shape reconstruction error','Total force error'], ['mm', 'N']):
        for rows, color, label in zip([ours,zero,aloi], colors, ['Ours, mu = 0.3','Ours, mu = 0','Aloi (Tongyu)']):
            ax.semilogy(x, col(rows,key), 'o-', ms=3, color=color, label=label)
        ax.set(title=title, ylabel=unit)
    for ax in axes.flat:
        ax.set(xlabel='Frame', xticks=[1,3,6,9,12])
        ax.grid(alpha=.18)
    axes[0,0].legend(fontsize=8)
    axes[2,0].legend(fontsize=8)
    fig.suptitle('Noiseless single-contact sliding: force separation depends on friction', fontsize=14)
    fig.savefig(dst / 'comparison.png', dpi=170)
    plt.close(fig)
    shape_rows = read_csv(src / 'shape_profiles.csv')
    fig, axes = plt.subplots(1, 2, figsize=(11,4), constrained_layout=True)
    for ax, frame in zip(axes, [1,12]):
        rows = [r for r in shape_rows if r['frame']==frame]
        for prefix,color,label in zip(['ours','zero','aloi'],colors,['Ours, mu = 0.3','Ours, mu = 0','Aloi (Tongyu)']):
            err = np.sqrt(sum((col(rows,prefix+c)-col(rows,'true'+c))**2 for c in ['X','Y','Z']))
            ax.plot(col(rows,'arcMm'),err,color=color,label=label)
        ax.axvline(ours[frame-1]['arcMm'],color='gray',ls=':',label='Estimated contact')
        ax.set(title=f'Frame {frame}',xlabel='Arc length (mm)',ylabel='Position error norm (mm)')
        ax.grid(alpha=.18)
    axes[0].legend(fontsize=8)
    fig.suptitle('Reconstructed shape error along the rod')
    fig.savefig(dst / 'shape_error.png',dpi=170)
    plt.close(fig)
    r = local[0]
    report = f'''# Noiseless sliding: current method, friction ablation, and Aloi

Date: 2026-10-06. Run: `{args.run_id}`. This is a measured replay of the current implementations, not a change to the estimator formulation.

## Result

The correct friction coefficient recovers contact and tip loads to millinewton accuracy. Setting only the estimator coefficient to zero still converges, but moves the missing friction into a biased tip load and a weaker contact normal. The shape stays close to truth; the curvature residual does not. Every zero-friction frame raises `fbg-mismatch`. Tongyu's Aloi implementation returns a distributed-load resultant, not separate contact and tip estimates.

| Method | Contact force RMSE (N) | Tip force RMSE (N) | Total force RMSE (N) | Shape RMS (mm) |
|---|---:|---:|---:|---:|
| Ours, mu = 0.3 | {a['contactErrorN']:.6f} | {a['tipErrorN']:.6f} | {a['totalErrorN']:.6f} | {a['shapeRmsMm']:.6f} |
| Ours, mu = 0 | {z['contactErrorN']:.6f} | {z['tipErrorN']:.6f} | {z['totalErrorN']:.6f} | {z['shapeRmsMm']:.6f} |
| Aloi, Tongyu implementation | Not exposed | Not exposed | {b['totalErrorN']:.6f} | {b['shapeRmsMm']:.6f} |

Force RMSE is sqrt(mean over frames of squared 3D error norm). Shape RMS pools squared 3D position error over the common rod nodes and all 12 frames. It is not mean per-frame RMS. Total force is contact plus tip for ours and the integrated Gaussian resultant for Aloi. These totals have the same physical units, but do not imply matching force parameterizations.

![Force and shape comparison](../../results/published/{args.run_id}/comparison.png)

![Position error along the rod](../../results/published/{args.run_id}/shape_error.png)

## Controlled inputs and unchanged settings

- The existing `sliding_clean.mat` has 12 frames and {meta['fbgCount']} FBG locations, measuring two bending components. True sliding friction is 0.3 in both of our replays. The mu = 0 run is a deliberately incorrect inference model on the same frictional data, not regenerated frictionless truth.
- Measured bending samples match truth exactly; maximum difference is zero. Plane points and normals also match truth exactly. No noise was added. Nonzero likelihood/prior covariances remain: FBG standard deviation 1e-6/mm, environment position standard deviation 0.03 mm and normal-component standard deviation 0.001. These are assumed uncertainty scales, not realized noise.
- Stage 1 uses the existing reconstruction and MATLAB `lsqnonlin`, trust-region-reflective; PCHIP reconstruction was not replaced. Unmeasured twist retains the existing intrinsic/calibrated assumption. The synthetic fixture is planar; the general solver was not reduced to 2D.
- Stage 2 uses our existing semismooth SQP and exact circular friction projection, state/sensitivity IVPs, and recurrent force priors. Contact normal and arc length come from Stage 1. No new within-step correction loop or acceptance criterion was introduced. Diagnosis only reports.
- Geometry: max 30 iterations / 300 evaluations; function 1e-8, optimality 1e-6, step 1e-8. Mechanics: max 60 SQP iterations / 300 trials, QP max 200, stationarity 1e-5, terminal moment 1e-5 N mm, contact projection 1e-6 N, gap 1e-5 mm, complementarity product 1e-5 N mm. IVP relative/absolute tolerances 2e-8 / 2e-10.
- Force/tip prior standard deviations 35/4 N; process standard deviations 10/2 N at reference period 0.02 s. Diagnostic FBG whitened RMS threshold 4; tangency threshold 0.001. Full unchanged settings and dependency hashes are in metadata.json.
- Each of our two deterministic repetitions resets tracking at frame 1 and then advances all frames sequentially. Only `mu` in the current and previous sensor packets changes. All numerical outputs checked between repetitions agree to 1e-10. Stage 1 arcs and normals are identical across coefficients.
- Aloi uses Tongyu's unchanged `estimate_aloi_gaussian_baseline` and `measurements_from_sensor_packet`; its settings match `run_fair_baseline_protocol`: 15 center seeds, width seeds [3,8,16,30] mm, 2 optimizer starts, amplitude bound 100 N, position scale 0.2 mm, max 25 optimizer iterations / 180 function evaluations, inner equilibrium max 15 iterations and 0.001 mm tolerance with relaxation 0.5. Its lsqnonlin function/step tolerances are 1e-9/1e-8.

## Timing and convergence

| Quantity, mean per frame unless stated | Ours mu = 0.3 | Ours mu = 0 | Aloi |
|---|---:|---:|---:|
| Full estimator seconds | {a['seconds']:.4f} | {z['seconds']:.4f} | {b['meanFrameSeconds']:.4f} |
| Geometry seconds | {a['geometrySeconds']:.4f} | {z['geometrySeconds']:.4f} | Not separated |
| Mechanics seconds | {a['mechanicsSeconds']:.4f} | {z['mechanicsSeconds']:.4f} | Not separated |
| Diagnosis seconds | {a['diagnosticSeconds']:.4f} | {z['diagnosticSeconds']:.4f} | Different diagnostics |
| Geometry iterations | {a['geometryIterations']:.2f} | {z['geometryIterations']:.2f} | Not applicable |
| SQP iterations | {a['sqpIterations']:.2f} (2-3) | {z['sqpIterations']:.2f} | Not SQP |
| QP inner iterations | {a['qpIterations']:.2f} | {z['qpIterations']:.2f} | Not applicable |
| Winning optimizer start iterations | Not applicable | Not applicable | {b['winningStartIterationsMean']:.2f} |
| State IVPs | {a['stateIVPs']:.2f} | {z['stateIVPs']:.2f} | No shooting IVP |
| Sensitivity-augmented IVPs | {a['augmentedIVPs']:.2f} | {z['augmentedIVPs']:.2f} | No shooting IVP |
| Full-rod mechanics IVP evaluations | {a['totalIVPs']:.2f} | {z['totalIVPs']:.2f} | Not comparable |
| Segmented ode45 calls | {a['ode45SegmentCalls']:.2f} | {z['ode45SegmentCalls']:.2f} | Not used |
| Mechanics ODE RHS calls | {a['rhsEvaluations']:.1f} | {z['rhsEvaluations']:.1f} | Not comparable |
| Repeated `solveShape` integrations | Not this integration scheme | Not this integration scheme | {b['shapeIntegrationsMean']:.2f} |

Our timing is the second full replay; the first cold frame took 1.1983 s. Aloi's unprofiled full sequence took {b['sequenceSeconds']:.4f} s plus {b['preparationSeconds']:.4f} s sensor preparation, divided by 12 above. Its API does not expose per-frame unprofiled times or summed optimizer iterations over both starts. The recorded first winning-start iteration count is 26 despite the configured maximum 25; this is retained as returned, not rewritten. These timings have different warmup protocols and are not a statistically controlled paper-level speedup claim.

A separate Aloi profiling replay took {profile['profiledSeconds']:.4f} s and counted 24 `lsqnonlin` calls, 2,128 nonlinear Gaussian shape predictions, and 32,105 `solveShape` calls. The profiled time is excluded from the timing comparison. Inner fixed-point shape updates dominate the repeated work. Its shape integrations are not equivalent in dimension or cost to our 15-state / 150-state augmented ODE integrations, and our IVP counts exclude Stage 1 kinematic reconstruction. One counted IVP is a complete rod evaluation, internally split at contacts/material changes; physics.csv also reports the actual segmented ode45 call counts. No rejected mechanics trial was recorded in either of our replays. Geometry is now the largest part of our single-contact step (~0.103 s versus ~0.046 s mechanics with correct friction). The current ~0.15 s step is slower than the 0.02 s reference period.

All 12 mechanics solves converged for each coefficient. Aloi returned positive solver exit flags (10 at 3 and 2 at 2), and all final equilibrium checks passed. Its Gaussian width hits the 3 mm lower bound in every frame. Solver termination is not an accuracy guarantee. Aloi fits one transverse Gaussian load and uses a different segment integration convention from our Cosserat solve; this comparison characterizes the current code and settings, not all possible implementations of Aloi's method.

## Physics and diagnostics

| Maximum/range across frames | Ours mu = 0.3 | Ours mu = 0 |
|---|---:|---:|
| Whitened FBG RMS range | {col(ours,'fbgRms').min():.3f}-{col(ours,'fbgRms').max():.3f} | {col(zero,'fbgRms').min():.3f}-{col(zero,'fbgRms').max():.3f} |
| Penetration into inferred plane (mm) | {col(ours,'penetrationMm').max():.3g} | {col(zero,'penetrationMm').max():.3g} |
| Contact projection residual (N) | {col(ours,'projectionN').max():.3g} | {col(zero,'projectionN').max():.3g} |
| Absolute normal-tangent dot product | {col(ours,'tangency').max():.3g} | {col(zero,'tangency').max():.3g} |
| Terminal moment norm (N mm) | {col(ours,'tipMomentNmm').max():.3g} | {col(zero,'tipMomentNmm').max():.3g} |
| Diagnostic flags | None | FBG mismatch, all 12 |

The unchanged diagnostics also check friction-cone excess, complementarity product and positive friction work; no such flags appear. Saved per-contact values are in physics.csv. Zero friction satisfies its assumed friction law while failing the observations: the model can be mechanically feasible and physically incorrect for the real frictional interaction. A finite plane estimate remains even for noiseless data: the estimated contact arc differs from truth by at most {max(abs(v['arcMm']-v['trueArcMm']) for v in ours):.6f} mm. Truth-plane penetration at common output nodes reaches {col(ours,'truthPlanePenetrationMm').max():.6f} mm with mu = 0.3 and {col(zero,'truthPlanePenetrationMm').max():.6f} mm with mu = 0. This is separately labelled; it is not the dense inferred-plane diagnostic.

## Why friction changes the forces much more than the shape

At fixed Stage 1 geometry, eliminate base-moment variations using the linearized terminal-moment condition. Convert the resulting measurement Jacobian into physical world-force coordinates ordered [tip X,Y,Z; contact X,Y,Z], all in N. This leaves a 48-by-6 map G from force perturbations to observed bending curvature. No force priors are included in this measurement-only sensitivity.

Its six singular values are nonzero; condition numbers are {summary['conditionRange'][0]:.2f}-{summary['conditionRange'][1]:.2f}. Tip-X and contact-X columns have cosine correlation {min(v['tipXContactXCorrelation'] for v in local):.4f}-{max(v['tipXContactXCorrelation'] for v in local):.4f}. Thus this is a finite weak force-separation direction, not an exact singularity. Restricting force perturbations to the tangent space of the actual fixed-mu contact constraints leaves three directions and condition numbers {summary['feasibleConditionRange'][0]:.2f}-{summary['feasibleConditionRange'][1]:.2f}. These are different-dimensional maps: the comparison illustrates how a known contact law removes weak freedoms, not a proof of jointly identifying unknown mu.

For a quantitative counterfactual, remove the estimated tangential contact force, then adjust tip XYZ and the contact normal amplitude to minimize the linearized FBG error while preserving active gap and terminal moment. This predicts the zero-friction compensation without rerunning an optimizer. Predicted and actual shape-change RMS use the identical collision mesh:

| First-frame quantity | Linear prediction | Actual nonlinear difference, mu=0 minus mu=0.3 |
|---|---:|---:|
| Tip X change (N) | {r['linearTipChangeN'][0]:.6f} | {r['actualTipChangeN'][0]:.6f} |
| Tip Z change (N) | {r['linearTipChangeN'][2]:.6f} | {r['actualTipChangeN'][2]:.6f} |
| Shape-change RMS (mm) | {r['linearShapeChangeRmsMm']:.6f} | {r['actualShapeChangeRmsMm']:.6f} |

Across all frames, the largest tip-change prediction error is {prediction_error:.6f} N. Compensation reduces the curvature mismatch caused by simply deleting friction by {min(v['curvatureMismatchReductionFactor'] for v in local):.1f}-{max(v['curvatureMismatchReductionFactor'] for v in local):.1f} times. It does not eliminate it. This explains both the almost identical positions and the large, flagged curvature mismatch relative to the tight noiseless likelihood scale.

In frame 1 the true forces are contact [1.10373,0,-3.67909] N and tip [1,0,-1] N. With zero friction they become approximately contact [0,0,-2.48107] N and tip [2.12554,0,-2.03354] N. Contact X is about 8e-6 N rather than exactly zero because the inferred normal has a tiny tilt. All Y components remain at numerical zero; there was no imposed 2D restriction in our estimator.

The beam-moment identity also explains the coupling: upstream of contact, m(s) = (pc-p(s)) x Fc + (pL-p(s)) x Ftip. Exchanging force between tip and contact can produce a much smaller change in bending than in either force alone, especially along directions close to the free-segment chord. Shape position is an integrated observation and is still less sensitive. A good reconstructed shape is insufficient evidence for correct force separation.

Independent sequential runs at mu=0.29 and 0.31 give a first-frame central difference dFtip/dmu = [-4.9531,0,4.1973] N per unit mu. A coefficient change of +0.01 therefore corresponds locally to about [-0.0495,0,0.0420] N tip change, while the shape changes by only about 0.000446 mm RMS. All perturbed solves converge without flags. Later-frame finite differences vary substantially (shape sensitivity 0.045-0.925 mm per unit mu), so these are finite perturbations of the recurrent estimator at current stopping tolerances, not a verified infinitesimal derivative. Full rows are retained rather than smoothing this variation away.

## Interpretation and limits

This clean single-contact case is well behaved when friction is specified correctly, but still sensitive to force-model mismatch. Keeping mu is essential for load decomposition here. Dropping it is not a harmless simplification merely because the shape looks right. The existing FBG diagnostic catches the ablation under the current tight covariance. Realistic noise may mask part of that discrepancy; no robustness or unknown-mu identifiability claim follows from this noiseless experiment. Shape-objective changes, noise sweeps, better mu calibration, and altered solver settings are future decisions, not implemented here.

## Reproduction and provenance

From the package directory in MATLAB R2025b with Optimization Toolbox and the documented baseline dependencies:

```matlab
setenv('TSFS_RUN_ID','{args.run_id}');
setup_tsfs;
run_single_contact_comparison('ours');
run_single_contact_comparison('aloi');
analyze_single_contact_mu;
run_single_contact_comparison('profile');
export_single_contact_evidence;
```

Then `python experiments/single_contact/summarize_single_contact.py --run-id {args.run_id}` (NumPy and Matplotlib). Raw MAT files and logs remain under `results/runs`, ignored by Git; compact CSV/JSON evidence and plots are published under `results/published/{args.run_id}`. Source provenance in metadata.json records the directly called baseline files. The estimator core, collaborator checkout and external dependency were not modified.
'''
    (dst / 'REPORT.md').write_text(report.replace(f'../../results/published/{args.run_id}/', ''), encoding='utf-8')
    (ROOT / 'docs' / 'reports' / 'SINGLE_CONTACT_MU_COMPARISON.md').write_text(report, encoding='utf-8')
    print(json.dumps(summary,indent=2))

if __name__ == '__main__':
    main()
