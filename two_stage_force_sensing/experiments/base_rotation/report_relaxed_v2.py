"""Generate complete v2 evidence report without filtering unsuccessful frames."""
from pathlib import Path
import csv,json,numpy as np
R=Path(__file__).resolve().parents[2];P=R/'results/published/base_rotation_relaxed_v2_comparison_20261007'
def read(p):
 with p.open(newline='',encoding='utf-8-sig') as f:return list(csv.DictReader(f))
new=json.loads((P/'summary.json').read_text());old=json.loads((R/'results/published/base_rotation_relaxed_comparison_20261007/summary.json').read_text());A=read(P/'objective_scales.csv');B=read(P/'constraint_scales.csv');BO=read(R/'results/published/base_rotation_relaxed_scaling_20261007/constraint_scales.csv')
t=['# Revised rotating-rod solver comparison','',
'2026-10-07. All four requested configurations were run serially, with all failures retained. Previous results are preserved. No commit or push.','',
'## Requested configuration','',
'Force-prior penalties are disabled completely. Both stages use FBG standard deviation 1e-4 /mm (covariance 1e-8 I); samples remain noiseless and unchanged. Nonlinear constraint tolerance is 1e-6. Duplicate contact-gap rows are removed. Epsilon stays 1e-6, objective multiplier stays 1e-4, optimality target stays 1e-8, and the maximum iteration count stays 300. All variable and constraint Jacobian-row scaling stays unchanged. Previous-state numerical initialization and preceding-shape friction history remain.','',
'[Exact implementation/reproduction contract](../../experiments/base_rotation/RELAXED_COMPARISON_V2.md). Geometry still precedes mechanics once per frame; diagnostics do not reject or correct steps. Both increment sizes retain the original 0.02 s sample interval and their independently generated friction reference trajectory.','',
'## Complete results','',
'All errors below include every frame. Shape RMS aggregates points and frames. Timing includes geometry, mechanics setup/solve and diagnostics but excludes saving/export. Each IVP is an augmented full-rod variational integration.','',
'| Solver | Step | Frames | Native success/positive | Relaxed feasible | Tip RMSE N | Contact RMSE N | Shape RMS mm | Mean s/frame | Mean iterations | Mean IVPs |',
'|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
for s in new:t.append(f"| {s['solver']} | {s['stepDeg']:g}° | {s['frames']} | {s['nativeSuccess']} | {s['relaxedFeasibleCount']} | {s['tipErrorN_rms']:.6g} | {s['contactErrorN_rms']:.6g} | {s['shapeRmsMm_rms']:.6g} | {s['seconds_mean']:.4g} | {s['iterations_mean']:.4g} | {s['ivps_mean']:.4g} |")
t+=['','![Revised errors and costs](../../results/published/base_rotation_relaxed_v2_comparison_20261007/comparison.png)','',
'Native success, sampled relaxed feasibility and exact contact-law consistency are different criteria. fmincon exit 2 is a small-step stop, not its first-order success criterion. IPOPT acceptable-level exits are retained distinctly. No failure is removed from the means.','',
'## Comparison with the previous experiment','',
'| Solver / step | Tip RMSE old -> new (N) | Contact RMSE old -> new (N) | Mean time old -> new (s) | Iterations old -> new | IVPs old -> new |',
'|---|---:|---:|---:|---:|---:|']
for n in new:
 o=next(s for s in old if s['solver']==n['solver'] and s['stepDeg']==n['stepDeg'])
 t.append(f"| {n['solver']} / {n['stepDeg']:g}° | {o['tipErrorN_rms']:.5g} -> {n['tipErrorN_rms']:.5g} | {o['contactErrorN_rms']:.5g} -> {n['contactErrorN_rms']:.5g} | {o['seconds_mean']:.4g} -> {n['seconds_mean']:.4g} | {o['iterations_mean']:.4g} -> {n['iterations_mean']:.4g} | {o['ivps_mean']:.4g} -> {n['ivps_mean']:.4g} |")
t+=['','This is a combined four-change experiment. It cannot attribute improvement or deterioration uniquely to covariance, prior removal, tolerance or deduplication. With only FBG residuals, sigma changes Stage 2 objective scaling at fixed geometry/history; Stage 1 geometry changes too because its sensor/environment balance changes.','',
'## Statuses and physical diagnostics','',
'| Solver / step | Native status counts | Max tip error N | Max relaxed violation | Max original projection N |',
'|---|---|---:|---:|---:|']
for s in new:t.append(f"| {s['solver']} / {s['stepDeg']:g}° | {s['statusCounts']} | {s['tipErrorN_max']:.5g} | {s['relaxedViolation_max']:.5g} | {s['projectionN_max']:.5g} |")
t+=['','Updated `flags` use gap tolerance 1e-6 mm, moment tolerance 2e-4 N mm, squared-cone tolerance 1e-6 and product/duality-gap upper bound 2e-6 N mm. `exactLawFlags` separately retains the previous tighter physical warnings; all original residual magnitudes remain in the CSVs. The projection is not a constraint in this smooth relaxed NLP. A dense-grid penetration flag can occur even when sampled relaxed feasibility passes.','',
'| Solver / step | Max moment N mm | Max penetration mm | Max positive work N mm | Max cone excess N | Max arclength error mm | Max normal error deg |',
'|---|---:|---:|---:|---:|---:|---:|']
for s in new:t.append(f"| {s['solver']} / {s['stepDeg']:g}° | {s['tipMomentNmm_max']:.5g} | {s['penetrationMm_max']:.5g} | {max(0,s['frictionWorkNmm_max']):.5g} | {s['coneExcessN_max']:.5g} | {s['arcErrorMm_maxabs']:.5g} | {s['normalErrorDeg_max']:.5g} |")
t+=['','## Jacobian-row audit','',
'The fixed-state test verifies that changing FBG covariance leaves the entire mechanical constraint Jacobian unchanged. The whitened sensor Jacobian divides by 100, and the sensor objective/gradient by 10,000. Deleting duplicate rows leaves all retained constraint values/gradients exactly unchanged. Central-difference derivative checks passed at 0 and 22.5 degrees (relative errors about 3.87e-7 and 8.63e-8). New objectives have exactly 48 sensor terms; exported tip/contact prior costs are zero.','',
'At the actual terminal states, constraint rows can change because the solutions change. The following compares median terminal row norms and the largest slip-cone multiplier; these are in the unchanged optimization coordinates.','',
'| Solver / step | Normal-product row median old -> new | Squared-slip row median old -> new | Max squared-slip multiplier old -> new | Median initial objective gradient old -> new |',
'|---|---:|---:|---:|---:|']
AO=read(R/'results/published/base_rotation_relaxed_scaling_20261007/objective_scales.csv')
for s in new:
 tag='375deg' if s['stepDeg']==3.75 else '1deg';name=s['solver']
 def select(rows,phase,block=None):return [r for r in rows if r['step']==tag and r['solver']==name and r['phase']==phase and (block is None or r.get('block')==block)]
 vals=[]
 for block in ['normalProduct','slipConeSquared']:
  vals.append(' -> '.join(f'{np.median([float(r["JacobianRowNormMax"]) for r in select(rows,"final",block)]):.5g}' for rows in [BO,B]))
 vals.append(' -> '.join(f'{max(float(r["multiplierAbsMax"]) for r in select(rows,"final","slipConeSquared")):.5g}' for rows in [BO,B]))
 vals.append(' -> '.join(f'{np.median([float(r["scaledGradientInf"]) for r in select(rows,"initial")]):.5g}' for rows in [AO,A]))
 t.append(f"| {name} / {s['stepDeg']:g}° | "+' | '.join(vals)+' |')
t+=['','The old residual Jacobian included force-prior rows; the new one does not. Observation-Jacobian conditioning and derivative norms are not independent KKT certificates. A scalar likelihood scale does not directly repair vanishing squared-cone derivatives. The exact relationship between row balance and every convergence failure requires a controlled fixed-problem test, beyond this combined comparison.','',
'![Selected convergence paths](../../results/published/base_rotation_relaxed_v2_comparison_20261007/convergence.png)','',
'## Every frame','']
for s in new:
 tag='375deg' if s['stepDeg']==3.75 else '1deg';name=s['solver'];run=f'base_rotation_relaxed_v2_{tag}_20261007';rr=read(R/f'results/published/{run}/{name}_frames.csv')
 t += [f"### {name}, {s['stepDeg']:g} degrees",'',f'[Full frame data](../../results/published/{run}/{name}_frames.csv) · [Iteration traces](../../results/published/{run}/{name}_iterations.csv) · [Objective blocks](../../results/published/{run}/{name}_objective_blocks.csv)','',
 '| Angle | Status | Seconds | Iterations | IVPs | Tip error N | Contact error N | Shape RMS mm | Relaxed feasible |',
 '|---:|---|---:|---:|---:|---:|---:|---:|---:|']
 for r in rr:t.append('| '+' | '.join([r['angleDeg'],r['reason'],f"{float(r['seconds']):.4g}",r['iterations'],r['ivps'],f"{float(r['tipErrorN']):.5g}",f"{float(r['contactErrorN']):.5g}",f"{float(r['shapeRmsMm']):.5g}",r['relaxedFeasible']])+' |')
 t+=['',f"Updated flag counts: `{json.dumps(s['flagCounts'])}`",'',f"Historical exact-law warning counts: `{json.dumps(s['exactLawFlagCounts'])}`",'']
t.insert(t.index('## Every frame'), '## Main findings and remaining failures\n\nCoarse fmincon improved, but the changes are not a uniform improvement: fine fmincon and both IPOPT sequences have larger aggregate errors than before. A smaller rotation increment does not guarantee robustness. All averages include the first frame and unsuccessful outputs.\n\n- fmincon, 3.75 degrees: the main force outlier is 18.75 degrees (0.30736 N tip error, 57.425 s, 258 iterations, 4744 augmented IVPs). It exhausted the 5000 function-evaluation budget, not the 300-iteration budget. The next frame recovers to 0.000500 N tip error. Every sampled relaxed problem is feasible.\n- fmincon, 1 degree: 27 and 28 degrees have 0.114 and 0.242 N tip errors. The largest error is at 37 degrees: 1.33567 N tip, 1.06640 N contact, 0.67250 mm shape RMS, 41.551 s, 275 iterations and 3338 IVPs. This is exit 2 (small step), with stationarity 4.45e-4 rather than the requested 1e-8. Error declines after that frame and reaches 0.000163 N at 45 degrees. Only 12/46 frames meet the reported first-order target, despite 46 positive exits.\n- IPOPT: severe failure begins at 37.5 degrees for the coarse sequence and 38 degrees for the fine sequence. The coarse failure increases sensor cost from 36.277 to 862.706 while reducing sampled constraint violation to 1.86e-13. The fine failure increases sensor cost from 2.630 to 801.725 with violation 7.78e-16. These are failed solves, not evidence that the poor fits are competing optimal MAP solutions. The fine 42-degree output is even reported acceptable while retaining 3.555 N tip error and an FBG mismatch warning.\n\nAt those first IPOPT failures, normal force collapses from the warm-start values 8.630 N / 7.728 N to 5.03e-8 N / 6.58e-8 N. The auxiliary slip bound beta grows to 421.17 mm / 69.23 mm while actual slip is only 13.83 mm / 11.50 mm. At zero contact force, beta has no objective penalty and the friction duality expression loses its beta dependence; this is a concrete numerical weakness of the conic epigraph representation. It permits a physically admissible detached state, but does not make the poor sensor fit optimal. No new beta penalty, bound, cone treatment, or restart has been introduced.\n\nStage 1 is identical between solvers at each increment size and remains very accurate: maximum arclength errors are 0.002272 / 0.002659 mm and maximum normal errors are 0.000324 / 0.000395 degrees (coarse / fine). Gross geometry bias therefore does not explain the late failures; sensitivity to small geometry differences is not ruled out.\n\nRemoving force penalties does not remove recurrence: both the numerical seed and friction displacement use the preceding mechanical estimate. An inaccurate accepted frame can alter the next problem. At fine fmincon 38 degrees, native first-order convergence with 0.179 N error occurs after the bad 37-degree history. Separating that history effect from local optimization requires a controlled replay and has not been done.\n\nThe larger assumed sigma also weakens the mismatch warning in physical curvature units. At fine fmincon 37 degrees, curvature RMS error is 7.60e-5 /mm, only 0.760 sigma. Consequently its revised flags are empty despite the large force/shape errors; the historical contact-projection warning remains visible. Passing diagnostics is not a truth-accuracy certificate. No noise was injected in these runs.\n')
(R/'docs/reports/RELAXED_ROTATION_V2_RESULTS.md').write_text('\n'.join(t)+'\n',encoding='utf-8')
print('\n'.join(t[:55]))
