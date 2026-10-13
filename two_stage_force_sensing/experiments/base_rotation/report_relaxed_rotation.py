"""Build the human-readable report from the published, unfiltered CSV files."""
from pathlib import Path
import json,csv
R=Path(__file__).resolve().parents[2];P=R/'results/published/base_rotation_relaxed_comparison_20261007'
summ=json.loads((P/'summary.json').read_text())
def rows(path):
 with path.open(newline='',encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def fmt(x):return f'{float(x):.6g}'
text=['# Direct 1e-6 relaxation: rotating-rod comparison','',
'Date: 2026-10-07. All four requested configurations were run serially, without continuation or restarts. This report includes unsuccessful solves and their consequences for later frames. The default estimator and all previous benchmark outputs remain unchanged. No commit or push.', '',
'## Scope and interpretation','',
'Noiseless simulated Bishop measurements; true/inverse friction coefficient 0.3. The same two-stage model, priors, 3D IVP and diagnostics are used. The circular-friction model uses a smooth conic duality-gap relaxation alongside scalar Scholtes normal complementarity. This is not a polygonal friction approximation. See [the mathematical and numerical contract](../formulation/RELAXED_MECHANICS_SOLVERS.md) and [reproduction instructions](../../experiments/base_rotation/RELAXED_COMPARISON.md).','',
'Both increments retain 0.02 s per frame and the same per-step priors. The fine-step reference is independently regenerated with its own preceding physical shape. Thus this compares 13 versus 46 updates, not equal rotation speed or identical friction trajectories. Reference solves and Bishop conversion are excluded from estimator timing. All timing is one serial workstation pass; first-frame JIT/loading is included, checkpoint I/O excluded.','',
'## Complete-sequence results','',
'Errors are RMS over all frames, including failures. Shape error is RMS over both centerline points and frames. Time includes geometry, mechanics and diagnostics. IVPs are full-rod augmented variational integrations; there are no nested BVP solves in these estimators.','',
'| Increment | Solver | Frames | Native positive/success exits | Tip RMSE (N) | Contact RMSE (N) | Shape RMS (mm) | Mean s/frame | Mean iterations | Mean IVPs | Sequence s |',
'|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
for s in summ:
 text.append(f"| {s['stepDeg']:g}° | {s['solver']} | {s['frames']} | {s['nativeSuccess']} | {s['tipErrorN_rms']:.6g} | {s['contactErrorN_rms']:.6g} | {s['shapeRmsMm_rms']:.6g} | {s['seconds_mean']:.4g} | {s['iterations_mean']:.4g} | {s['ivps_mean']:.4g} | {s['seconds_sum']:.5g} |")
text+=['','![Errors and computation](../../results/published/base_rotation_relaxed_comparison_20261007/comparison.png)','',
'Native success is not equivalent to accurate estimation or verified first-order stationarity. fmincon exit 2 means small-step termination with feasibility, not meeting its first-order optimality target. IPOPT acceptable-level exits are counted separately below. Its acceptable_iter was set to zero, but the returned status still includes acceptable-level outcomes; the recorded native status takes precedence over assumptions about the setting. Reported optimality quantities use each solver\'s convention and are not identical independent KKT certificates.','',
'## Stopping and physics','',
'| Increment / solver | Native status counts | Reported optimality <= 1e-8 | Maximum tip error (N) | Maximum projection residual (N) | Maximum relaxed violation |',
'|---|---|---:|---:|---:|---:|']
for s in summ:
 text.append(f"| {s['stepDeg']:g}° / {s['solver']} | {s['statusCounts']} | {s['reportedOptimalityAt1e8']} | {s['tipErrorN_max']:.6g} | {s['projectionN_max']:.6g} | {s['relaxedViolation_max']:.6g} |")
text+=['','| Increment / solver | Max terminal moment (N mm) | Max estimated-plane penetration (mm) | Max true-plane penetration (mm) | Max positive friction work (N mm) | Max cone excess (N) | Max normal product (N mm) |',
'|---|---:|---:|---:|---:|---:|---:|']
for s in summ:
 text.append(f"| {s['stepDeg']:g}° / {s['solver']} | {s['tipMomentNmm_max']:.5g} | {s['penetrationMm_max']:.5g} | {s['truthPlanePenetrationMm_max']:.5g} | {max(0,s['frictionWorkNmm_max']):.5g} | {s['coneExcessN_max']:.5g} | {s['normalProductNmm_max']:.5g} |")
text+=['','A fixed 1e-6 N mm friction duality gap is not a 1e-6 N force-projection guarantee. On the sliding boundary, squared friction-direction error is bounded by 2 R epsilon / ||slip||. Smaller per-step slip can therefore loosen the force-direction allowance for the same epsilon. Near sticking, the squared slip cone also has small derivatives; finite constraint tolerance can translate into a larger slip error. The mathematical contract derives these effects.','',
'## Stage 1','',
'| Increment / solver | Maximum arclength error (mm) | Maximum normal error (degrees) |',
'|---|---:|---:|']
for s in summ:text.append(f"| {s['stepDeg']:g}° / {s['solver']} | {s['arcErrorMm_maxabs']:.6g} | {s['normalErrorDeg_max']:.6g} |")
text+=['','Stage 1 is independent of the mechanics solver, and the duplicate geometry columns permit checking that both solvers received the same geometry. Small Stage 1 errors do not guarantee good nonlinear solver convergence. These runs do not separately identify geometry bias, nonconvex branches and stopping behavior as causal sources of every outlier.','',
'## Convergence paths and interpretation','',
'![Selected convergence paths](../../results/published/base_rotation_relaxed_comparison_20261007/convergence.png)','',
'- Coarse IPOPT has 0.00435 N tip-force RMSE and avoids the previous custom-SQP 22.5-degree error of about 0.71 N. Its own 22.5-degree error is 0.0130 N and its native status is unsuccessful. At 41.25 degrees its error is 0.000156 N with success. This is strong estimation accuracy, not uniform convergence.','- Coarse fmincon develops 0.474 N tip error at 26.25 degrees, reaches 300 iterations at 33.75 degrees with 3.93 N error, and has 1.21 N error at 37.5 degrees before recovering. Its 33.75-degree MAP objective is about 5.60e7. The preceding finite state continues to feed history and priors under the existing policy.','- At 1 degree, fmincon improves to 0.0637 N tip RMSE and 2.17 s/frame, but has a 0.432 N outlier at 27 degrees. All 46 exits are small-step exits, none satisfy its first-order criterion. IPOPT has a 0.619 N outlier at 24 degrees and a failure chain starting at 41 degrees; errors reach 4.74 N and the final frame is still inaccurate. Its 1-degree tip RMSE is 1.233 N and mean time is 11.84 s/frame.','- Smaller increments help fmincon in this particular run, but do not establish a robust direct-relaxation method. The two solvers received identical Stage 1 geometry at each increment; solver/globalization behavior and recurrent history differ. These data cannot by themselves establish global nonuniqueness or assign every bad result to one cause.','- IPOPT often spends a substantial tail trying to certify stationarity after the objective and feasibility have stabilized. For the coarse 3.75-degree frame, the saved trace first reaches feasibility and within 1% (or 0.001 absolute) of its terminal cost at trace index 71, but the reported total is 161 iterations with a failed step computation. Trace indices can include restoration entries.','- These results do not establish that a four-level continuation schedule is required. They establish that this direct-1e-6 formulation and these settings are not uniformly robust. The next controlled tests should examine constraint/optimality scaling and the sticking cone, then compare a short 1e-4 -> 1e-6 schedule against this preserved reference. No additional continuation has been run or substituted here.','- The historical custom semismooth SQP averaged approximately 0.272 s/frame, 7.62 iterations and 16.62 total IVPs on the 13-frame Bishop test, with tip RMSE 0.197 N. Its reported 13/13 convergence includes the previously audited premature 22.5-degree stop; its loose convergence test and mixed state/variational IVPs make an equivalent-accuracy speed comparison inappropriate.','',
'## Settings and validation','',
'Both solvers: epsilon=1e-6; fixed objective multiplier 1e-4; 300 iterations; optimality target 1e-8; constraint tolerance 1e-9. fmincon SQP uses analytic first derivatives and step tolerance 1e-12. IPOPT 3.14.19 via CasADi 3.8.0 uses limited-memory Hessian, no additional NLP scaling and no bound relaxation. ODE relative tolerance 2e-8, scaled absolute tolerance 2e-10. The same initial state/prior policy is retained; multipliers and Hessians are not transferred between frames.','',
'Constraint derivative central-difference checks passed at 0, 22.5 and 41.25 degrees (relative discrepancies 3.87e-7, 8.61e-8 and 3.57e-8). The zero-force/nonzero-slip cone test passed. Every fine-step mechanical reference frame passed its independent physical audit. Full settings, objective blocks, solver traces and frame CSVs are published in the two run directories. The dependency archive is stored only in ignored package-local storage, including its license notices.','',
'## Every frame','',
'Flags and all force components, residual blocks, geometry errors, diagnostic timing and ODE RHS counts are in the linked CSV files. Tables below retain every native failure rather than filtering the averages.']
for s in summ:
 tag='375deg' if s['stepDeg']==3.75 else '1deg';run=f'base_rotation_relaxed_{tag}_20261007';name=s['solver'];rr=rows(R/f'results/published/{run}/{name}_frames.csv')
 text+=['',f"### {s['stepDeg']:g} degrees, {name}",'',f'[Complete frame data](../../results/published/{run}/{name}_frames.csv) · [Iteration trace](../../results/published/{run}/{name}_iterations.csv) · [Objective blocks](../../results/published/{run}/{name}_objective_blocks.csv)','',
 '| Angle | Native status | s/frame | Iterations | IVPs | Tip error N | Contact error N | Shape RMS mm | Projection N |',
 '|---:|---|---:|---:|---:|---:|---:|---:|---:|']
 for r in rr:
  text.append('| '+' | '.join([r['angleDeg'],r['reason'],fmt(r['seconds']),r['iterations'],r['ivps'],fmt(r['tipErrorN']),fmt(r['contactErrorN']),fmt(r['shapeRmsMm']),fmt(r['projectionN'])])+' |')
 text+=['',f"Diagnostic flag counts: `{json.dumps(s['flagCounts'])}`"]
(R/'docs/reports/RELAXED_ROTATION_COMPARISON.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
