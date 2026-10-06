from pathlib import Path
import csv,json,hashlib,subprocess,argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description='Regenerate the historical comparison report from preserved evidence.')
parser.add_argument('--results',type=Path,default=root/'results/published/two_stage_20261005')
parser.add_argument('--baseline',type=Path,default=root/'results/published/baseline_20261005')
parser.add_argument('--output',type=Path,default=root/'results/runs/comparison_report')
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
s=json.loads((args.results/'benchmark_summary.json').read_text());b=json.loads((args.baseline/'summary.json').read_text());cold=json.loads((args.results/'cold_start.json').read_text())
rows=list(csv.DictReader((args.results/'all_frames.csv').open()));ab=list(csv.DictReader((args.results/'geometry_ablations.csv').open()))
names={'sliding_clean':'Sliding, 1 contact','s_channel_two_contact':'S-channel, 2 contacts','two_contact':'3-D packet, 2 contacts','three_contact':'3 contacts','spatial_sliding':'Spatial sliding, 2 contacts','spatial_sliding_noisy':'Spatial sliding, noisy','spatial_three_channel':'Spatial + measured twist','free_space':'Free space'}
def fmt(v):return f'{v:.4g}'
def good(r):
 flags=set(r['flags'].split(';'))
 return r['converged']=='1' and not flags.intersection({'penetration','active-contact-tangency','terminal-equilibrium','contact-projection','friction-cone','normal-complementarity','positive-friction-work','fbg-mismatch'})
report='''# Implementation and measured results — 2026-10-05

The two-stage method is implemented in this separate project and replayed on the requested datasets. It removes the nested BVP bottleneck. It is **not ready to replace the current estimator**: frozen geometry produces significant multi-contact bias and some solver failures. Failed and flagged outputs are retained. The student's checkout was not modified during implementation.

Each physical step executes geometric MAP once, mechanics MAP once, and diagnostics once. No diagnostic-driven correction, frame rejection, force clipping, friction sign flip, or hidden mode-polishing pass was added. Numerical SQP iterations remain inside Stage 2.

## Requested comparisons

Times below are seconds per frame. The new warm median uses the second sequential replay. Baselines come from the retained earlier audit, not a parallel timing run. MATLAB startup, fixture loading, and serialization are excluded. Both requested datasets contain 12 frames.

| Replay | Old median s | New warm median s | Old mean iterations | New mean SQP steps | Old mean IVPs | New mean IVPs |
|---|---:|---:|---:|---:|---:|---:|
'''
for name,key,it in [('sliding_clean','sliding',1088/12),('s_channel_two_contact','planar',79/12)]:
 a=s[name];old=b[key];report+=f"| {names[name]} | {fmt(old['median_seconds'])} | {fmt(a['warmMedianSeconds'])} | {fmt(it)} | {fmt(a['meanSQPIterations'])} | {fmt(old['total_ivps']/12)} | {fmt(a['meanIVPs'])} |\n"
report+=f'''
A separate fresh-process first sliding estimate took **{cold['seconds']:.3f} s** (geometry {cold['geometrySeconds']:.3f} s; mechanics {cold['mechanicsSeconds']:.3f} s). Warm latency is not cold-start latency.

The sliding median is approximately **{b['sliding']['median_seconds']/s['sliding_clean']['warmMedianSeconds']:.0f} times lower**, with an accuracy tradeoff: contact-force RMSE changes from 5.56e-6 N to {s['sliding_clean']['contactRMSE_N']:.4g} N; tip RMSE from 3.79e-6 N to {s['sliding_clean']['tipRMSE_N']:.4g} N. All 12 sliding frames pass the physics and FBG checks. Friction work is nonpositive; no reversal is hidden.

The specialized 2-D S-channel baseline is faster and much more accurate than the new generic 3-D replay. Its old contact/tip errors were about 1e-10 N. The new adapter preserves its **one measured bending channel** and does not impose planar force constraints. Unobserved out-of-plane forces and shapes are therefore possible. Nine of twelve frames meet sampled solver convergence criteria; all twelve have quality warnings. This is not an equivalent-information speed or accuracy comparison.

The closer full 3-D reference took 57.18 s for a two-frame offline window (5,110 IVPs). The new causal pair takes approximately {2*s['two_contact']['warmMedianSeconds']:.3f} s, but with significant geometry bias and a different information set. This is not an equivalent-accuracy speedup.

## All retained outcomes

Contact RMSE is the square root of the mean squared Frobenius force error across contacts, ordered by arc length. Tip RMSE uses the vector norm. All finite outputs count, including failed/flagged estimates. Both deterministic repeats produced the same estimates. Frame counts below refer to one replay. Solver convergence uses the 0.5 mm grid and scaled KKT test; the independent 0.1 mm diagnostic can still find penetration or tangency mismatch.

| Dataset | Frames | Warm median / p95 s | SQP/frame | IVP/frame | Contact RMSE N | Tip RMSE N | Solver converged | Physics + FBG pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
'''
for name,a in s.items():
 rr=[r for r in rows if r['scenario']==name and r['repeat']=='2']
 report+=f"| {names[name]} | {len(rr)} | {fmt(a['warmMedianSeconds'])} / {fmt(a['warmP95Seconds'])} | {fmt(a['meanSQPIterations'])} | {fmt(a['meanIVPs'])} | {fmt(a['contactRMSE_N'])} | {fmt(a['tipRMSE_N'])} | {a['convergedFrames']//2}/{len(rr)} | {sum(good(r) for r in rr)}/{len(rr)} |\n"
report+='''
The extra twist-channel case is a separately labeled synthetic information ablation. Its first two channels exactly match the original spatial packet. The additional measurement is sampled from independent forward truth. It does not replace the original replay. Its remaining bias shows that missing twist alone does not explain geometric reconstruction errors.

![Measured comparison](../results/comparison.png)

## Where the difficulty moved

- **Repeated BVP work is gone.** Base moment is an optimization variable; terminal moment is an outer equality. The new core never calls a shooting root solver or `fsolve`. Each accepted/trial evaluation uses one state or augmented IVP. Augmented dimensions are 150, 195, and 240 for one, two, and three contacts. CSVs include RHS and segment counts; an augmented IVP is not the same cost as an old 15-state IVP.
- **Frozen geometry is the dominant measured bias in the full-channel planar multi-contact packets.** Sparse curvature interpolation, kinematic discretization, prior weighting, and tangency profiling shift arc lengths and offsets. Clean FBG then conflicts with hard Stage 2 geometry. Small geometric errors can cause large force errors. Measurement noise was not loosened to hide this.
- **Force ambiguity remains with incomplete sensing.** S-channel has one bending channel. The spatial packet has two channels despite elastic torsion from its loads. Stage 1 fills missing channels with calibrated intrinsic values. Stage 2 flags a material twist contradiction. This differs from the deferred friction-contact torque model.
- **The exact disk still needs globalization.** In sticking, its projection residual is locally zero inside the cone. A redundant exact circular-cone inequality supplies a useful QP boundary derivative; no friction polygon is used. Objective/merit scaling and an adaptive elastic penalty resolved an earlier spatial stall. Degenerate cases can still reach the iteration cap.
- **Sampled feasibility is weaker than continuous consistency.** Dense diagnostics find about 2e-5–4e-5 mm penetration in several solver-converged multi-contact outputs, above the declared 1e-5 mm check. Large clean-data FBG residuals and force errors are the larger practical issue. Diagnostics do not repair either.

## Geometry ablation: first frame only

This separate diagnostic test substitutes known arc lengths, normals, and offsets after Stage 1. It supplies no true forces or true shape initializer. It is not part of the sensing algorithm. The spatial first frame has no predecessor, so only cone feasibility is available, not observed sliding direction.

| Dataset | Geometry | SQP steps | Contact error N | Tip error N | FBG whitened RMS | Converged |
|---|---|---:|---:|---:|---:|---:|
'''
for r in ab:
 report+=f"| {names[r['scenario']]} | {r['mode']} | {r['iterations']} | {fmt(float(r['contactErrorN']))} | {fmt(float(r['tipErrorN']))} | {fmt(float(r['fbgRms']))} | {r['converged']} |\n"
report+='''
With accurate geometry, full 3-D two- and three-contact errors return to a few micronewtons, in four and seven steps. The one-channel adapter retains out-of-plane ambiguity even with known geometry. Spatial known-geometry forces are less precise: missing history, weak separation, tolerance and nonlinear convergence remain relevant. The evidence does not support blaming every failure solely on geometry or solely on contact-tip singularity.

## Actual settings and implementation details

| Component | Settings |
|---|---|
| Stage 1 | `lsqnonlin`, trust-region-reflective; 30 iterations, 300 objective calls; function 1e-8, optimality 1e-6, step 1e-8 |
| Stage 2 | Custom semismooth SQP; 60 major iterations, 300 trial evaluations |
| QP | `quadprog` active-set; 200 iterations; constraint 1e-9, optimality 1e-8 |
| IVP | ODE45 RelTol 2e-8; componentwise dimensionless AbsTol 2e-10 |
| Projection parameters | rho_n = rho_t = 1 N/mm |
| Initial globalization | Gauss–Newton damping 1e-6; scaled trust radius 1; scaled merit penalty 0.01, raised for persistent elasticity |
| Mechanics feasibility | Terminal moment 1e-5 N mm; projection 1e-6 N; gap 1e-5 mm; negative normal force 1e-8 N; cone 1e-6 N; force-gap product 1e-5 N mm |
| KKT | Scaled stationarity and inequality dual complementarity 1e-5; successful QP required |
| Diagnostics | Active force 1e-3 N; tangency 1e-3; whitened FBG RMS 4; unobserved twist departure 1e-5 /mm |

Stage 1 supplies a centered numerical Jacobian of its small geometric residual, including the changing correlated covariance. Kinematic FBG sensitivities also use centered differences; neither invokes Cosserat mechanics. Cross-contact covariance is retained. The explicit reconstruction floors are 0.001 mm closure and 1e-5 tangency. These initial engineering settings are not calibrated uncertainty bounds and do not fully represent sparse interpolation or missing twist. This is the local profiled MAP approximation, not an exact jointly optimized geometric shape.

Stage 2 fixes normals, arc lengths, and offsets. Contact world position remains the unknown mechanical position p(s;x). The objective is multiplied by a fixed positive scalar determined by its initial Jacobian. Temporary nonnegative elastic QP slacks aid globalization; persistent original physical residual is never certified as a relaxed solution. Every accepted trial checks the original circular projections. No Scholtes relaxation schedule is used.

Initial world-coordinate force prior standard deviations are contact 35 N / tip 4 N. Recurrent process standard deviations are contact 10 N / tip 2 N per 0.02 s reference period. Identity matching uses surface and nearest arc within 10 mm. This is recurrent constrained MAP filtering with declared Gaussian priors, not a calibrated full joint Kalman covariance. Stage 1 shape is not added as an independent pseudo-observation to Stage 2.

Matching predecessor timestamps use the previous mechanical posterior shape. The sliding packet's supplied predecessor generally differs from the previous displayed frame; paired reconstructed FBG history is explicitly labeled. No true slip is passed to the core. With missing history only cone feasibility is enforced.

## Verification and scope

Twelve focused checks passed: projection derivatives; maximum dissipation; zero-force apex, separation and zero friction; lifted IVP derivatives; ODE refinement; coordinate invariance; zero-contact/no-surface operation; immutable diagnostics; continuation after an invalid frame; contact activation branches. Separate genuinely spatial two-contact derivative and rigid-rotation checks passed. Relative mechanics Jacobian errors were 5.7e-9 and 4.6e-9. Tighter/rotated IVP positions differed by less than 6e-8 mm.

These tests do not establish global convergence, calibrated covariance, continuous collision freedom, arbitrary contact topology or reliable stick/slip transitions. Spatial truth prescribes sliding; it is not transition truth. The agreed deferred items are in [DEFERRED_WORK.md](DEFERRED_WORK.md): friction torque/twist, tip-environment contact, and edge/corner geometry. Unknown applied tip force remains estimated.

## Reproducibility and isolation

See [README](../README.md). `results/*_frames.csv` contains every frame's timing, counts, errors and flags. `*_benchmark.mat` retains estimates and iteration traces but omits serialized ODE closures. Use `benchmark_summary.json` and `release_validation.log` for final results; `development_*` directories preserve earlier outcomes.

The reference snapshot came from original commit `13bdb7d4d65b819229905781e6c4aede1f9ab64a`; copied LCP dependency commit `56bfd089665efc95bba3e0e49ad11557cf206524`. `results/isolation_manifest.json` records source/input hashes and unchanged checkout status. Seven pre-existing audit/planning additions in the student's checkout were left as found. Nothing was pushed to GitHub.
'''
(args.output/'IMPLEMENTATION_AND_RESULTS.md').write_text(report,encoding='utf-8')
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(1,3,figsize=(13.5,4.3),layout='constrained');x=np.arange(2);w=.35
old=[b['sliding']['median_seconds'],b['planar']['median_seconds']];new=[s[n]['warmMedianSeconds'] for n in ['sliding_clean','s_channel_two_contact']]
axs[0].bar(x-w/2,old,w,label='Previous',color='#8b98ab');axs[0].bar(x+w/2,new,w,label='Two-stage',color='#286fa8');axs[0].set_yscale('log');axs[0].set_ylim(.06,130);axs[0].set_ylabel('Median seconds / frame (log)');axs[0].set_xticks(x,['Sliding','S-channel']);axs[0].legend(frameon=False)
for i,(a,c) in enumerate(zip(old,new)):
 axs[0].text(i-w/2,a*1.15,f'{a:.3g}',ha='center');axs[0].text(i+w/2,c*1.15,f'{c:.3g}',ha='center')
for name,label,color in [('sliding_clean','Sliding','#286fa8'),('s_channel_two_contact','S-channel','#b04f38')]:
 rr=[r for r in rows if r['scenario']==name and r['repeat']=='2'];k=[int(r['frame']) for r in rr]
 axs[1].plot(k,[float(r['tipErrorN']) for r in rr],'-o',ms=3,label=label,color=color);axs[2].plot(k,[float(r['fbgRms']) for r in rr],'-o',ms=3,label=label,color=color)
for a in axs[1:]:a.set_yscale('log');a.set_xlabel('Frame');a.legend(frameon=False)
axs[1].set_ylabel('Tip-force error N (log)');axs[2].set_ylabel('Whitened FBG RMS (log)');axs[2].axhline(4,ls='--',color='black',lw=1,label='Warning threshold');axs[2].legend(frameon=False)
fig.suptitle('Two-stage sensing: fast sliding solve; unresolved multi-contact accuracy',fontsize=13);fig.savefig(args.output/'comparison.png',dpi=180);plt.close(fig)
print('Wrote implementation report and comparison figure.')
