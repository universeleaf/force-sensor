# Current solver bottleneck examination

Historical report from the pre-migration workspace. Current entry points are in [README](../../README.md); preserved evidence is in [results](../../results/README.md). Original path references and timings describe that historical run.

Measured on 2026-10-05 with MATLAB R2025b, Optimization Toolbox 25.2, and an Intel Core i9-13900K. The proposed [2 stage force sensing](2_STAGE_FORCE_SENSING.md) formulation is preserved separately and has not been implemented. Production files under `rod/` were not changed.

The clean cases indicate that computational architecture, redundant friction coordinates, and geometry coupling dominate runtime. Contact and tip forces are locally separable in these cases, although their weakest observable directions contain opposing force changes and amplify noise. Freezing accurate geometry substantially reduces work; freezing inaccurate geometry can instead cause penetration or large force error.

## Scope and measurement method

Both requested demo sequences were regenerated from their current scene definitions and run for all 12 states. Their original MAT inputs are not included with the demo videos in the repository. The documented LCP-Continuum dependency was obtained at commit `56bfd089665efc95bba3e0e49ad11557cf206524`. The main repository baseline was `13bdb7d4d65b819229905781e6c4aede1f9ab64a`.

The demos use different estimators. `sliding_clean` calls the single-contact 3-D frictional MPCC, using 16 friction generators and an inner shooting solve. `s_channel_two_contact` calls the planar, frictionless, known-contact-order least-squares estimator, with fixed planes. It does not exercise the full 3-D MPCC. To examine that framework too, the archived two-frame `out/formulation_window/two_contact/input.mat` was replayed from its observation-based initialization.

All timed estimations ran sequentially. Timings exclude MATLAB startup, independent truth generation, video rendering, and the additional post-run sensitivity checks. Lightweight counters were inserted in isolated shadow copies under the audit output directory. These count existing calls without changing equations, tolerances, initializations, or numerical decisions. Source and shadow hashes are saved in the instrumentation manifest. They add some overhead, so these are diagnostic single-run timings, not a statistically controlled performance benchmark.

An IVP means one integration of a candidate rod state from base to tip. One IVP can require several `ode45` segments at contacts or intrinsic-curvature changes. An uncached single-contact equilibrium evaluation is a separate level: it uses `fsolve` to select the base moment and repeatedly evaluates whole-rod IVPs.

## Runtime and accuracy

| Estimator and scope | Time | Outer iterations | Whole-rod IVPs | Contact force RMSE | Tip force RMSE |
|---|---:|---:|---:|---:|---:|
| Sliding clean, 12 frames | 918.67 s total; 76.49 s mean/frame | 1,088 total, excluding seed refinement | 880,271 | 5.558e-6 N | 3.793e-6 N |
| Planar S channel, 12 frames | 3.049 s total; 0.254 s mean/frame | 79 total | 1,377 | 9.670e-11 N | 1.501e-10 N |
| Full 3-D two-contact window, 2 frames | 57.18 s total | 135 initialization + 4 SQP | 5,110 before covariance | 9.416e-7 N | 9.906e-7 N |

Sliding median time was 54.49 s/frame. Its first frame took 326.03 s; later frames took 45.23–62.39 s. The planar median was 0.185 s/frame, with 5–9 iterations and 91–151 IVPs per frame. Its first call took 0.937 s; later calls took 0.077–0.327 s. These paths differ in model dimension, friction, priors, and constraint treatment, so their speed ratio is not an isolated geometry effect. Two-frame window time is not streaming per-step latency.

These near-zero errors are ideal clean-model simulation results, not physical sensing precision. Sliding total-force RMSE was 2.258e-6 N; planar total-force RMSE was 1.546e-10 N.

![Measured timing, evaluation counts, and force sensitivity](../out/bottleneck_audit_20261005/bottleneck_summary.png)

## Why the sliding solver is expensive

The first sliding frame made 24,357 calls to the mechanics map, of which 9,985 missed its exact cache. Those evaluations required 169,733 whole-rod IVPs, 509,199 ODE segments, and 44,579,439 ODE right-hand-side evaluations. Its mechanics work consumed 288.08 of 326.03 seconds. Across all frames, mechanics consumed 789.75 seconds, about 86% of measured frame time, with no failed mechanics calls.

The average uncached equilibrium evaluation used almost 17 whole-rod IVPs. This is caused by the inner three-variable `fsolve` in [solve_cosserat_force_map.m](../rod/solve_cosserat_force_map.m). The optimizer does not merely call one IVP per iteration: it evaluates the objective and constraints repeatedly for finite-difference derivatives, and each new force state invokes nested shooting.

The outer sliding problem has 27 variables: 3 plane-point coordinates, 2 normal coordinates, 1 arc length, 1 normal force, 16 friction coefficients, 1 slip multiplier, and 3 tip-force components. Central differences require approximately 55 objective evaluations for a derivative sweep. The 16 friction coefficients generate a two-dimensional tangential force, so the mechanical map contains many redundant coefficient directions. Priors and active constraints restrict these directions, but do not eliminate their numerical cost.

The first frame's recorded continuation and polishing were:

| Stage | Iterations | Objective calls | Seconds | Exit | Constraint violation |
|---|---:|---:|---:|---:|---:|
| tau = 1e-2 | 100 | 5,558 | 153.17 | 0, iteration limit | 3.54e-13 |
| tau = 1e-4 | 62 | 3,467 | 127.53 | 1 | 1.65e-11 |
| tau = 1e-6 | 11 | 662 | 16.82 | 1 | 1.78e-13 |
| tau = 1e-8 | 34 | 1,909 | 23.34 | 2, small step | 3.02e-14 |
| Active-mode polish | 10 | 233 | 2.68 | 1 | 1.14e-14 |

The final polished state was accepted. Initial nonlinear shape refinement took only 1.09 seconds and 3 iterations. Thus initialization was not the main cost in this sliding frame. Much continuation work occurred after contact feasibility was already excellent, while optimizing a nearly flat objective.

The public quality flag reads the selected final solver exit, not every intermediate homotopy exit. Consequently it reports no optimization warning even though the first continuation stage hit its iteration limit. The original traces retain that exit. A successful final estimate should not be described as every stage converging.

An additional derivative-step check compared equilibrated force-to-FBG derivatives at force perturbations from 1e-8 to 1e-3 N. The 1e-8 N derivative differed from the 1e-4 N reference by only 2.56e-7 in relative Frobenius norm at the final first-frame state. This test does not support gross ODE derivative noise as the dominant explanation there; it does not certify derivatives at every optimizer trial.

## The full 3-D window has a different bottleneck

[integrate_cosserat_load_state.m](../rod/integrate_cosserat_load_state.m) already treats the base moment as an outer variable and enforces the terminal moment outside the IVP. It has no nested shooting solve. This distinction matters when selecting the implementation to simplify.

Its three initialization solves used 51, 51, and 33 recorded iterations, with the first two returning iteration-limit exits. Initialization and surrounding work inside the optimization timer used approximately 44.61 seconds. The four SQP stages used 9.11 seconds; remaining preprocessing and covariance work accounted for approximately 3.46 seconds of end-to-end time.

All four SQP stages made only one iteration each and selected the same objective, 4.947018. They terminated on small steps, with a force-weighted tangency residual around 7.85e-7. That passes the 1e-5 audit threshold but exceeds the 1e-8 exact-continuation stop threshold, so the implementation proceeds through all four stages. Final accuracy and feasibility are excellent here, but the trace is not evidence of strong first-order optimality convergence.

## Controlled geometry ablations

The geometry ablations fix plane parameters and contact arc lengths to values estimated by the completed baseline. Forces remain unknown and are optimized again; no true force or true contact coordinate is supplied. These are best-case diagnostic ablations, not implementations or timing predictions for Stage 1. The frozen geometry has already benefited from the full mechanical fit.

| Matched solve | Free geometry | Frozen geometry | Change |
|---|---:|---:|---:|
| Sliding first-frame time | 326.03 s | 55.47 s | 5.88 times faster |
| Sliding outer iterations | 217 | 93 | 57% fewer |
| Sliding whole-rod IVPs | 169,733 | 65,761 | 61% fewer |
| 3-D two-frame time, including covariance | 57.18 s | 22.63 s | 2.53 times faster |
| 3-D recorded mechanical evaluations | 5,110 | 2,997 | 41% fewer |

Sliding first-frame contact error changed from 1.884e-5 to 1.936e-5 N; tip error changed from 1.283e-5 to 1.318e-5 N. The frozen 3-D case had contact RMSE 7.000e-7 N and tip RMSE 6.000e-7 N. Geometry simplification therefore removes real numerical difficulty in these clean cases without materially harming accuracy. The remaining 55-second sliding solve still contains the old nested shooting and polygonal friction structure.

Do not interpret the extremely small frozen-geometry covariance as an achievable sensing interval: fixing geometry inferred from the same data removes uncertainty by assumption and can nearly determine the load through hard geometric constraints.

## Physics checks

All 12 sliding frames passed the original final checks. An independent mechanics re-evaluation used relative ODE tolerance 2e-8, terminal-moment tolerance 2e-5 N mm, and a 0.1 mm collision grid rather than the solver's 0.5 mm grid.

| Diagnostic | Sliding clean | Planar S channel |
|---|---:|---:|
| Maximum original complementarity residual | 3.89e-13 | Original audit retained in JSON |
| Maximum penetration in checked solution | 3.38e-7 mm, tighter sliding re-evaluation | 7.00e-10 mm |
| Maximum absolute normal-tangent residual | 1.58e-9 | 5.86e-14 |
| Maximum terminal-moment residual | 5.18e-10 N mm | 1.33e-11 N mm |
| Final review flags | 0 / 12 | 0 / 12 |

Sliding frictional work was negative in every frame, from -0.03532 to -0.02187 N mm. Circular-cone violation was zero, and the residual `ft dot slip + mu fn norm(slip)` was at most 1.19e-13 N mm. This clean replay shows no reversed friction or hidden deletion of the friction law. It does not establish correctness for arbitrary noisy slip directions or eliminate the polygonal-cone approximation in general. The small negative gap from the tighter re-evaluation is numerical tolerance, not an exact proof of continuous nonpenetration.

## Force separation and uncertainty

For sliding, the equilibrated map from six world-force components to the two measured bending channels has rank 6 in every frame. Its condition number ranges from 98.15 to 111.75. Five FBG positions lie downstream of contact in the first frame; the contact is about 39.94 mm from the tip. The forces are therefore not acting at an indistinguishable location in this case.

The weakest unit-norm force perturbation in the first frame is approximately

```
delta Fc = [ 0.455, 0, -0.588] N
delta Fe = [-0.476, 0,  0.469] N
```

Their sum has magnitude only 0.121 N. Opposing contact/tip changes really are a weak direction, but the curvature response is nonzero. The smallest singular value is 2.646e-5 per mm per N. At the declared curvature uncertainty of 1e-6 per mm, the FBG-only conditional component standard deviations are about 0.003–0.023 N. Multiplying the uncertainty by 25 multiplies this linear noise amplification by 25. These are local diagnostics at fixed arc length, without using the contact constraints to tighten the covariance; they are not calibrated posterior intervals.

For planar frame 6, the four-force equilibrated derivative has rank 4 and condition number 104.81. At an assumed curvature standard deviation of 2.5e-5 per mm, its corresponding FBG-only component uncertainties are approximately 0.56, 0.62, 0.34, and 0.66 N. Thus noise can make force separation an accuracy issue even when clean-case solving is well behaved.

The full 3-D two-frame constrained diagnostic reports all physical force components resolved by the observations. It has 28 locally feasible directions and measurement-information rank 26; the two remaining directions do not affect forces and are associated with frictionless auxiliary slack variables. The prior fills their information deficit. This is auxiliary-variable degeneracy, not evidence of unobservable physical forces in this particular case.

## Fixing incorrect geometry can break consistency

Using the same clean planar frame 6 observations, the first plane was displaced along its normal. Only the assumed curvature uncertainty and the optional plane-offset uncertainty changed. No random noise was added in these tests.

| Assumed FBG standard deviation | Plane error | Offset estimation | Contact RMSE | Maximum penetration |
|---|---:|---|---:|---:|
| 1e-7 per mm | 0.1 mm | Fixed | 0.000902 N | 0.09995 mm |
| 1e-7 per mm | 1 mm | Fixed | 0.009023 N | 0.99946 mm |
| 2.5e-5 per mm | 0.1 mm | Fixed | 3.250 N | 0.00704 mm |
| 2.5e-5 per mm | 1 mm | Fixed | 33.937 N | 0.07367 mm |
| 2.5e-5 per mm | 1 mm | Latent offset, 1 mm prior std | 0.000445 N | 1.00e-6 mm |

The planar estimator uses soft gap and tangency residuals. With extremely tight FBG weights it largely preserves shape and forces while violating geometry. With weaker FBG weights it distorts forces to accommodate the wrong plane, and still fails the physical audit. All fixed shifted-plane examples above trigger review despite positive solver exits. Accurate-looking forces or successful optimizer termination alone do not certify physical consistency.

For the proposed two-stage scheme, this supports estimating the plane offset in Stage 1 rather than treating a nominal 1 mm construction tolerance as exact geometry. Fixing normal and arc length may still be useful, but the fixed surface anchor must be consistent with the sensed rod within its uncertainty. Reporting-only diagnostics should explicitly expose the residual mismatch, as agreed.

## Priorities before implementation

1. Use the existing lifted IVP mechanics as the base for Stage 2. Avoid carrying the single-contact inner `fsolve` into the new formulation. The observed factor of roughly 17 IVPs per equilibrium call shows the work it introduces; it is not a guaranteed 17-fold end-to-end speedup after reformulation.
2. Replace the 16-generator friction representation and auxiliary multiplier with the agreed two-component disk projection and normal half-line projection. Preserve maximum dissipation as well as cone membership. Projection still has nonsmooth switching.
3. Separate uncertain geometric fusion from the mechanical update. The best-case ablations support this direction, but do not establish its speed or accuracy when Stage 1 is imperfect.
4. Reduce repeated finite-difference mechanics evaluations through consistent sensitivities and derivative reuse. Exact caching already helps; a smaller state alone does not eliminate derivative work.
5. Revisit initialization continuation and stopping criteria using physical residuals and objective progress. The present 3-D clean case spends most of its time in seed solves; the sliding case spends most in MPCC continuation. One generic iteration limit is not the same bottleneck in both.
6. Retain diagnosis of force separation under realistic noise. Fixing geometry cannot remove a physical force-null direction if sensors and contact placement make one present. The current clean examples do not exhibit such an exact force singularity.

## Reproducible artifacts

- [Raw run folder](../out/bottleneck_audit_20261005/): regenerated inputs, estimates, logs, counters, and source hashes.
- [Summary JSON](../out/bottleneck_audit_20261005/summary.json) and [sliding frame metrics CSV](../out/bottleneck_audit_20261005/sliding_frame_metrics.csv).
- [Independent residual and sensitivity analysis](../out/bottleneck_audit_20261005/independent_analysis.json).
- [3-D geometry ablation](../out/bottleneck_audit_20261005/window_frozen_summary.json) and [sliding geometry ablation](../out/bottleneck_audit_20261005/sliding_frozen_summary.json).
- [Instrumentation builder](../scripts/prepare_bottleneck_audit.py), [MATLAB runner](../scripts/run_bottleneck_audit.m), [independent analysis](../scripts/analyze_bottleneck_audit.m), and [summary/figure generator](../scripts/summarize_bottleneck_audit.py).

These results cover the specified clean scenarios, one two-frame full-formulation replay, and illustrative local ablations. They do not measure the proposed two-stage estimator, noisy mode transitions, global uniqueness, hardware accuracy, or real-time execution guarantees.
