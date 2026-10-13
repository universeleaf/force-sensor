# Linearized rod versus full nonlinear Cosserat: measured comparison

Date: 2026-10-11. Branch: codex/two-stage-force-sensing; base b376013. All work is local and uncommitted in our package. Collaborator folders were not changed.

## Outcome

Successive rod linearization reduces nonlinear IVP evaluations substantially, but it is not a uniform speed improvement. It improves insertion time, while difficult rejected contact modes make rotation slower. The full nonlinear solver remains the default; the new method is opt-in. Both retain the full circular friction law.

These are fresh sequential runs in MATLAB R2025b, using the same observations, geometry, physics, mode order, and fit thresholds. Historical timing values are not used as the speed baseline. The full solver reproduces the earlier insertion estimates exactly; rotation errors likewise reproduce the saved baseline. Times include Stage 1, all attempted mechanics modes, and diagnostics, but exclude simulation, MAT/CSV serialization, plotting, regression tests, and the diagnostic profile. Each configuration was timed once, so these are measured runs rather than a statistical hardware benchmark.

## Method

The original augmented IVP provides analytic sensitivities for Bishop measurements, contact positions, sampled collision positions and terminal moment. Affine predictions replace IVP calls inside each fmincon branch solve. Every candidate then receives a full nonlinear IVP check and a new Jacobian when accepted. The initial linearization is shared across attempted modes; later branch-specific trajectories have their own expansions. Stage 1 geometry and the preceding physical frame's mechanical history remain fixed during all outer corrections.

The circular sticking cone and the exact sliding equations remain unchanged. Sliding angle is still optimized. Thus the inner problem remains nonlinear in sliding coordinates; this implementation is not an MIQP, polygonal cone, frozen-friction-direction approximation, or C3 method. No force priors or measurement noise were added.

Full details: [formulation and numerical settings](../formulation/LINEARIZED_ROD_MODES.md).

## Complete trajectories

Insertion/withdrawal: 41 frames, perpendicular wall distance 80 -> 40 -> 80 mm. Rotation: 25 saved states, 0 -> 45 -> 0 degrees at 3.75-degree increments and fixed 40 mm wall distance. Rotation frame 1 restores the same original maximum-insertion checkpoint for both solvers and is excluded from timing/error aggregates, leaving 24 new solves. Each method then propagates its own previous estimate independently.

Both estimators use the same true mu=0.3, world tip load [1,0,0] N reference. Mu=0 is a frictionless inverse-model ablation, not a different physical trajectory.

| Scenario | Estimator mu | Mechanics | Mean / median / max time [s] | Mean IVPs | Mean inner iterations | Tip RMSE [N] | Contact RMSE [N] | Shape RMS [mm] |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| Insertion | 0.3 | Full nonlinear | 0.390 / 0.360 / 1.383 | 21.95 | 20.27 | 0.004407 | 0.003746 | 0.0003920 |
| Insertion | 0.3 | Linearized + correction | 0.284 / 0.184 / 4.253 | 4.98 | 34.88 | 0.007676 | 0.009175 | 0.0004130 |
| Insertion | 0 | Full nonlinear | 0.460 / 0.483 / 0.806 | 33.51 | 28.68 | 0.486778 | 3.379910 | 0.0407207 |
| Insertion | 0 | Linearized + correction | 0.279 / 0.273 / 0.461 | 8.00 | 48.76 | 0.487237 | 3.379430 | 0.0407250 |
| Rotation | 0.3 | Full nonlinear | 1.536 / 0.979 / 12.223 | 121.04 | 57.67 | 0.097548 | 0.345073 | 0.0155449 |
| Rotation | 0.3 | Linearized + correction | 4.916 / 0.771 / 47.247 | 16.63 | 451.83 | 0.097559 | 0.345072 | 0.0155452 |
| Rotation | 0 | Full nonlinear | 0.657 / 0.631 / 1.267 | 46.17 | 38.33 | 1.558521 | 5.507780 | 0.2963045 |
| Rotation | 0 | Linearized + correction | 1.485 / 0.676 / 19.386 | 16.42 | 303.46 | 1.558648 | 5.507800 | 0.2962696 |

All costs include losing branches. Iterations are total inner fmincon iterations across every outer correction, not the number of rod integrations. Mean outer-iteration counts for the linearized method: insertion mu=0.3 / mu=0: 3.98 / 7.00; rotation: 15.63 / 15.42. There is one shared initial IVP plus the candidate IVPs. No IVP exceptions occurred in the final comparisons.

For mu=0.3 insertion, mean runtime improves by 1.38x and IVPs decrease by 4.41x, with a small absolute increase in force error. Median time improves more than mean time because first contact has a slow losing branch. Rotation reduces IVPs by 7.28x but mean runtime becomes 3.20x worse. The median improves slightly while two long-tail mode searches dominate the mean.

## Convergence, fit acceptance, and physics

| Scenario / mu | Full: converged + feasible | Linearized: converged + feasible | Full / linearized FBG-gate acceptance |
|---|---:|---:|---:|
| Insertion / 0.3 | 41/41 | 41/41 | 41/41 and 41/41 |
| Insertion / 0 | 41/41 | 41/41 | 8/41 and 8/41 |
| Rotation / 0.3 | 24/24 | 24/24 | 10/24 and 10/24 |
| Rotation / 0 | 24/24 | 23/24 | 0/24 and 0/24 |

The linearized mu=0 return at frame 25 reaches the 40-outer-iteration limit. Its selected state is branch-feasible but nonconverged, and is included in all aggregate errors and timing. Its 19.386 s and 4,430 total inner iterations are not discarded.

The existing raw FBG RMS <= 1e-6 /mm acceptance rule remains. The same 14 correct-friction rotation frames fail it: 8, 13, and all return frames 14-25. All return modes are sticking for both methods. Diagnostics remain reporting-only.

Every published shape is a full nonlinear IVP result. For mu=0.3, maximum final implemented mode violation is 6.63e-9 in insertion and 2.82e-12 in rotation, below 1e-6. Projection residual maxima are 3.61e-12 N and 2.77e-12 N. Small dense-mesh penetration warnings persist: maxima 1.53e-5 mm and 2.40e-5 mm, essentially unchanged from the full solver. The linearized mu=0 rotation maximum implemented violation is 3.23e-7, still feasible despite the final nonconvergence flag. These checks do not eliminate the existing Stage 1/Stage 2 tangency mismatch.

## Where rotation time goes

At return frame 19 (22.5 degrees), the preferred sticking branch takes 0.074 s and four IVPs, but normalized FBG RMS is 0.03159, above the 0.01 gate. The scheduler consequently tries separation and sliding. The rejected sliding attempt uses 46.625 s and reaches the outer-iteration limit without feasibility. Total frame time is 47.247 s.

At frame 23 (return 7.5 degrees), selected sticking takes 0.072 s and four IVPs. The rejected sliding branch consumes 36.849 s, 5,391 inner iterations and 40 nonlinear candidate evaluations. Total frame time is 37.393 s. Both full and linearized approaches ultimately select the nearly identical sticking state.

A separate profile of the first five outer iterations of frame 19's rejected sliding branch reports 3.041 s in linearized_mode_branch, including 2.783 s in fmincon and 0.164 s in ivp. The affine evaluator itself uses only 0.015 s. Profile times are inclusive and must not be summed across parent/child functions. This diagnostic is excluded from benchmark times. It supports the interpretation that the remaining nonlinear sliding subsolver, rather than rod integration, dominates this failure path.

At insertion frame 4, the losing sticking branch similarly costs 4.018 s and reaches the outer limit. The selected sliding branch costs only 0.055 s. Conditional search therefore remains sensitive to the difficulty of modes encountered before acceptance.

The experiment supports keeping linearization as an optional acceleration for benign branches. It does not support replacing the full solver universally. A convex/conic subproblem treatment or a better bounded mode-search strategy is future work; changing the physical law or relaxing the fit threshold merely to suppress these costs was not done.

## Settings and validation

Unchanged: full circular friction, sensor-only objective multiplier 1e-4, Bishop sigma 1e-4 /mm, no force priors, fmincon SQP max 300 iterations / 5000 evaluations, optimality tolerance 1e-8, inner step tolerance 1e-12, constraint tolerance 1e-6, IVP relative/absolute tolerances 2e-8 / 2e-10. The new outer loop stops on a positive inner exit, nonlinear feasibility, and a scaled Cartesian step <= 1e-4; max 40 outer iterations. Trust radius initial/minimum/maximum: 4 / 1e-6 / 64. This outer criterion differs from the full solver's native termination and is not a global optimality guarantee.

Validation passed: exact affine expansion-point agreement; Taylor error ratio 0.2500 when halving a perturbation; relative analytic derivative errors 6.17e-10, 2.31e-10, 2.15e-10, 1.99e-10 for separation/sticking/sliding/frictionless inner problems; no IVP calls in affine callbacks; nonlinear-validated first rotation and insertion-reversal regression tests. Analysis verifies all 264 saved states, identical Stage 1 results, identical restored checkpoints, saved force/shape error consistency, full nonlinear state provenance, branch costs, previous-mode-first ordering, and early-exit/fallback semantics. It exports 1,195 outer iterations. The original evaluate.m and ivp.m source hashes are unchanged from before this work.

## Preliminary implementation failure

An earlier prototype changed the sliding-angle numerical seed to the preceding force direction and mishandled trust-box acceptance/expansion. Insertion reversal frame 22 then took 78.24 s and selected a poor separation fallback (tip error 16.65 N). The original displacement-based initializer was restored; out-of-box acceptance was disallowed; radius expansion after an infeasible solve now requires a boundary step. No physics or residual tolerances were changed. The final controlled reversal test passes with seven IVPs and tip error 0.000626 N.

The preliminary CSV/log and source snapshots are retained under the unsuffixed 20261011 tags. It was interrupted after 57 CSV rows; its MAT checkpoint was interrupted during serialization and is not a supported restart. Final comparisons use fresh 20261011_v2 tags and temporary-file checkpoint replacement. Preliminary outputs are not included in final aggregates.

## Outputs and reproduction

Summary, plots, aggregate metrics, and outer traces:
`results/published/linearized_modes_20261011_v2/`.

Per-frame estimates and every attempted branch:
`results/published/inclined_{insertion,rotation}_{full,linearized}_20261011_v2/`.

Corresponding complete MAT states are under `results/runs/` with the same tags. Test/profile logs and preliminary source hashes are under `results/runs/linearized_modes_20261011/`.

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
run_linearized_comparison('NEW_STAMP');
```

```text
python experiments/inclined_wall/analyze_linearized_comparison.py NEW_STAMP
python experiments/inclined_wall/export_linearized_trace.py NEW_STAMP
```

Use a new stamp for reruns. Earlier insertion and rotation videos and all prior published runs remain unchanged.
