# Single-pass rod and octagonal-friction comparison

Date: 2026-10-12. Author: Codex, mentor-side collaborator. Final stamp `20261012_unbounded`. Branch `codex/two-stage-force-sensing`, base b376013. All 16 requested new-method trajectories completed. User approved comparison against available baselines and stopping the optional extra controls. No commit or push.

The user approved removing the single-pass trust box after a preliminary radius-4 test blocked insertion reversal. The final comparison uses no step bound for either single-pass method. The outer-loop reference keeps its existing adaptive trust region. Preliminary bounded runs are preserved and excluded from the tables.

## Experiment controls

Each method uses the same 41-frame insertion trajectory (80 to 40 to 80 mm perpendicular wall distance), and 24 newly solved rotation frames (0 to 45 to 0 degrees in 3.75-degree increments) at maximum insertion. Rod length 200 mm: 80 mm straight + 120 mm precurved by 90 degrees. True tip force [1,0,0] N; mu=0.3. The inverse variables and tangent-plane friction remain 3D. Rotation begins from that method's insertion frame 21; its restored first frame is excluded from scores. This includes history feedback and is not a matched-state one-frame causal ablation.

Noise is iid Gaussian in 24 x 2 Bishop channels, using the same seed33001 draws for every method. The 0/.5/1/2 mm labels are geometric endpoint RMS calibration targets; sigmas are 0, 9.26814e-5, 1.85363e-4, 3.70726e-4 mm^-1. Actual realization RMS is 0/.505/1.011/2.029 mm insertion and 0/.516/1.024/2.049 mm rotation. Zero injected noise keeps the existing finite covariance scale 1e-4 mm^-1. No environment or mu noise was added. Stage 1 geometry is checked to match across methods.

- Outer-loop circular: exhaustive 3 contact modes; up to 40 rod outer iterations per mode, 300 SQP iterations per inner solve; true constraint tolerance 1e-6 and outer step tolerance 1e-4. The existing bit-identical 2 mm exhaustive runs are reused; the zero-noise exhaustive reference is new and complete. The extra 0.5 mm reference was interrupted after five frames and excluded; the 1 mm reference was not run. The user approved this reporting scope.
- One-pass circular: one rod Jacobian per frame; exhaustive 3 modes; fmincon SQP with 100 iterations per mode; exact circular friction on affine rod outputs.
- One-pass octagon: one rod Jacobian per frame; inscribed eight-sided polygon; exhaustive separation, sticking, eight edge-sliding and eight vertex-sliding cases (18 branches/contact). Each branch is a convex quadprog problem, maximum 100 iterations. Both edge and vertex branches are required for polygon maximum dissipation.
- Both one-pass methods: no outer correction, no trust box, no force priors; a final nonlinear IVP diagnoses the selected force solution without refinement or reselection. Mode selection uses affine feasibility/objective; the resulting nonlinear state is the next-frame history. Exactly two IVPs per new frame, including validation.

Timing includes geometry estimation, all branches, and nonlinear validation. It excludes MATLAB startup and checkpoint serialization. SQP and interior-point QP iteration counts measure different work; compare wall time and IVPs as well. These are single-run timings, not repeated performance confidence intervals.

## Insertion

Accuracy (force errors are RMS vector norms):

| Noise mm | Method | Tip N | Contact N | Contact error % | Affine shape mm | Nonlinear shape mm | Nonlinear tip position mm |
|---|---|---|---|---|---|---|---|
| 0 | Outer-loop circular | 0.007676 | 0.009175 | 0.08812 | 0.000413 | 0.000413 | 0.0007199 |
| 0 | One-pass circular SQP | 0.01013 | 0.04729 | 0.4542 | 0.02436 | 0.06823 | 0.1796 |
| 0 | One-pass octagon QP | 0.01044 | 0.04797 | 0.4607 | 0.02444 | 0.06829 | 0.1797 |
| 0.5 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 0.5 | One-pass circular SQP | 0.7506 | 0.8245 | 7.919 | 0.1304 | 0.1387 | 0.3083 |
| 0.5 | One-pass octagon QP | 0.7507 | 0.8321 | 7.992 | 0.1513 | 0.1585 | 0.3362 |
| 1 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 1 | One-pass circular SQP | 1.528 | 1.723 | 16.55 | 0.2672 | 0.2699 | 0.5959 |
| 1 | One-pass octagon QP | 1.537 | 1.746 | 16.77 | 0.308 | 0.3104 | 0.6542 |
| 2 | Outer-loop circular | 3.335 | 3.793 | 36.43 | 0.6196 | 0.6196 | 1.307 |
| 2 | One-pass circular SQP | 3.32 | 3.774 | 36.25 | 0.6381 | 0.6696 | 1.44 |
| 2 | One-pass octagon QP | 3.311 | 3.775 | 36.26 | 0.6957 | 0.7215 | 1.522 |

Cost and convergence:

| Noise mm | Method | Mean s | Median s | Max s | Total inner iterations/frame | IVPs/frame | Model solve success | Nonlinear mode feasible + converged |
|---|---|---|---|---|---|---|---|---|
| 0 | Outer-loop circular | 32.3 | 13.58 | 657.5 | 276.1 | 44.32 | 41/41 | 41/41 |
| 0 | One-pass circular SQP | 0.7834 | 0.661 | 3.363 | 49.49 | 2 | 41/41 | 5/41 |
| 0 | One-pass octagon QP | 1.963 | 1.95 | 3.72 | 469.4 | 2 | 41/41 | 5/41 |
| 0.5 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 0.5 | One-pass circular SQP | 0.7759 | 0.7616 | 1.847 | 112 | 2 | 41/41 | 0/41 |
| 0.5 | One-pass octagon QP | 1.859 | 1.877 | 3.813 | 463.4 | 2 | 41/41 | 0/41 |
| 1 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 1 | One-pass circular SQP | 0.7936 | 0.736 | 2.021 | 112.8 | 2 | 41/41 | 0/41 |
| 1 | One-pass octagon QP | 1.927 | 1.937 | 3.498 | 484.8 | 2 | 41/41 | 0/41 |
| 2 | Outer-loop circular | 28.93 | 17.18 | 149.2 | 590.7 | 61.71 | 36/41 | 36/41 |
| 2 | One-pass circular SQP | 0.664 | 0.6338 | 2.396 | 115.5 | 2 | 41/41 | 0/41 |
| 2 | One-pass octagon QP | 1.924 | 1.922 | 4.028 | 477.9 | 2 | 41/41 | 0/41 |

For the reference, model solve success is outer convergence plus nonlinear feasibility. For one-pass methods it is native inner convergence plus affine feasibility. The last column checks the selected mode after nonlinear forward evaluation; for the octagon it checks its own polygon law, not exact circular Coulomb friction.

## Rotation

Accuracy (force errors are RMS vector norms):

| Noise mm | Method | Tip N | Contact N | Contact error % | Affine shape mm | Nonlinear shape mm | Nonlinear tip position mm |
|---|---|---|---|---|---|---|---|
| 0 | Outer-loop circular | 0.09756 | 0.3451 | 1.521 | 0.01555 | 0.01555 | 0.03753 |
| 0 | One-pass circular SQP | 0.04595 | 0.1434 | 0.6322 | 0.01358 | 0.01344 | 0.03302 |
| 0 | One-pass octagon QP | 0.06034 | 0.2799 | 1.234 | 0.01865 | 0.0187 | 0.03785 |
| 0.5 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 0.5 | One-pass circular SQP | 1.024 | 3.043 | 13.41 | 0.1276 | 0.1278 | 0.3137 |
| 0.5 | One-pass octagon QP | 1.043 | 3.085 | 13.6 | 0.1591 | 0.1593 | 0.3481 |
| 1 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 1 | One-pass circular SQP | 1.815 | 4.768 | 21.02 | 0.2788 | 0.2793 | 0.7066 |
| 1 | One-pass octagon QP | 1.815 | 4.734 | 20.87 | 0.3372 | 0.3388 | 0.7654 |
| 2 | Outer-loop circular | 4.985 | 8.8 | 38.8 | 2.102 | 2.102 | 3.356 |
| 2 | One-pass circular SQP | 3.521 | 7.764 | 34.23 | 0.6129 | 0.6172 | 1.513 |
| 2 | One-pass octagon QP | 3.505 | 7.705 | 33.97 | 0.622 | 0.6269 | 1.48 |

Cost and convergence:

| Noise mm | Method | Mean s | Median s | Max s | Total inner iterations/frame | IVPs/frame | Model solve success | Nonlinear mode feasible + converged |
|---|---|---|---|---|---|---|---|---|
| 0 | Outer-loop circular | 5.182 | 0.7438 | 51.85 | 479.8 | 22.33 | 24/24 | 24/24 |
| 0 | One-pass circular SQP | 0.1797 | 0.176 | 0.2136 | 56.54 | 2 | 24/24 | 0/24 |
| 0 | One-pass octagon QP | 1.155 | 1.149 | 1.281 | 209 | 2 | 24/24 | 0/24 |
| 0.5 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 0.5 | One-pass circular SQP | 0.2202 | 0.1828 | 0.4018 | 74.5 | 2 | 24/24 | 0/24 |
| 0.5 | One-pass octagon QP | 1.045 | 1.044 | 1.19 | 207.9 | 2 | 24/24 | 0/24 |
| 1 | Outer-loop circular | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| 1 | One-pass circular SQP | 0.1989 | 0.1803 | 0.4218 | 74 | 2 | 24/24 | 0/24 |
| 1 | One-pass octagon QP | 1.057 | 1.047 | 1.189 | 215.2 | 2 | 24/24 | 0/24 |
| 2 | Outer-loop circular | 27.63 | 2.475 | 500.1 | 1143 | 34.88 | 24/24 | 24/24 |
| 2 | One-pass circular SQP | 0.2568 | 0.1939 | 0.8958 | 94.71 | 2 | 24/24 | 0/24 |
| 2 | One-pass octagon QP | 1.156 | 1.061 | 3.27 | 248 | 2 | 24/24 | 0/24 |

For the reference, model solve success is outer convergence plus nonlinear feasibility. For one-pass methods it is native inner convergence plus affine feasibility. The last column checks the selected mode after nonlinear forward evaluation; for the octagon it checks its own polygon law, not exact circular Coulomb friction.

## Earlier conditional-search results (context, not the matched exhaustive control)

The older current-method runs stopped at the first acceptable mode. Their timings must not be confused with exhaustive reference timings. Zero-noise used normalized FBG gate 0.01; the noisy runs used 1.24. Different selection policies can change posterior history and accuracy.

| Noise mm | Motion | Old conditional tip N | Old conditional contact N | Old conditional mean s | One-pass circular mean s |
|---|---|---|---|---|---|
| 0 | insertion | 0.007676 | 0.009175 | 0.2837 | 0.7834 |
| 0 | rotation | 0.09756 | 0.3451 | 4.916 | 0.1797 |
| 2 | insertion | 4.495 | 5.642 | 1.906 | 0.664 |
| 2 | rotation | 6.162 | 9.495 | 26.86 | 0.2568 |

The older noiseless insertion controller was already fast (0.284 s/frame), and is faster than exhaustive one-pass circular SQP (0.783 s/frame). Therefore do not describe single-pass exhaustive search as universally faster than the earlier controller. The primary tables compare exhaustive policies; the older conditional table provides the requested historical context.

## Physical consistency and approximation error

Low force error does not certify the original nonlinear mechanics. The one-pass models enforce affine terminal moment and contact geometry exactly to solver tolerance, but their nonlinear forward evaluations can violate these constraints. These diagnostics do not terminate or alter the step. Positive separation gaps are excluded from the active-gap column.

| Motion | Noise mm | Method | Max absolute active gap mm | Max penetration mm | Max terminal moment N mm | Original circular-law feasible* |
|---|---|---|---|---|---|---|
| insertion | 0 | Outer-loop circular | 5.278e-13 | 1.532e-05 | 1.327e-06 | 41/41 |
| rotation | 0 | Outer-loop circular | 7.437e-13 | 2.4e-05 | 2.692e-10 | 24/24 |
| insertion | 2 | Outer-loop circular | 1.693e-07 | 0.0003242 | 3.376e-05 | 41/41 |
| rotation | 2 | Outer-loop circular | 1.094e-10 | 0.0002822 | 3.366e-08 | 24/24 |
| insertion | 0 | One-pass circular SQP | 0.1107 | 0.1117 | 29.25 | 5/41 |
| rotation | 0 | One-pass circular SQP | 0.0003401 | 0.0003401 | 0.195 | 0/24 |
| insertion | 0.5 | One-pass circular SQP | 0.1129 | 0.1138 | 29.44 | 0/41 |
| rotation | 0.5 | One-pass circular SQP | 0.004994 | 0.0006278 | 4.204 | 0/24 |
| insertion | 1 | One-pass circular SQP | 0.1071 | 0.1079 | 27.7 | 0/41 |
| rotation | 1 | One-pass circular SQP | 0.004678 | 0.003018 | 7.426 | 0/24 |
| insertion | 2 | One-pass circular SQP | 0.1364 | 0.1171 | 54.75 | 0/41 |
| rotation | 2 | One-pass circular SQP | 0.02102 | 0.02111 | 22.37 | 0/24 |
| insertion | 0 | One-pass octagon QP | 0.1107 | 0.1117 | 29.25 | 5/41 |
| rotation | 0 | One-pass octagon QP | 0.0003188 | 0.0003188 | 0.1779 | 0/24 |
| insertion | 0.5 | One-pass octagon QP | 0.1129 | 0.1139 | 29.49 | 0/41 |
| rotation | 0.5 | One-pass octagon QP | 0.004574 | 0.0007769 | 4.175 | 0/24 |
| insertion | 1 | One-pass octagon QP | 0.1072 | 0.1081 | 27.87 | 0/41 |
| rotation | 1 | One-pass octagon QP | 0.004731 | 0.003677 | 7.27 | 0/24 |
| insertion | 2 | One-pass octagon QP | 0.1377 | 0.1174 | 58.24 | 0/41 |
| rotation | 2 | One-pass octagon QP | 0.02101 | 0.02111 | 16.68 | 0/24 |

*For one-pass methods, the circular diagnostic uses the original disk-projection contact residual together with terminal moment and nonpenetration; the outer reference uses its exact mode residual. Their residual parameterizations differ, so magnitudes should not be equated. All use the 1e-6 check. `frames.csv` additionally reports exact sliding force-law errors on the affine and nonlinear shapes, friction work, tangency, and warning flags.

## Iteration cap and checks

| Method | All branch solves | Selected frames | Max selected iterations | Selected at cap | All branches at cap |
|---|---|---|---|---|---|
| One-pass circular SQP | 748 | 260 | 98 | 0 | 81 |
| One-pass octagon QP | 4408 | 260 | 28 | 0 | 89 |

The cap limits each branch, not the total iterations of a frame. A convex QP branch has a global optimum when successfully solved; however, exhaustive enumeration is a certificate for the entire affine polygon problem only if every potentially better branch is solved or certified infeasible. Truncating an alternative branch prevents an unconditional global claim. Circular branches remain nonlinear because of the full friction law.

Validation includes 3242 independent checks of polygon maximum dissipation against all eight force vertices and circular-cone containment, plus comparison of computed errors against truth, shared geometry, complete frame counts, and two-IVP accounting. Numerical output hashes are unchanged by the descriptive `exactCircularFriction=false` label correction documented in `metadata_label_correction.json`. No numerical solution was changed by that correction.

## Interpretation and recommendation

1. A single rod linearization is a useful fast approximation for these trajectories. Noiseless circular one-pass force errors are about 0.010/0.047 N (tip/contact) for insertion and 0.046/0.143 N for rotation. Its dominant noiseless insertion issue occurs at the reversal: forward shape and equilibrium residuals increase even when force error remains small. It is not a drop-in way to meet the current strict nonlinear physical tolerances.
2. The octagon is an acceptable coarse approximation for force accuracy in these particular tests, especially once sensor noise dominates. Its noiseless rotation error is larger, and polygon sliding is not the original circular law. An inscribed octagon can underestimate radial capacity by 7.612%, with up to 22.5-degree sliding-force direction discrepancy. Accuracy here does not establish adequacy for other wall orientations or contact configurations.
3. For this single-contact implementation, prefer one-pass full-cone SQP over exhaustive octagonal QPs if the goal is speed: the three nonlinear branches are cheaper in measured total time than eighteen QPs, while retaining the circular friction law. QP structure can become useful with a different mode-search strategy, but this experiment does not benchmark MIQP pruning or C3 consensus.
4. Keep nonlinear diagnostics. If strict mechanical consistency is required, retain correction/relinearization or design a separately approved adaptive correction rule. The tests here deliberately apply no such correction and never silently discard failed diagnostics.
5. Noise-induced tip/contact coupling remains. Neither polygonization nor a faster optimizer resolves force observability. One seed and one pair of trajectories cannot establish a statistical ranking when error differences are small. Some reference-mode failures also affect aggregate errors; a lower one-pass error is not a proof that affine mechanics is more accurate.

## Reproduction and artifacts

The final report intentionally leaves the unmeasured full-trajectory outer-loop controls at 0.5/1 mm unavailable; it does not substitute estimates for them. The figure uses isolated baseline markers at 0/2 mm.

From the package root in MATLAB: `setup_tsfs; addpath(fullfile(pwd,'experiments','inclined_wall')); run_single_pass_comparison('FRESH_STAMP',{'direct','polygon'})`. For the optional zero-noise exhaustive reference, call `run_single_pass_comparison('FRESH_STAMP',{'current'},0)`. Existing 2 mm exhaustive references are read automatically. Then run `analyze_single_pass_comparison.py FRESH_STAMP` and `publish_single_pass_report.py FRESH_STAMP`. The metadata correction helper is only needed for this initial batch, whose descriptive flag inherited the old full-cone default; the runner now sets it correctly.

Published output: `results/published/singlepass_comparison_20261012_unbounded/`. Detailed tables: summary.csv, frames.csv, branches.csv, polygon_law_checks.csv. Plot: comparison.png. Raw trajectories: `results/runs/singlepass_*_20261012_unbounded/comparison.mat`. Main log: `singlepass_comparison_20261012_unbounded.log`. Formulation: [SINGLE_PASS_POLYGON_QP.md](../formulation/SINGLE_PASS_POLYGON_QP.md).
