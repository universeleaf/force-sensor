# Single-pass full-cone estimator with fixed-load BVP correction

Date: 2026-10-12. Branch codex/two-stage-force-sensing, base b376013. Eight complete trajectories / 260 new solves; no commit or push. User explicitly authorized this correction and exhaustive search. All outputs are separate from prior trials.

The existing full-cone method uses fmincon SQP, not a pure QP: circular sliding retains a nonlinear direction variable. Each frame exhausts separation/sticking/sliding on one affine rod model (100 SQP iterations per branch), selects the lowest affine-feasible objective, then calls MATLAB bvp4c once with all tip/contact forces fixed. No contact-mode reselection or force optimization occurs after that selection. Only base moment and nonlinear shape change. The corrected state feeds the next frame, so full-trajectory force differences reflect history feedback even though every individual correction leaves forces exactly unchanged.

Configuration and measurements match the prior single-pass test: insertion 80->40->80 mm distance, 41 frames; rotation 0->45->0 degrees by 3.75 degrees at maximum insertion, 24 new solves excluding restored seed. Rod 200 mm, 80 straight +120 precurved 90 degrees; true tip [1,0,0] N, mu=0.3. Gaussian Bishop noise targets 0/.5/1/2 mm use identical prior packets; no environment or mu noise. Zero injected noise retains covariance std 1e-4/mm. Stage 1 geometry matches the prior trajectories.

BVP unknowns are scaled position, nine rotation entries, moment and (for Bishop sensing) material-to-Bishop roll. Base position/orientation/roll and zero terminal moment are enforced, with continuity at intrinsic-curvature/stiffness/contact interfaces. Interior contact arclengths and loads stay fixed; wall gap/tangency and friction slip constraints are not part of this BVP. The initial guess is the optimized nonlinear IVP shape and base moment. bvp4c uses RelTol=1e-7, AbsTol=1e-9 on scaled states, NMax=4000, an initial 5 mm mesh, and an analytic ODE Jacobian. The existing SQP/diagnostic tolerances are unchanged.

After the BVP call an independent augmented IVP with the corrected base moment recomputes the full estimator diagnostics. Reported state is this IVP representation of the BVP solution. Successful frames thus use three IVPs plus the BVP collocation solve; BVP RHS evaluations are reported separately and are not counted as IVPs. One BVP call can contain multiple internal Newton/mesh iterations. Warnings/failures remain recorded, and contact diagnostics do not stop or change the step.

## Force, shape and timing comparison

All errors are trajectory RMS vector norms; shape RMS includes all sampled centerline nodes. Old columns refer to the preserved no-correction single-pass full-cone trajectories, not outer-loop controls.

| Motion | Noise mm | Old tip N | BVP tip N | Old contact N | BVP contact N | Old shape mm | BVP shape mm | Old mean s | BVP mean s |
|---|---|---|---|---|---|---|---|---|---|
| insertion | 0 | 0.01013 | 0.01044 | 0.04729 | 0.04788 | 0.06823 | 0.12 | 0.7834 | 0.7993 |
| rotation | 0 | 0.04595 | 0.4235 | 0.1434 | 1.97 | 0.01344 | 0.04378 | 0.1797 | 0.2469 |
| insertion | 0.5 | 0.7506 | 0.7502 | 0.8245 | 0.8243 | 0.1387 | 0.236 | 0.7759 | 0.7789 |
| rotation | 0.5 | 1.024 | 0.9405 | 3.043 | 2.759 | 0.1278 | 0.1379 | 0.2202 | 0.2634 |
| insertion | 1 | 1.528 | 1.529 | 1.723 | 1.723 | 0.2699 | 0.4736 | 0.7936 | 0.8394 |
| rotation | 1 | 1.815 | 2.056 | 4.768 | 5.163 | 0.2793 | 0.6438 | 0.1989 | 0.2698 |
| insertion | 2 | 3.32 | 3.391 | 3.774 | 3.84 | 0.6696 | 1.039 | 0.664 | 0.759 |
| rotation | 2 | 3.521 | 3.755 | 7.764 | 8.038 | 0.6172 | 0.9475 | 0.2568 | 0.3426 |

## Correction effects and physical consistency

| Motion | Noise mm | Max moment before N mm | Max moment after N mm | Max active gap before mm | Max active gap after mm | Max penetration after mm | BVP numerical success | Full mode success |
|---|---|---|---|---|---|---|---|---|
| insertion | 0 | 30.1 | 1.131e-06 | 0.1139 | 0.5696 | 0.005165 | 41/41 | 6/41 |
| rotation | 0 | 1.29 | 1.793e-07 | 0.001027 | 0.01751 | 0.01751 | 24/24 | 0/24 |
| insertion | 0.5 | 29.56 | 5.523e-07 | 0.1133 | 0.5662 | 0.334 | 41/41 | 7/41 |
| rotation | 0.5 | 3.114 | 2.53e-07 | 0.003067 | 0.04839 | 0.04864 | 24/24 | 0/24 |
| insertion | 1 | 27.16 | 7.235e-07 | 0.1049 | 0.8216 | 0.7524 | 41/41 | 8/41 |
| rotation | 1 | 15.54 | 3.492e-07 | 0.01871 | 0.3497 | 0.198 | 24/24 | 0/24 |
| insertion | 2 | 54.88 | 1.261e-06 | 0.1687 | 1.887 | 1.944 | 41/41 | 10/41 |
| rotation | 2 | 55.35 | 5.703e-07 | 0.04285 | 1.181 | 0.6113 | 24/24 | 0/24 |

Before/after columns compare the same frame with identical forces; the first table compares complete recurrent trajectories. Positive separation gaps are excluded from active-gap errors. BVP convergence certifies its fixed-load equilibrium problem, while full mode success also requires contact geometry and friction consistency at 1e-6.

## Computational cost and validation

| Motion | Noise mm | Mean BVP s | IVPs/frame | Mean BVP RHS calls | Total SQP iterations/frame | Modes/frame | Max mesh nodes | Max IVP-BVP shape RMS mm |
|---|---|---|---|---|---|---|---|---|
| insertion | 0 | 0.0476 | 3 | 2341 | 48.8 | 2.805 | 152 | 1.158e-07 |
| rotation | 0 | 0.04876 | 3 | 2960 | 56 | 3 | 185 | 6.981e-08 |
| insertion | 0.5 | 0.04012 | 3 | 2426 | 111.9 | 2.805 | 159 | 1.077e-07 |
| rotation | 0.5 | 0.0491 | 3 | 2985 | 70.88 | 3 | 190 | 7.226e-08 |
| insertion | 1 | 0.04005 | 3 | 2473 | 116.5 | 2.805 | 162 | 1.046e-07 |
| rotation | 1 | 0.05001 | 3 | 3039 | 81 | 3 | 196 | 6.888e-08 |
| insertion | 2 | 0.04355 | 3 | 2634 | 122.3 | 2.805 | 170 | 9.156e-08 |
| rotation | 2 | 0.05155 | 3 | 3106 | 107.6 | 3 | 200 | 5.455e-08 |

Branch solves: 748; maximum selected SQP iterations: 100; selected at the 100-iteration cap: 3; all alternatives at the cap: 88. A capped alternative prevents an unconditional global-optimality certificate.

Positive friction work appears on 52 corrected frames (before correction: 48 frames); this is explicitly flagged in the saved diagnostics. Fixed-load BVP convergence does not establish dissipative contact behavior.

Selected modes differ from the preserved uncorrected trajectories on 35/260 frames. This is a recurrent-history comparison, not a same-state force-refit experiment.

All 260 corrections preserve optimized forces exactly (max difference 0 N). Terminal moment decreased in 258/260 frames. FBG fit worsened in 148 frames and shape error worsened in 174 frames. These within-frame comparisons measure correction effects directly. Native affine success: 257/260; BVP numerical success: 260/260. Exceptions/warnings, exact circular sliding errors, and all frame residuals are in frames.csv.

## Interpretation

A fixed-load BVP addresses terminal equilibrium, but it cannot simultaneously adjust erroneous force estimates to preserve the measured shape or enforce wall contact. Physical contact constraints can worsen even when the BVP converges. This experiment measures that tradeoff without silently changing the force estimates or imposing an unapproved correction rule. A contact-consistent force correction would be a different formulation. No general speed or accuracy claim follows from one noise seed and these two trajectories.

In noiseless rotation, force RMS increases from 0.046/0.143 N to 0.424/1.970 N (tip/contact), while eight frames change from sliding to sticking. At 2 mm insertion, maximum corrected active gap reaches 1.887 mm. Consequently this fixed-load BVP is useful as an equilibrium audit, but the measured results do not support adopting it as a contact-consistent estimator correction. A future correction should allow forces to change while enforcing contact and FBG fit together; that was not implemented here.

## Reproduction and artifacts

From package root: setup_tsfs; addpath('experiments/inclined_wall'); run_single_pass_bvp('FRESH_STAMP'). Then python experiments/inclined_wall/analyze_single_pass_bvp.py FRESH_STAMP. Use a fresh stamp; raw checkpoints reject accidental overwrites. Historical 20261012_unbounded input packets and baseline results are read-only dependencies.

Combined output: results/published/singlepass_bvp_comparison_20261012_fixedload_v2/. Per-trajectory CSVs: results/published/singlepass_bvp_*_20261012_fixedload_v2/. Raw MATs: results/runs/singlepass_bvp_*_20261012_fixedload_v2/comparison.mat. Main log: singlepass_bvp_20261012_fixedload_v2.log. Preliminary smoke interface failures, successful smoke_v2 validation, and the interrupted first batch (20261012_fixedload) are preserved and excluded from scores. The first batch exposed floating-point interface evaluation outside a dense-ODE segment by roundoff; initial guesses now clamp to exact segment endpoints.
