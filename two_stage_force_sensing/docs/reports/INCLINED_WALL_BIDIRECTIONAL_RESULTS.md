# Inclined-wall insertion and withdrawal: measured results

Date: 2026-10-08. Branch: codex/two-stage-force-sensing; base commit b376013. Local, uncommitted work. This is a separate experiment; prior simulations and original collaborator algorithms are unchanged.

## Outcome and limits

The requested geometry produces the intended proximal contact migration. The independent forward reference completed all 41 frames and passed its physical checks. The friction-aware estimator returned all 41 frames, including 6 budget exits and 1 infeasible exit. The unchanged mu=0 estimator returned the first 3, non-contact frames, then raised tsfs:RHSBudget at frame 4, first actual contact. Frames 5-41 were not attempted. A complete force-accuracy comparison between the two methods is therefore not yet available.

[Video](../../results/published/inclined_wall_bidirectional_20261008/unreduced_attempt/inclined_wall_comparison.mp4) shows every reference frame and every available estimate. Missing mu=0 estimates are explicitly marked, never held or replaced. [Metrics figure](../../results/published/inclined_wall_bidirectional_20261008/unreduced_attempt/comparison_metrics.png), [all returned estimates](../../results/published/inclined_wall_bidirectional_20261008/unreduced_attempt/estimates.csv), [truth physics](../../results/published/inclined_wall_bidirectional_20261008/unreduced_attempt/truth_frames.csv), and [summary](../../results/published/inclined_wall_bidirectional_20261008/unreduced_attempt/summary.csv).

## Configuration

- Length 200 mm: proximal straight 80 mm; distal curved 120 mm; intrinsic bend 90 degrees, radius 76.394372684 mm.
- Fixed wall z-x=113.13708499 mm, contact normal [1,0,-1]/sqrt(2). The user confirmed that 40 mm means perpendicular base-to-plane distance.
- Base inserts along +Z then withdraws. Perpendicular distances 80 -> 40 -> 80 mm in 2 mm increments; 41 samples; maximum base Z travel 56.56854249 mm. The entire 200 mm exposed rod translates.
- True tip force [1,0,0] N; true mu=0.3. Both inverse models receive the same truth-generated observations. No injected FBG/environment noise. Ideal geometric Bishop w1,w2 at 24 FBG stations. The estimator still weights measurements using sigma=1e-4 /mm, covariance 1e-8 I. Environmental prior standard deviations are 0.03 mm for each position component and 0.001 for each normal component.
- Stage 1: existing lsqnonlin geometry MAP, max 30 iterations / 300 evaluations, function/step tolerances 1e-8, optimality 1e-6. Stage 2: existing fmincon SQP relaxed adapter; direct epsilon=1e-6; constraint tolerance 1e-6, optimality 1e-8, step tolerance 1e-12, 300 iterations and 5000 function-evaluation budget. Objective multiplier 1e-4, sensor-only objective (48 terms), no tip/contact force penalties. Existing variable/constraint scales unchanged. ODE tolerances relative 2e-8, absolute 2e-10. IVP RHS cap 100000 per integration.
- Mechanics collision spacing 0.5 mm; denser diagnosis 0.1 mm. Diagnostics remain reporting-only. All returned states, including failures, are retained. Existing initialization uses the previous converged mechanics result when matching criteria hold; friction history uses the previous returned mechanics result even if unconverged. No restart or history rejection was introduced.
- Reference exploits this fixture's planar symmetry; the inverse solver retains full 3D variables, including Y forces. Finite-radius friction torque, tip contact, and corner/edge contact remain outside the agreed model. The 0.02 s timestamps are quasistatic history bookkeeping, not a validated dynamic speed; video playback is illustrative.

## Reference behavior and physical checks

The free loaded rod first touches near perpendicular distance 74.7547 mm. Sampled contact begins at frame 4, distance 74 mm, s=134.849 mm. At frame 21, distance 40 mm, contact is s=100.214 mm: a proximal migration of 34.635 mm. Normal force reaches 23.1454 N; signed tangential force is -6.94362 N. At the first withdrawal sample, distance 42 mm, the normal force is 16.8524 N and tangent force +5.05572 N. The friction sign reverses consistently with slip. Samples at distances 76, 78, 80 mm are separated on both legs; 35 frames have actual contact.

Across the forward reference, maximum penetration is 6.07e-9 mm, complementarity product magnitude 1.40e-8 N mm, terminal stationarity residual 8.14e-9 N mm, and disk-projection residual 1.78e-15 N. All friction work is nonpositive. The forward solver enforces tangency directly; the separately logged tangent interpolation check is at most 3.22e-6. There are no sampled sticking frames; the 2 mm trajectory does not resolve any brief sticking transient at reversal.

## Friction-aware estimation

All failed frames contribute to these statistics. RMS force error is sqrt(mean(||F_est-F_true||^2)); shape RMS pools the 201 material stations and the stated frames. Elapsed time includes both stages and diagnosis, excludes saving/video and the independent reference simulation. IVPs are augmented state-and-sensitivity integrations, not individual ODE segments or RHS calls.

| Measure | All 41 frames | 35 true-contact frames |
|---|---:|---:|
| Tip-force RMS error [N] | 3.73488 | 4.04235 |
| Contact-force RMS error [N] | 4.87061 | 5.27159 |
| Shape RMS [mm] | 0.89034 | 0.96364 |
| Median tip-force error [N] | 0.002258 | 0.002525 |
| Median contact-force error [N] | 0.004494 | 0.005897 |
| Median per-frame shape RMS [mm] | 0.003565 | 0.008139 |
| Mean elapsed [s/frame] | 8.88892 | 10.31897 |
| Median elapsed [s/frame] | 0.83514 | 0.99380 |
| Mean SQP iterations | 85.073 | 98.086 |
| Mean augmented IVPs | 853.073 | 995.371 |
| Native positive exit | 34/41 | 28/35 |
| Relaxed-feasible return | 40/41 | 34/35 |

Mean Stage 1 time is 0.13424 s; mean Stage 2 time 8.74933 s. Maximum total per-frame time is 52.84890 s. Native positive exits include 19 optimality exits and 15 small-step exits; a positive exit is not a certificate of accurate force recovery. Six frames hit a budget and one was infeasible. Reporting flags occur in 9 frames; stricter exact-law flags occur in 35 frames and remain in the CSV. Relaxed feasibility is distinct from strict friction-law satisfaction or dense nonpenetration.

At closest approach, the tip-force error is 0.01944 N, contact-force error 0.01469 N, shape RMS 0.00981 mm, total time 1.306 s, 62 iterations, and 81 IVPs. This is a small-step exit, not the requested optimality tolerance: stationarity is 1.32e-5. Its relaxed constraint violation is 2.13e-14, but a dense penetration warning remains relative to the fitted plane; strict projection residual is 6.37e-4 N. No penetration is found against the true plane at the saved collision samples. This illustrates the distinction between the sampled fitted-geometry optimization constraints and denser diagnostics.

Worst failure: frame 15, insertion distance 52 mm, infeasible exit. Tip-force error 23.5467 N, contact-force error 30.7577 N, shape RMS 5.69768 mm; relaxed violation 1.0161e-3, stationarity 386.526, terminal moment 0.03536 N mm. This state develops spurious Y tip/contact forces -1.07891 / -0.49641 N despite planar truth. It is not omitted from plots, scoring, or playback.

Stage 1 is identical between models for the three completed paired frames; it does not depend on the assumed friction coefficient. Over the full mu=0.3 contact trajectory its maximum absolute arclength error is 0.04725 mm and normal-direction error 0.001345 degrees. The largest geometric-only shape RMS is 0.04796 mm. Small reconstruction/interpolation error remains despite noiseless samples. An inactive candidate at 76 mm has a larger 0.04263-degree normal perturbation; it must not be confused with an actual loaded-contact normal error.

These data show that failures occur even with small Stage 1 errors. They do not isolate whether each failure is caused by conditioning, history, active-set geometry, or numerical scaling. Moving contact proximally succeeded geometrically but did not make the current solver uniformly reliable. A causal claim that the new trajectory improves identifiability needs a matched sensitivity analysis, not just the contact-location change.

## Unchanged mu=0 failure and proposed next action

The original circular cone becomes tau' tau <= 0 for mu=0. Its feasible set is exactly tau=0, but its gradient vanishes there. This is a structural degeneracy. The observed exception arose during a trial IVP, not from an ordinary fmincon exit; the trial exceeded the existing RHS budget. The log alone does not prove this degeneracy caused that trial.

A mathematically equivalent exact mu=0 reduction would remove the two tangential force variables per contact and the slip-epigraph beta variables. Friction-force cone and friction-duality inequalities then become identities; the slip epigraph can always be satisfied and is irrelevant to mechanics. Retain lambda>=0, full-rod nonpenetration, lambda*g<=epsilon, equilibrium and the identical sensor objective. This removes the degenerate representation without adding a prior or changing frictionless physics. It changes the numerical formulation, however, and is pending explicit user approval under AGENTS.md. It has not been implemented or used in the published results.

The 3 completed mu=0 frames precede true contact and cannot support a loaded-contact comparison. Their timing/error summary must not be compared against the full 41-frame friction-aware aggregate. Frame 4 has no returned estimate or reliable final iteration count; the crash log records the available evidence. No subsequent state is invented.

## Reproduction and files

[Experiment instructions](../../experiments/inclined_wall/README.md). Sources are isolated under experiments/inclined_wall. The normal runner intentionally uses the unchanged solver and reproduces the failure; publisher option --allow-incomplete renders the recorded incomplete attempt with explicit missing-data labels. Raw results are under results/runs/inclined_wall_bidirectional_20261008/inclined_wall. Copies named comparison_unreduced_attempt.mat and estimates_unreduced_attempt.csv preserve the pre-decision experiment. The crash log is results/runs/inclined_comparison.log, also copied into the published folder. No commit or push was made.
