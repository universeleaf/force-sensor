# Noisy mode-search and iteration-budget audit

Date: 2026-10-11. Author: Codex for mentor. Work is isolated in the collaboration package; no commit or push.

## Measured outcome

All four full recurrent exhaustive trajectories completed with MATLAB exit code 0. Exhaustive mode search reduced both tip- and contact-force RMSE in every case, but did not make the noisy estimator reliable. Force error reductions range from 16.4% to 25.8% for tip force and 7.3% to 32.8% for contact force. Geometry misses, noise-sensitive force decomposition, history mismatch, and failed continuous branch solves remain.

Each entry below is **conditional first-acceptable -> exhaustive lowest-objective feasible**. Force RMSE is sqrt(mean(||F_est-F_true||^2)), not per-coordinate RMS. All failed/unaccepted returned states remain included.

| Noise | Motion | Tip-force RMSE [N] | Contact-force RMSE [N] | Mean step time [s] | Converged + feasible | Fit accepted |
|---|---|---:|---:|---:|---:|---:|
| 2 mm | insertion | 4.495 -> 3.335 | 5.642 -> 3.793 | 1.91 -> 28.93 | 39/41 -> 36/41 | 39/41 -> 36/41 |
| 2 mm | rotation | 6.162 -> 4.985 | 9.495 -> 8.800 | 26.86 -> 27.63 | 24/24 -> 24/24 | 20/24 -> 22/24 |
| 5 mm | insertion | 8.068 -> 6.743 | 9.762 -> 7.735 | 2.19 -> 36.45 | 41/41 -> 36/41 | 41/41 -> 36/41 |
| 5 mm | rotation | 16.598 -> 13.283 | 20.531 -> 17.312 | 15.37 -> 30.41 | 23/24 -> 22/24 | 13/24 -> 16/24 |

Exhaustive search improves force RMS while selecting more flagged nonconverged candidates: 118/130 selected solves converge versus 127/130 for conditional search. This follows the preserved selection behavior described below. Every selected state in both policies satisfies the implemented 1e-6 nonlinear feasibility tolerance. Feasibility, fit acceptance, correct mode, and convergence are separate tests.

| Noise / motion | Correct mode + activation: conditional -> exhaustive | Shape RMS [mm] | Shape tip RMS [mm] | Exhaustive median / max step [s] | Exhaustive mean IVPs | Exhaustive mean inner / outer iterations |
|---|---:|---:|---:|---:|---:|---:|
| 2 mm / insertion | 17/41 -> 34/41 | 0.681 -> 0.620 | 1.390 -> 1.307 | 17.182 / 149.213 | 61.71 | 590.73 / 60.71 |
| 2 mm / rotation | 13/24 -> 14/24 | 2.102 -> 2.102 | 3.244 -> 3.356 | 2.475 / 500.125 | 34.88 | 1142.62 / 33.88 |
| 5 mm / insertion | 7/41 -> 22/41 | 2.016 -> 1.953 | 3.987 -> 3.912 | 18.046 / 378.841 | 51.44 | 836.29 / 50.44 |
| 5 mm / rotation | 3/24 -> 9/24 | 5.073 -> 5.687 | 6.950 -> 7.831 | 11.187 / 137.699 | 61.62 | 2705.04 / 60.62 |

At 5 mm rotation, shape RMS worsens from 5.073 to 5.687 mm despite lower force RMSE; exhaustive search is not a uniform improvement across all outputs. The experiment uses one paired noise realization, not a statistical multi-seed guarantee. Times include both stages, diagnostics, and all losing branches; they exclude calibration, forward truth generation, input preparation and serialization. Conditional and exhaustive timings are separate sequential runs with their respective recurrent histories.

Selected exhaustive outer-limit frames: 2 mm insertion 9, 12, 33, 35, 36; 5 mm insertion 19, 20, 25, 32, 38; 5 mm rotation 7 and 20. No selected 2 mm rotation state fails convergence. All twelve failures are retained in the tables.

[2 mm trajectories](../../results/published/noisy_mode_search_20261011/comparison_2mm.png), [5 mm trajectories](../../results/published/noisy_mode_search_20261011/comparison_5mm.png), [cycling versus convergence](../../results/published/noisy_mode_search_20261011/cycle_example.png).

## Experiment

The user requested explicit evidence of whether exhaustive mode search reduces the large force RMS errors at both calibrated noise levels, plus an examination of long versus converged steps and a suggested iteration budget. The new experiment replays exactly the previous 2 mm and 5 mm noisy Bishop packets, for insertion and rotation, with inverse and true friction coefficient 0.3. It changes only conditional versus exhaustive mode selection. The previous mu=0 ablation is preserved and not rerun for this mode-selection question.

The curvature standard deviations remain 3.7072566884243936e-4 and 9.245848421568634e-4 per mm. Both stages receive the same calibrated covariance. Normalized FBG RMS gate remains 1.24. Environmental inputs, Stage 1, sensor-only objective, full circular friction, successive rod linearization, nonlinear correction, tolerances, initializations, and iteration budgets are unchanged. No force prior, noise draw, geometry threshold, or physics change was added.

Each trajectory propagates its own selected estimates. Each rotation restores its corresponding maximum-insertion estimate, including any error in it. There are 41 insertion solves and 24 new rotation solves per level, with one restored rotation state excluded from solve aggregates. This is a recurrent policy comparison, not a single-step comparison on a shared posterior history.

The exhaustive policy enumerates separation, sticking and sliding for every Stage 1 candidate, starting with the previous mode. Every continuous branch is still a local nonlinear solve. This is not a globally certified mixed-integer solution. Missing Stage 1 contact candidates cannot be recovered by enumerating modes.

## Existing selection behavior

Conditional search stops at the first branch satisfying convergence, implemented nonlinear feasibility <=1e-6, and normalized FBG RMS <=1.24. If none passes, it returns the lowest-objective feasible candidate; when no candidate is feasible, it returns the least-violating one.

Exhaustive search uses the same fallback ranking after testing every assignment: the lowest-objective feasible candidate is eligible even if it did not converge. Therefore a flagged iteration-limit candidate can displace a converged higher-cost candidate. This behavior was retained to isolate the requested comparison and is reported explicitly. Diagnosed failures remain in accuracy and timing aggregates and continue into history.

With 48 observations and no force priors, J=0.5*||r||^2=24*rho^2, where rho is normalized FBG RMS. Thus the exhaustive ranking is exactly the residual ranking among eligible candidates. The constant 1e-4 multiplier used internally by SQP does not change that ordering.

## Current limits and tolerances

| Setting | Current value | Scope |
|---|---:|---|
| Rod linearization iterations | 40 | Per mode |
| SQP iterations | 300 | Per inner subproblem, reset every outer iteration |
| Objective evaluations | 5,000 | Per inner subproblem |
| SQP optimality tolerance | 1e-8 | Scaled objective |
| SQP step tolerance | 1e-12 | Inner coordinates |
| Nonlinear feasibility tolerance | 1e-6 | Implemented scaled mode constraints |
| Outer step tolerance | 1e-4 | Infinity norm of scaled Cartesian state increment |
| Initial / maximum / minimum trust radius | 4 / 64 / 1e-6 | Outer scaled coordinates |
| IVP relative / absolute tolerance | 2e-8 / 2e-10 | Nonlinear validation and sensitivity integration |
| FBG RMS gate | 1.24 | Whitened measurement residual, not an iteration tolerance |

The 300/5000 inner limits are currently hardcoded in mode_branch.m; similarly named configuration fields mirror them but do not override these two optimoptions values. There is no wall-clock cutoff. A mode can consume 12,000 inner iterations across 40 successive subproblems; three modes can consume 36,000. Reported per-frame iteration/IVP totals include losing branches. Positive inner exit, true feasibility and a sufficiently small outer step define reported convergence; this is not a global KKT certificate.

## Why some steps are long

In the original conditional mu=0.3 tests, rejected rod updates consumed 998.29 of 1163.16 recorded outer-trace seconds (85.8%). All 21 inner solves hitting 300 iterations were rejected rod updates. Nevertheless accepted updates sometimes needed 270 iterations, and some eventually successful branches contain lengthy rejected sequences.

The most expensive old branch was 2 mm rotation frame 3, sliding: 430.47 s, 40 outer updates, 6047 SQP iterations, 41 charged IVPs. It cycles between trust radii without updating the rod model. Repeating the same model, history, mode, initialization and radius produces the same subproblem. The first repeated state is detectable before outer iteration 6, after approximately 63.89 s. Its final violation is about 8.47e-4, so it is not merely waiting to pass a tight stationarity test.

By contrast, 5 mm rotation frame 15 sliding converges in 9 outer updates, 868 inner iterations and 69.56 s. A cap on outer iterations alone cannot remove all slow successful steps. Another selected successful branch needs 34 outer updates. Five consecutive rejected updates are not a safe rejection criterion: eight original branches with long rejection streaks eventually converge.

## Candidate-level examples

At 2 mm insertion frame 13 in the exhaustive trajectory, both separation and sliding converge and are feasible. Sliding has normalized RMS 1.058471 and tip-force error 3.704 N; separation has RMS 1.053293 and tip error 6.942 N. Exhaustive residual minimization selects separation. The difference in total squared normalized residual is only 0.524901. Exhaustion therefore does not guarantee selecting the more accurate force explanation.

At frame 24, the reference and selected mode are sliding, the solver converges, and normalized RMS is 1.075540. Tip-force error is still 8.745 N, while centerline error is only 0.326 mm RMS. Estimated tip Fx is 9.736 N instead of 1 N, while contact Fx falls from the true 12.608 N to 4.500 N: a concrete tip/contact load tradeoff. Predicted noiseless FBG discrepancy is only 0.4093 times the curvature noise standard deviation. Large force error can persist on the correct mode with a nearly correct shape.

These examples establish a combination of premature conditional acceptance, noise-sensitive force decomposition, geometry/history error, and imperfect continuous solves. They do not isolate a pure sensor-noise contribution. Wrong-mode labels alone cannot establish causality, and iteration count cannot identify physical mode correctness.

## Proposed budget policy (not implemented)

Retain the 40-outer safety limit and prioritize cycle detection. A hard 20-outer cap loses useful branches: in the completed exhaustive 2 mm insertion trajectory it truncates 14 of 36 selected converged branches, even though all 36 selected successful branches together cost only 11.81 s. A 20-iteration soft screen with later resumption can be considered for scheduling, but is not recommended as an irreversible mode rejection rule. Preserve the best feasible candidate and an unresolved flag rather than declaring the physical mode impossible when a budget expires. If an objective-best mode is required, an unfinished mode cannot be rigorously pruned without a valid lower bound.

Detect a repeated model/history/mode/initialization plus trust radius before solving it again. Mark this as numerical cycling and return the best already evaluated candidate. This is a more targeted remedy than a fixed wall-clock cutoff or five-rejection rule. The observed replay test must verify identical candidates and preservation of the best feasible objective before claiming saved work.

Keep the inner limit at 300 for now. In the previous conditional tests, a hard cap of 200 interrupts three previously selected successful branches. A smaller inner screening budget with a resume mechanism is possible, but simply restarting after changing the limit is not equivalent to preserving the SQP state. No early-stop or rescue implementation is included in this audit.

## Full iteration-budget audit

The following counts are selected branches that reached convergence within the proposed hard outer cap on their original trace. These are retrospective prefixes; no new limited-budget trajectory was run.

| Outer cap | Original conditional selected successes retained | Exhaustive selected successes retained |
|---:|---:|---:|
| 10 | 115/127 | 84/118 |
| 20 | 121/127 | 96/118 |
| 30 | 125/127 | 115/118 |
| 35 | 127/127 | 118/118 |
| 40 | 127/127 | 118/118 |

A 35-outer cap retains all selected successes observed here, so it is a plausible experimental lower cap if one is required. It does not preserve every successful alternative: 5 mm insertion frame 37 has a correct sliding branch that converges only at outer iteration 40 (50.12 s), though a lower-cost separation candidate is selected. For preserving good modes rather than just the choices made in these particular histories, retaining 40 is safer. A 30-outer cap loses only 3/118 exhaustive selected successes but saves only 0.20 s on those successful branches; the useful saving must come from failed attempts.

Lowering the inner cap is also not automatically safe. In the exhaustive traces, 6 selected successful branches include an inner solve longer than 200 iterations; 4 include one longer than 250. Earlier termination changes the outer path, so these are affected paths, not a prediction that each capped rerun must fail. A reduced inner cap needs its own validation.

## Cycle audit and potential saving

Across conditional and exhaustive mu=0.3 runs, 264 saved states, 546 mode attempts and 9259 outer iterations were audited. All five core solver source hashes match the previous noisy benchmark.

| Policy | Attempts | Failed cycling branches | Cycling branches that later converge | Cycling branches selected | Returned states already available before first repeat | Recorded time after first repeat [s] |
|---|---:|---:|---:|---:|---:|---:|
| conditional | 186 | 24 | 0 | 0 | 24/24 | 787.48 |
| exhaustive | 360 | 89 | 0 | 0 | 89/89 | 3123.19 |

In every detected cycle, the repeated candidate is bit-for-bit identical and the eventual returned state was already present before the first repeated solve. No converged or selected branch was flagged. The exhaustive first-repeat iteration ranges from 4 to 29, with median 7. A simple five-rejection rule would instead interrupt 36 eventually converged exhaustive branches (maximum rejected streak 16). The correct detection key includes the unchanged rod model, history, mode, initialization and trust radius; equal radii across accepted model updates are not cycles.

The exhaustive run totals 4074.03 s. Its 3123.19 s of post-repeat work is 76.7% of that time. Prefix replay suggests the following savings while preserving every returned candidate; these are **estimates from saved timings, not measured reruns with a new guard**.

| Noise / motion | Measured exhaustive mean [s] | Prefix estimate with repeated subproblems skipped [s] |
|---|---:|---:|
| 2 mm / insertion | 28.934 | 5.434 |
| 2 mm / rotation | 27.632 | 8.486 |
| 5 mm / insertion | 36.454 | 7.653 |
| 5 mm / rotation | 30.414 | 8.774 |

The 2 mm exhaustive rotation frame 3 costs 500.12 s: sliding takes 483.39 s and remains infeasible at 8.48e-4, separation takes 0.40 s and is feasible with FBG RMS 1.3057, and sticking takes 16.22 s and remains infeasible. Separation is a flagged fit fallback, not a wrong mode accepted by the 1.24 gate. The sliding branch uses 6146 SQP iterations, 62,255 affine evaluations and 14,889 RHS evaluations across 41 IVPs; this supports targeting repeated optimization, but is not an exclusive CPU profile.

At 5 mm insertion frame 22, failed sticking takes 375.00 s while the selected sliding solution takes 3.55 s. At frame 30, failed sticking takes 211.67 s while selected sliding takes 0.51 s. Long total steps and long successful branches are different phenomena.

## How much residual error can mode selection explain?

The full recurrent comparison above is the primary answer. Additional offline choices among the exhaustive candidates use the *same exhaustive history* at each frame; they are not new recurrent simulations.

| Noise / motion | First acceptable on exhaustive history: tip/contact RMSE [N] | Exhaustive selected: tip/contact RMSE [N] | Truth-oracle among returned feasible candidates: minimum tip/contact RMSE [N] |
|---|---:|---:|---:|
| 2 mm / insertion | 3.634 / 4.471 | 3.335 / 3.793 | 2.411 / 2.843 |
| 2 mm / rotation | 5.196 / 8.864 | 4.985 / 8.800 | 4.861 / 8.579 |
| 5 mm / insertion | 7.671 / 9.474 | 6.743 / 7.735 | 5.788 / 7.010 |
| 5 mm / rotation | 14.251 / 17.761 | 13.283 / 17.312 | 12.630 / 16.440 |

The oracle uses truth only for retrospective scoring, with separate minimizing choices for tip and contact force. It is neither implementable sensing nor a lower bound over all possible continuous solutions. Even this oracle leaves 12.63 N tip RMSE at 5 mm rotation, so simply choosing a different already-returned mode cannot remove most of that error. Unfound continuous solutions remain possible. If only converged feasible candidates are eligible, offline tip/contact RMS at 5 mm insertion is 7.387/8.895 N and at 5 mm rotation 14.218/18.242 N, worse than the retained feasible-but-flagged selection; changing this eligibility rule is a distinct decision.

The next table partitions exhaustive tip-force squared error by observed outcome. Categories are descriptive, not a causal decomposition of sensor noise versus geometry/history versus solver error. A missing feasible correct-mode candidate is not proof that the correct mode is globally infeasible.

| Noise / motion | Outcome category | Frames | Share of tip-force squared error |
|---|---|---:|---:|
| 2 mm / insertion | Correct mode and converged | 29 | 42.7% |
| 2 mm / insertion | Selected candidate not converged | 5 | 18.5% |
| 2 mm / insertion | Wrong mode despite feasible correct-mode candidate | 4 | 34.1% |
| 2 mm / insertion | Wrong mode; no feasible correct-mode candidate returned | 3 | 4.7% |
| 2 mm / rotation | Correct mode and converged | 14 | 27.3% |
| 2 mm / rotation | Wrong mode despite feasible correct-mode candidate | 5 | 10.2% |
| 2 mm / rotation | Wrong mode; no feasible correct-mode candidate returned | 5 | 62.5% |
| 5 mm / insertion | Correct mode and converged | 17 | 11.8% |
| 5 mm / insertion | Selected candidate not converged | 5 | 16.2% |
| 5 mm / insertion | Stage 1 missed contact | 6 | 38.1% |
| 5 mm / insertion | Wrong mode despite feasible correct-mode candidate | 8 | 15.5% |
| 5 mm / insertion | Wrong mode; no feasible correct-mode candidate returned | 5 | 18.3% |
| 5 mm / rotation | Correct mode and converged | 8 | 16.0% |
| 5 mm / rotation | Selected candidate not converged | 2 | 1.8% |
| 5 mm / rotation | Stage 1 missed contact | 1 | 8.0% |
| 5 mm / rotation | Wrong mode despite feasible correct-mode candidate | 1 | 2.9% |
| 5 mm / rotation | Wrong mode; no feasible correct-mode candidate returned | 12 | 71.4% |

At 5 mm insertion, Stage 1 misses true contact at frames 4, 10, 21, 23, 28 and 36; at rotation frame 22 it also misses contact. Rotation frame 1 inherits the missed-contact insertion seed and is excluded from solve statistics. Enumeration cannot recover a contact absent from the candidate set. Conversely, correct-mode converged frames still have substantial force error, including the frame-24 load-transfer example above. The evidence supports a mixed cause, not noise alone or premature mode acceptance alone.

## Validation and handoff

MATLAB and the complete Python audit both exited successfully. The analysis checked all 264 saved states against the nonlinear reference, normalized residuals, branch accounting, exact exhaustive assignment counts, minimum-objective selection semantics, unchanged Stage 1 outputs and solver hashes. Figures were visually inspected. All prior results remain intact. Only our experiment wrapper, new exhaustive runner, analysis, reports and published diagnostic artifacts changed for this audit; core solver limits and physics did not. No commit or push.

## Reproduction and artifacts

From this package, using a fresh stamp:

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
run_noisy_exhaustive('NEW_STAMP');
```

```text
python experiments/inclined_wall/analyze_noisy_mode_search.py NEW_STAMP
```

The wrapper deliberately reuses the calibrated packets from noisy_linearized_20261011. Raw results use inclined_{insertion,rotation}_noisy{2,5}mm_exhaustive_NEW_STAMP. Aggregate outputs use results/published/noisy_mode_search_NEW_STAMP. The current run log is results/runs/noisy_exhaustive_20261011/benchmark.log.

The analyzer independently scores nonlinear states, checks normalization and unchanged core hashes, and exports every branch and outer iteration. Its budget tables are retrospective trace-prefix censoring, not actual runs with new limits. Its same-history first-acceptable and truth-informed candidate comparisons are offline one-step diagnostics on the exhaustive history; they are not separately propagated trajectories. Truth-informed selections are never used by the estimator.
