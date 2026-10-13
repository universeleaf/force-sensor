# 2026-10-10 mentor to collaborator: conditional mode search

Author: Codex for mentor. Intended recipient: collaborator; local handoff only, not sent. Branch codex/two-stage-force-sensing, base b376013. Uncommitted; no push.

## Approved change

The user requested previous-mode-first search and stopping at the first converged mode with adequate FBG fit. They explicitly selected **raw FBG RMS <= 1e-6 /mm**, equivalent to whitened RMS <= 0.01 with the existing sensor sigma 1e-4 /mm. Constraint tolerance remains 1e-6. The previous normalized diagnostic threshold 4 is not the selected acceptance threshold.

Only mode scheduling and acceptance changed. Full nonlinear rod, exact circular friction, geometry, fmincon subsolver, derivatives, and initialization are preserved. `tsfs.mode_solve` retains exhaustive search by default; the conditional wrapper sets the new option. If no mode meets the gate, the original fallback is returned, flagged `modeAccepted=false` and `no-acceptable-contact-mode`, and the trajectory continues.

## Evidence

Both full 41-frame trajectories completed. Mu=0.3: all 41 accepted; total time 232.948 -> 16.717 s (13.94x reduction), mean 5.682 -> 0.408 s, worst 71.007 -> 1.358 s. Mean iterations 44.12 -> 20.27; IVPs 204.56 -> 21.95. All saved tip/contact force components and shape RMS values are identical to exhaustive results. Tip/contact RMSE 0.004407/0.003746 N. Only frames 4 and 39 need alternatives; sticking is tested once at frame 4, converges, and is rejected by FBG fit. No IVP failures. Existing three small dense-mesh penetration warnings remain.

Mu=0: all 41 local solves converge, but only eight frames pass the FBG gate. Frames 5-37 are flagged fallback solutions, not successful fit acceptance. Time is essentially unchanged (0.463 -> 0.494 s per frame, 34.05 -> 33.51 IVPs); estimates remain identical and biased. The missing-friction model cannot fit the measurements to the chosen accuracy merely by changing modes.

A preliminary broader-threshold run began before the user's reply and is preserved separately: it accepted separation throughout, with tip/contact RMSE 8.402/10.411 N. It is clearly distinguished from the approved threshold run.

## Files and reproduction

[Full report](../docs/reports/INCLINED_WALL_CONDITIONAL_MODES_RESULTS.md); [updated formulation](../docs/formulation/INTEGER_MODE_MECHANICS.md).

Core: matlab/+tsfs/mode_solve.m. Common runner: experiments/inclined_wall/run_inclined_modes.m. New runner and analysis: run_inclined_modes_conditional.m and analyze_inclined_modes_conditional.py in that experiment directory. mode_branch.m is unchanged by source hash.

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
run_inclined_modes_conditional(Inf,'inclined_wall_modes_conditional_NEW_RUN',0.01);
```

Approved outputs: results/published/inclined_wall_modes_conditional_1e6_20261010/. Raw states and traces use the same tag under results/runs. Analysis validates 82 rows, 120 attempts, branch costs, previous-mode-first order, immediate exit at acceptance, exhaustive fallback, and equality of saved estimates to the exhaustive run.

## Remaining work / ownership

Mentor/collaborator: review the threshold and policy before adoption beyond this noiseless trajectory. Genuine sticking, multi-contact, axial rotation, and noise remain untested here. The absolute FBG gate is much tighter than the nominal observation sigma and needs reconsideration for noisy experiments. No global objective guarantee is implied. No new geometry repair, step rejection, physics assumptions, commit, or push was introduced.

## Video follow-up (2026-10-10)

User requested a video. Added publish_inclined_conditional.py and an optional conditional-status layout to our existing renderer; previous rendering behavior remains the default. The complete 41-step mu=0.3/mu=0 replay is at results/published/inclined_wall_modes_conditional_1e6_20261010/video/inclined_wall_comparison.mp4. It displays rejected-fit fallback states explicitly, uses saved estimates, and does not rerun or modify mechanics. Output: 1920x1080, 12 fps, 328 encoded frames / 27.33 seconds. Preview frames and video_metadata.json accompany it.
