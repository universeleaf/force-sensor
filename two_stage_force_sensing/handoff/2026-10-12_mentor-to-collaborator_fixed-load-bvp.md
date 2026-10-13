# Completed handoff: fixed-load BVP correction

Date: 2026-10-12. Author/direction: Codex, mentor-side collaborator to collaborator (local only, not sent). Branch codex/two-stage-force-sensing, base b376013. No commit or push.

## Authorization and scope

User explicitly requested one nonlinear BVP call after exhaustive single-pass full-cone mode selection, using optimized shape/base moment as the initial guess and holding every load fixed. We implemented that exact scope, retaining Stage 1, full circular friction, 100 SQP iterations per branch, no trust bound, the four prior noise packets, and standard diagnostics. All eight insertion/rotation trajectories completed; 260 newly solved frames exclude restored rotation seeds.

## Implementation and reproducibility

New modular functions: matlab/+tsfs/fixed_load_bvp.m (multipoint bvp4c with analytic ODE Jacobian) and single_pass_bvp_solve.m (original exhaustive solver followed by the fixed-load correction and independent IVP audit). Separate runner and analyzer: experiments/inclined_wall/run_single_pass_bvp.m and analyze_single_pass_bvp.py. Original single_linearization_solve and all earlier outputs remain unchanged. README updated.

BVP enforces base position/rotation/Bishop roll, zero terminal moment, and state continuity at material/load interfaces. All tip/contact forces and contact arclengths stay fixed. Only base moment/shape change. RelTol=1e-7, AbsTol=1e-9 in scaled states, NMax=4000, initial 5 mm mesh. Contact gap, tangency, and friction slip are diagnostics, not BVP conditions. Independent IVP agrees with the BVP centerline to <=1.16e-7 mm RMS. Every correction preserves forces exactly. Each frame has three mechanics IVPs plus BVP collocation work; a single BVP call can contain multiple Newton/mesh iterations. Corrected state feeds next-frame history.

Run from package root: setup_tsfs; addpath('experiments/inclined_wall'); run_single_pass_bvp('FRESH_STAMP'). Then python experiments/inclined_wall/analyze_single_pass_bvp.py FRESH_STAMP. Historical identical input packets and no-correction benchmark at 20261012_unbounded are read-only dependencies. Use fresh stamps; checkpoints protect against overwriting.

## Results and failures

Final stamp: 20261012_fixedload_v2. All 260 BVP calls converged; maximum independently validated terminal moment is 1.261e-6 N mm. Native affine convergence: 257/260. Three selected branches reached iteration cap: rotation 1 mm frame 10; insertion 2 mm frame 35; rotation 2 mm frame 6. They remain marked unconverged. Total branch solves 748; 88 alternatives hit the cap, preventing a global-optimality certificate.

Noiseless insertion tip/contact RMS is 0.0104/0.0479 N, nearly unchanged, but shape RMS rises from 0.0682 to 0.120 mm. Noiseless rotation tip/contact RMS rises from 0.0460/0.1434 to 0.4235/1.9697 N; eight early frames select sticking instead of sliding. Complete recurrent trajectories differ because shape/slip history changes, despite within-frame force invariance. At 2 mm, corrected insertion errors are 3.391/3.840 N and rotation 3.755/8.038 N.

Full-mode success is 31/260 (6/7/8/10 insertion frames at noise 0/.5/1/2; zero rotation frames). Maximum active gap reaches 0.570 mm without noise and 1.887 mm at 2 mm insertion. Positive friction work is flagged on 52 rotation frames (17/13/13/9 by noise level); largest positive work 1.682 N mm. The direction/slip law is therefore not restored by correcting equilibrium. FBG fit worsens within 148 corrections and shape error within 174.

Mean total time: insertion 0.759-0.839 s/frame, rotation 0.247-0.343 s/frame. BVP itself adds about 0.040-0.052 s/frame. Tests confirm force invariance, shared Stage 1 geometry, complete frames, three-IVP counts, BVP/IVP agreement, terminal boundary residuals, and physical diagnostics. Plot visually inspected. No unrelated MATLAB session was touched; only the user's separate MATLAB2023a session remains running.

## Artifacts and interpretation

Full report: ../docs/reports/SINGLE_PASS_FIXED_LOAD_BVP_RESULTS.md. Equations/settings: ../docs/formulation/FIXED_LOAD_BVP_CORRECTION.md. Combined output: ../results/published/singlepass_bvp_comparison_20261012_fixedload_v2/ (summary.csv/json, frames.csv, branches.csv, comparison.png, analysis_console.txt). Separate raw MATs: ../results/runs/singlepass_bvp_*_20261012_fixedload_v2/comparison.mat. Main log: ../singlepass_bvp_20261012_fixedload_v2.log.

Smoke v1 history-interface failures, smoke v2 success, and first-batch roundoff-interface failures are preserved and excluded. The endpoint issue was fixed by clamping dense-ODE seed evaluations to exact segment endpoints within 1e-8 mm roundoff; no scientific tolerance or load changed. Final run has no BVP warning/exception.

Recommendation for mentor review: fixed-load BVP correctly repairs equilibrium but is not a suitable final contact-consistent estimator correction. Forces should be allowed to change in a jointly constrained correction if strict contact consistency is required. No such new formulation was implemented. Preserve all earlier local changes; no collaborator message or commit/push was made.
