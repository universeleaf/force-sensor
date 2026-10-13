# 2026-10-11 mentor to collaborator: calibrated Gaussian FBG noise

Author: Codex for mentor. Local handoff, not sent. Branch codex/two-stage-force-sensing; base b376013. No commit or push.

User authorized two Gaussian Bishop noise levels targeting 2 mm and 5 mm integrated tip RMS, both insertion and rotation, normalized FBG acceptance 1.24, otherwise unchanged successive-linearized rod and full circular friction. Implemented reproducible calibration, noisy input preparation, full trajectory runs for mu=0.3 and mu=0, and audited timing/physics/accuracy reports.

Calibration sigma: 3.7072566884243936e-4 and 9.245848421568634e-4 per mm. Validation uses independent draws, while inverse tests use one paired trajectory realization. Both stages receive the calibrated covariance. The common experiment runner now honors cfg.sensorStd (default unchanged) instead of hardcoding 1e-8 I. Rotation restores its corresponding noisy maximum-insertion estimate. No friction uncertainty or boundary softening was added.

All 264 saved states and 3140 outer iterations audited. Selected states are all branch-feasible, but four hit the outer limit. For mu=0.3: 2 mm insertion 39/41 converged, mean 1.906 s, tip RMSE 4.495 N; rotation 24/24 converged, mean 26.857 s, tip RMSE 6.162 N. At 5 mm: insertion 41/41, mean 2.188 s, tip RMSE 8.068 N; rotation 23/24, mean 15.366 s, tip RMSE 16.598 N. FBG acceptance is not force correctness: correct mode plus activation is 17/41, 13/24, 7/41, 3/24 respectively. One rejected sliding branch takes 430.47 s; all losing branches remain included.

The 2 mm geometry candidate-distance threshold misses contact in some 5 mm noise states. This was preserved as instructed. Core solver source hashes are unchanged, no previous simulation was overwritten, and all diagnostics remain reporting-only.

See [full report](../docs/reports/NOISY_LINEARIZED_ROD_RESULTS.md). Combined outputs: results/published/noisy_linearized_20261011/. Raw noise/input files and log: results/runs/noisy_linearized_20261011/. New entry points: calibrate_noisy_modes.py, run_noisy_linearized.m, analyze_noisy_linearized.py under experiments/inclined_wall/.

Next owner: mentor/collaborator should review the distinction between numerical feasibility, residual acceptance, and identifiability before selecting a new mode-search or uncertainty treatment. This run does not establish a statistically reliable force-error rate across noise realizations. Further solver, history, physics or acceptance changes require a separate agreed plan.
