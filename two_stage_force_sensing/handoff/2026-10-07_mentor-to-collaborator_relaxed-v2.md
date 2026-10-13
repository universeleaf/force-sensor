# Revised relaxation benchmark handoff

Date: 2026-10-07. Author: Codex assisting the mentor. Direction: mentor to collaborator; local report, not sent. Branch: codex/two-stage-force-sensing. Base commit: b376013; this milestone remains uncommitted and unpushed.

## Approved changes and scope

Within two_stage_force_sensing only: disable Stage 2 tip/contact force priors, use FBG sigma=1e-4 /mm (covariance 1e-8 I) in both stages, set nonlinear constraint tolerance=1e-6 with corresponding reporting thresholds, and remove the duplicated contact-gap inequality. Preserve epsilon=1e-6, objective multiplier=1e-4, optimality=1e-8, step tolerance=1e-12, 300 iterations, function budget=5000 for fmincon, variable/row scaling, IVP settings and previous-state initialization/history. No noise was added. The default semismooth solver remains unchanged; these are the experimental relaxed adapters.

The optional configuration preserves v1 behavior when omitted. Geometry and mechanics remain separate stages, followed by reporting-only diagnostics. Original stricter physical warnings are retained in exactLawFlags. No failed output was rejected or replaced.

## Evidence

[Full results and all 118 frames](../docs/reports/RELAXED_ROTATION_V2_RESULTS.md). [Exact configuration and reproduction](../experiments/base_rotation/RELAXED_COMPARISON_V2.md). Published evidence folders: base_rotation_relaxed_v2_375deg_20261007, base_rotation_relaxed_v2_1deg_20261007, and base_rotation_relaxed_v2_comparison_20261007. Original runs remain intact.

| Solver / increment | Tip RMSE N | Contact RMSE N | Shape RMS mm | Mean seconds | Mean iterations | Mean augmented IVPs | Native positive / frames |
|---|---:|---:|---:|---:|---:|---:|---:|
| IPOPT / 3.75 deg | 1.71362 | 3.91507 | 3.17218 | 12.894 | 240.92 | 914.92 | 5/13 |
| fmincon / 3.75 deg | 0.085247 | 0.090831 | 0.000299 | 8.183 | 100.38 | 658.00 | 12/13 |
| IPOPT / 1 deg | 1.50340 | 3.28955 | 2.83185 | 13.099 | 218.61 | 923.20 | 20/46 |
| fmincon / 1 deg | 0.20334 | 0.17425 | 0.099274 | 3.268 | 57.13 | 254.83 | 46/46 |

All frames, including failures, contribute to these values. Coarse fmincon improves over v1; the other three accuracy aggregates worsen. Fine fmincon reports 34 small-step exits, and its 37-degree error reaches 1.336 N despite a positive exit. Coarse fmincon at 18.75 degrees exhausts its function-evaluation budget. IPOPT fails severely from 37.5/38 degrees onward, often reaching relaxed-feasible near-zero-contact-force states with poor FBG fits. The auxiliary slip bound beta becomes much larger than actual slip at those failures. This is evidence to investigate, not proof of a global/local alternative optimum.

Stage 1 maximum errors are below 0.00266 mm in arclength and 0.000395 degrees in normal direction. Sensor scaling reduced objective gradients and multipliers, but normal-product rows remain around 33-38 versus squared-slip rows around 1e-5 to 1e-4. At fixed state, the mechanical constraint Jacobian is exactly independent of sensor covariance. The squared cones still have vanishing derivatives near sticking/apex states. A larger sigma also makes the FBG mismatch diagnostic less sensitive in absolute curvature units.

## Validation and reproduction

- test_relaxed_v2 passed at 0 and 22.5 degrees: retained constraint rows and derivatives unchanged after deduplication; covariance-only change leaves constraint Jacobian identical; observation Jacobian divides by 100 and squared sensor cost by 10000. Central-difference relative derivative errors 3.87e-7 and 8.63e-8.
- Both-solver first-frame smoke checks passed; then all four sequences ran serially without fallback, continuation or restart.
- Verified 118 frames spanning 0-45 degrees, 48 sensor-only objective terms and zero prior cost in every frame, identical Stage 1 geometry between solvers for each increment size, and 409 constraint rows in all 236 saved-state audits.
- Run setup_tsfs, test_relaxed_v2, run_relaxed_rotation_v2('all'), export_relaxed_rotation_v2. Then run experiments/base_rotation/summarize_relaxed_v2.py and report_relaxed_v2.py. Existing v2 destinations are overwritten by reruns; choose new identifiers for a new experiment.
- Logs and MAT files are under ignored results/runs. Lightweight CSVs, settings, plots, report and source hashes are under results/published.

## Interpretation and next owner

This combined experiment does not isolate the contribution of each change. Uniform sensor scaling alone cannot change a sensor-only exact Stage 2 minimizer at fixed geometry/history; it changes numerical scales, while Stage 1 changes the sensor/environment weighting. Bad previous mechanical outputs still alter friction history even without force priors.

Mentor to select a next investigation before further changes: fixed-problem/history replay at failure onset; squared-cone/apex and beta behavior; or deliberate row equilibration with explicit physical tolerances. No further solver strategy, penalty, history rejection, noise assumption, commit or push is authorized by this handoff.
