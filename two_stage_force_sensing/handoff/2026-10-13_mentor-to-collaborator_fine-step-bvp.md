# Completed finer-step rod/BVP experiment

2026-10-13. Author/direction: Codex, mentor-side collaborator to collaborator (local only). Branch codex/two-stage-force-sensing, base b376013. No commit or push.

User-authorized scope: reduce actuation increments fivefold, preserve one linearized rod solve with exhaustive full circular modes and one fixed-load nonlinear BVP, with no outer correction loop. Insertion 0.4 mm /201 states; rotation 0.75 degrees /121 states (restored seed excluded). Noise targets 0/0.5/1/2 mm. All eight trajectories and 1284 new solves are complete; MATLAB batch exited successfully.

Implementation: optional step/folder parameters in build_inclined_truth.m and build_inclined_rotation_truth.m preserve legacy defaults; run_inclined_modes.m adds optional checkpointEvery (default1; fine runner10) and omits redundant dense interpolation objects after diagnostics. Added run_fine_bvp_experiment.m and analyze_fine_bvp.py. Core mechanics/BVP solver settings unchanged. All writes are within our package; collaborator algorithms are called read-only. Existing dirty/untracked work is preserved.

References were regenerated, not interpolated. All reference frames pass their audits. Common insertion truth forces match coarse exactly; rotation reference changes by 0.205 N RMS /0.373 N maximum owing to friction history. Noise std is unchanged; common samples reuse seed33001 exactly, added frames use independent seed33005 samples. Rotation seed noise is taken from insertion frame101. Timing remains quasistatic bookkeeping.

| Motion | Noise mm | Tip RMSE N | Contact RMSE N | Shape RMS mm | Mean s/frame |
|---|---|---|---|---|---|
| insertion | 0 | 0.0149899 | 0.0232678 | 0.0127045 | 0.86573 |
| rotation | 0 | 0.108739 | 0.317639 | 0.0134295 | 0.2394 |
| insertion | 0.5 | 0.893892 | 0.994725 | 0.124788 | 0.458269 |
| rotation | 0.5 | 2.23925 | 5.46476 | 1.08166 | 0.276309 |
| insertion | 1 | 1.81596 | 2.05821 | 0.298985 | 0.449616 |
| rotation | 1 | 2.80057 | 5.57598 | 0.727983 | 0.290526 |
| insertion | 2 | 4.25986 | 6.05116 | 7.2493 | 0.446851 |
| rotation | 2 | 8.00554 | 10.9036 | 8.58824 | 0.354516 |

Validation: 1279 affine/native successes; 1281/1284 BVP convergence; 169/1284 strict nonlinear selected-mode successes. Exact force invariance, exhaustive assignments, zero outer iterations, and exact common noise draws verified. 3710 branch solves; 287 hit the cap, including5 selected branches. 3851 mechanics IVPs plus BVP collocation work. Total inverse solve time 585.604 s; excludes truth generation and file serialization. Comparison figure visually inspected.

Failures: 2 mm insertion frame173 and rotation frame72 hit BVP mesh limits; warning-returned states are audited and passed onward by the existing wrapper. Rotation frame78 throws singular-collocation-Jacobian and retains its pre-BVP state. All failures remain in metrics. No retries or unapproved fallback changes. Terminal moments on failed frames reach686 N mm. Gap/friction violations persist even with converged BVPs.

Interpretation: noiseless rotation improves substantially; noiseless insertion shape/contact improve, with a new intermediate force spike. Common-grid insertion force errors at0.5/1 mm are nearly unchanged. Noisy rotation is mixed/worse, and2 mm has major failures. Smaller steps reduce deterministic linearization changes, but iid sensor noise differences remain O(sigma), so uncertain history/slip becomes relatively more important. This is evidence consistent with the mechanism, not a causal ablation or statistical ranking.

Reproduction: setup_tsfs; addpath('experiments/inclined_wall'); run_fine_bvp_experiment('20261012_step5'). Analyze with python experiments/inclined_wall/analyze_fine_bvp.py 20261012_step5. Existing complete outputs are skipped; partial inverse outputs are not overwritten. For fresh stamps copy noise_payload.mat and noise_protocol.json into the fresh input folder.

Report: docs/reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md. Combined outputs: results/published/fine_bvp_comparison_20261012_step5/{summary.csv,summary.json,frames.csv,branches.csv,truth_audit.csv,comparison.png}. Raw trajectories/truth/inputs: results/runs/fine_bvp_*_20261012_step5/. Log: fine_bvp_20261012_step5.log.

Proposed next discussion (user approval before implementation): treat history/slip uncertainty consistently, inspect noisy rotation within-mode constraints, and define safe handling of nonconverged BVP states. Do not infer authorization for extra inner loops, smoothing, new priors, or altered acceptance criteria. Owner: mentor/user decides the next formulation.
