# Completed handoff: single-pass rod / octagonal QP comparison

Date: 2026-10-12. Author/direction: Codex, mentor-side collaborator to collaborator (local only, not sent). Branch codex/two-stage-force-sensing, base b376013. No commit or push.

## Scope and approvals

The user authorized noise levels 0/0.5/1/2 mm, insertion and rotation, exhaustive modes, one rod linearization, full circular friction with 100 SQP iterations per branch, and an inscribed eight-sided friction polygon with quadprog. The user approved removing the single-pass trust bound after it blocked insertion reversal, then approved reporting completed tests against available references instead of finishing optional slow controls. No further numerical changes were made.

## Implementation

New modular functions: single_linearization_solve, polygon_qp_constraints, polygon_qp_branch, polygon_qp_violation. Existing mode_branch reads optional maxIterationsPerAssignment (default 300 unchanged) and omits trust inequalities for an infinite radius. run_inclined_modes accepts optional solverFunction. Experiment run/analyze/publish_single_pass_comparison scripts preserve separate outputs.

See ../docs/formulation/SINGLE_PASS_POLYGON_QP.md. Polygon enumeration includes separation, sticking, eight edge and eight vertex sliding branches (18/contact). Both edge and vertex branches are required for maximum dissipation. Stage 1 remains unchanged. One augmented IVP builds the affine model; another validates the chosen loads without correction or reselection. The final nonlinear state feeds next-frame history. Native affine convergence is distinct from nonlinear feasibility.

## Completed evidence

Final stamp: 20261012_unbounded. All 16 requested trajectories completed: 520 newly solved frames, 260 per method. Each selected affine solution converged and met affine constraints; all nonlinear IVP audits returned. Exactly two IVPs per frame. No selected solution hit the 100-iteration cap (maximum 98 SQP / 28 QP); some alternative branches did, so exhaustive enumeration does not provide an unconditional global certificate.

The complete zero-noise exhaustive outer-loop controls and existing bit-identical 2 mm exhaustive controls are included. Extra 0.5 mm insertion control stopped after five complete frames; exclude it from trajectory aggregates. Remaining optional 0.5/1 mm controls are unavailable. Our batch processes were stopped; the unrelated MATLAB2023a session was untouched.

Circular single-pass mean time: insertion 0.664-0.794 s/frame; rotation 0.180-0.257 s/frame. Polygon: insertion 1.859-1.963 s/frame; rotation 1.045-1.156 s/frame. Circular tip/contact RMS N: noiseless insertion 0.0101/0.0473, rotation 0.0460/0.1434; at 2 mm insertion 3.320/3.774, rotation 3.521/7.764. Polygon force accuracy is similar under noise but worse for noiseless rotation and costs more due to 18 branches.

Critical limitation: only five noiseless insertion frames per method pass strict nonlinear mode feasibility; other tested one-pass trajectories have zero strict passes. At 2 mm circular single-pass maximum active gap error is 0.136 mm insertion / 0.0210 mm rotation, and maximum terminal moment residual is 54.75 / 22.37 N mm. Low force error does not certify nonlinear mechanics. Diagnostics do not change the step.

Exhaustive outer references are slower, but the old conditional noiseless insertion controller was already faster than exhaustive single-pass (0.284 vs 0.783 s/frame). Do not overstate speedup across search policies. One seed and two trajectories do not establish general statistical superiority.

## Validation and artifacts

Full report: ../docs/reports/SINGLE_PASS_ROD_POLYGON_COMPARISON.md.
Combined outputs: ../results/published/singlepass_comparison_20261012_unbounded/ (summary.csv/json, frames.csv, branches.csv, polygon_law_checks.csv, comparison.png). Raw MAT checkpoints: ../results/runs/singlepass_*_20261012_unbounded/comparison.mat. Log: ../singlepass_comparison_20261012_unbounded.log.

Validated complete counts, shared Stage 1 geometry, truth scoring, two-IVP accounting, and 3242 independent polygon support/dissipation and circular-cone containment checks. Figure visually inspected. Descriptive exactCircularFriction labels corrected for polygon artifacts; numerical-array hashes unchanged, recorded in metadata_label_correction.json. Source now sets the correct label. Preserve this correction manifest.

Preliminary bounded runs (stamp 20261012) and smoke runs are preserved and excluded. Removing their radius-4 bound reduced noiseless circular insertion tip/contact RMS from 0.551/2.079 N to 0.0101/0.0473 N.

## Reproduction and next actions

From package root in MATLAB: setup_tsfs; addpath('experiments/inclined_wall'); run_single_pass_comparison('FRESH_STAMP',{'direct','polygon'}). Optional zero-noise reference: run_single_pass_comparison('FRESH_STAMP',{'current'},0). Existing 2 mm exhaustive controls are read automatically. Then run analyze_single_pass_comparison.py FRESH_STAMP and publish_single_pass_report.py FRESH_STAMP. Historical truth/noise inputs are dependencies documented in the runner. Use fresh stamps; partial outputs intentionally block accidental overwrites.

Recommendation for mentor review: use full-cone single-pass only as an approximate fast estimate, retaining nonlinear diagnostics. Strict mechanics still needs correction/relinearization. Adaptive correction or a different mode search would require separate approval; neither was implemented. Polygon QP speed advantage is not supported by this exhaustive single-contact benchmark. No collaborator message was sent and no agents were spawned. All edits are confined to our package; preserve other existing local changes.
