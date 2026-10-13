# Rotation directional sensitivity handoff

Date: 2026-10-07. Author: Codex assisting mentor. Direction: mentor to collaborator; prepared locally, not sent. Branch codex/two-stage-force-sensing; base commit b376013. No commit or push.

## Request and scope

Examine whether 3D base rotation introduces a weak Y force direction and compare against prior planar sliding. Added offline audit and report only; no estimator, solver, data or previous results were modified. All files remain within two_stage_force_sensing.

[Full report](../docs/reports/ROTATION_DIRECTION_SENSITIVITY.md). Reproducible entry points: experiments/base_rotation/audit_direction_sensitivity.m and summarize_direction_sensitivity.py. Published evidence: results/published/base_rotation_direction_audit_20261007.

## Findings

- Audited 71 independent reference states and 118 recent v2 estimates. Both scenarios use the same current Bishop derivative and physical world-force units, eliminating base moment through terminal equilibrium.
- The planar estimator already had full 3D forces. Planar symmetry kept Y at zero; rotation breaks that decoupling.
- An isolated tip-Y perturbation remains strongly visible. At 33.75 degrees, allowing all other forces to compensate while maintaining the exact sticking branch reduces curvature sensitivity by about 86 times. This profiled Y sensitivity is about 41 times weaker than the prior planar sliding reference.
- The overall smallest force singular value changes little (2.67e-5 at rotation zero to 2.54e-5 /mm/N at 45 degrees). Condition ranges are 231-273 in prior planar sliding, 305-379 for coarse rotation and 305-387 for fine rotation. No exact force singularity appeared.
- The weakest force direction mostly rotates with the base: at 45 degrees its world-Y fraction is 0.526, but base-rotated Y fraction is only 0.046. X and Z compensation is essential; equal-and-opposite Y alone is much more visible.
- Exact-branch conditioning improves at sliding onset (coarse 96.46 to 13.85), despite IPOPT failure there. Numerical branch following, squared-cone degeneracy and auxiliary beta behavior therefore remain important hypotheses, distinct from local observability.
- Actual errors are not mainly Y: coarse/fine IPOPT component RMSE is [0.431,0.494,1.583] / [0.415,0.415,1.384] N; coarse/fine fmincon is [0.0596,0.0220,0.0568] / [0.1814,0.0870,0.0295] N.

The prior published planar condition 98-112 used material bending observations. This audit recomputes both cases with current Bishop observations; out-of-plane perturbation derivatives differ even when nominal twist is zero. Previous experiment results are not overwritten or relabeled.

## Validation and limits

Reference IVPs match saved shapes within 7.01e-8 mm. Ten central-difference checks of isolated Y and weakest force directions agree within 1.03e-7 relative error. Profile equality residuals are below 1e-7. All six force singular values are nonzero. Fixed exact contact-branch constraints are for local diagnostic analysis only, not new estimator assumptions. Geometry/history uncertainty, branch switches and finite force excursions are outside the local derivative interpretation. First planar paired history uses its recorded previous FBG reconstruction; later histories use reference shapes. Sensor-only sensitivity does not depend on that history choice.

## Reproduction and next owner

Set TSFS_RUN_ID=base_rotation_direction_audit_20261007, run setup_tsfs then audit_direction_sensitivity in MATLAB; run summarize_direction_sensitivity.py afterward. New CSVs, full G matrices, figures and reports are preserved separately. No force optimization rerun was needed.

Mentor to choose future solver/branch-following changes. This audit does not authorize implementation of row scaling, mode selection, beta penalties, continuation, history rejection, or noise changes.
