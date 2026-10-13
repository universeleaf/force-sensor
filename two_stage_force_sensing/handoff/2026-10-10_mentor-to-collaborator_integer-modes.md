# 2026-10-10 mentor to collaborator: explicit contact modes

Author: Codex for mentor. Intended recipient: collaborator; saved locally, not sent. Branch: codex/two-stage-force-sensing. Base commit: b376013. Changes remain uncommitted; no push.

## Scope and approved decisions

The user requested an explicit mixed-integer friction-mode formulation while retaining nonlinear Cosserat mechanics, preceding-mode initialization, and a full-trajectory success/timing benchmark of the previous inclined-wall problem. Implemented an isolated enumerative solver with exact separation/sticking/sliding modes, circular friction, and local fmincon SQP subproblems. For mu=0, tangential force is eliminated exactly and only separation/active normal contact are enumerated. No geometry, sensor model, force priors, or original solver changes. Default solver and collaborator source remain untouched.

## Implementation

- `matlab/+tsfs/mode_branch.m`: branch coordinates, analytic derivatives through the existing IVP, local nonlinear solves, traces, and explicit callback-failure retention.
- `matlab/+tsfs/mode_solve.m`: all integer assignments, previous mode first, minimum-objective feasible returned solution, aggregate all-attempt accounting. No continuous global certificate.
- `experiments/inclined_wall/run_inclined_modes.m`: unchanged 41-frame reference replay for mu=0.3 and mu=0, non-blocking diagnostics, per-frame checkpointing.
- `experiments/inclined_wall/analyze_inclined_modes.py`: completeness/accounting/geometry checks, numerical summaries, and plots.
- `tests/test_mode_formulation.m`: finite-difference derivative checks and exact frictionless-force elimination.

[Formulation and settings](../docs/formulation/INTEGER_MODE_MECHANICS.md). [Full results](../docs/reports/INCLINED_WALL_INTEGER_MODES_RESULTS.md).

## Results and failures

All 82 estimates returned. Mu=0.3 succeeds on 41/41 frames (40 optimality exits, one small-step), versus 34/41 for the historical relaxed run. Mean time falls from 8.889 to 5.682 s, but median rises from 0.835 to 3.390 s and worst case from 52.85 to 71.01 s. Tip/contact RMSE falls from 3.735/4.871 N to 0.004407/0.003746 N. Shape RMS is 0.000392 mm.

All candidate modes are tested. Sticking consumes 213.03 of 232.95 total seconds and is never selected: 21 RHS-budget exceptions, 10 infeasible exits, one evaluation-limit exit, and five feasible but worse-fit results. Failures are saved and counted, not reinterpreted as global infeasibility. The selected branch averages 0.221 s, but this excludes competing-mode search and is not an end-to-end performance claim.

Three frictional frames retain small dense-mesh penetration warnings (max 1.53e-5 mm); no projection, cone, positive-work, or active-tangency warnings. Mu=0 also succeeds on 41/41, averages 0.463 s, but has tip/contact RMSE 0.487/3.380 N and 24 warning frames. Convergence does not make the frictionless model physically correct for frictional observations.

## Validation and reproduction

Derivative discrepancies for the four mode types are below 3.4e-8. A four-frame smoke test passed for both models. Full validation checks all 82 frames, all-attempt iteration/IVP sums, 48 sensor-only residuals, previous-mode-first ordering, and identical Stage-1 arclengths.

From the package in MATLAB:

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
test_mode_formulation;
run_inclined_modes(Inf,'inclined_wall_modes_NEW_RUN');
```

Original truth comes from the preserved 2026-10-08 inclined-wall packet. Outputs: `results/published/inclined_wall_modes_20261010/`; raw traces: `results/runs/inclined_wall_modes_20261010/comparison.mat`. The analysis script targets the dated benchmark. Source and truth hashes are in summary.json. Prior algorithms are called read-only; none are copied.

## Open issues and next owners

Mentor: assess whether to approve a faster conditional mode-search policy or keep exhaustive enumeration. Such a policy was not implemented. Collaborator: review the modular interface, diagnostics, and reproducibility before adoption. Further validation needed on 3D axial rotation, noise, and multiple contacts. Enumerating 3^C assignments will not scale to many contacts; local continuous solves give no global MINLP certificate. Dense-mesh geometry warnings remain visible and unresolved.
