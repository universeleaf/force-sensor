# Exact branch solves on the shared rod linearization (`tsfs.qp_branch`)

Date: 2026-10-25. Branch `codex/two-stage-force-sensing`. Implementation: [qp_branch.m](../../matlab/+tsfs/qp_branch.m), selected with `cfg.singlePassMethod='profiled-qp'` inside the unchanged [single_linearization_solve.m](../../matlab/+tsfs/single_linearization_solve.m).

## What is unchanged

Stage 1 geometry, the frozen contact arc lengths/normals/offsets, the single rod linearization per frame, the exhaustive 3^C full-cone mode enumeration, selection of the lowest-objective affine-feasible branch, the nonlinear audit IVP and the single fixed-load BVP are the existing framework. `qp_branch` is a drop-in replacement for `mode_branch`: same inputs, same output fields (`z`, `evaluation`, `mode`, `modeY` layout, `modeViolation`, ...). With both options below disabled it minimizes exactly the same affine problem as `mode_branch`.

## Why each branch is a QP or a one-angle family of QPs

After the rod linearization every quantity used by a branch is affine in the Cartesian scaled loads z (see `linearized_evaluate`): whitened FBG residual r(z)=r0+J dz, terminal moment, sampled non-penetration rows, contact gap g_i(z) and tangential displacement v_i(z). The objective 0.5||r(z)||^2 is convex quadratic.

| Branch | Constraints | Solve |
|---|---|---|
| Separation | x_i = 0; all sampled non-penetration rows | one QP |
| Sticking | g_i = 0, v_i = 0, lambda_i >= 0, ‖tau_i‖ <= mu lambda_i | QP without the cone; if the cone is violated it is active at the optimum (convex problem), so tau_i = mu lambda_i d(phi) and the minimum is a profile over phi |
| Sliding | g_i = 0, lambda_i >= 0, a_i >= 0, v_i = a_i d(theta), tau_i = -mu lambda_i d(theta) | for fixed theta all rows are linear: one QP per angle; profile over theta |

d(theta) = [cos theta; sin theta]. The angle profile is evaluated on a 36-point periodic grid plus the history/initial direction, and the three best local grid minima are refined with bounded `fminbnd` (tolerance 1e-7 rad). Infeasible angles are excluded rather than penalized. This is not an approximation of the friction law (no polygon, no frozen direction): for one contact the returned branch minimum is global up to the grid/refinement tolerance. Several contacts needing angles use coordinate descent over the angles and are reported as `certificate=false`.

QPs use MATLAB `quadprog` (active-set) with constraint generation over the sampled non-penetration rows: rows within 1 mm of the plane at the expansion point start in the working set, violated rows are added until all rows hold (the result is then optimal for the full row set). Exact gap equality replaces the active contact's own sampled row, as in `mode_branch`.

## Optional switches (default off)

| Switch | Meaning | Status of the idea |
|---|---|---|
| `cfg.forcePriors=true` | Keeps the framework's existing force-tracking rows from `tsfs.initialize`: (F_tip - previous)/`o.tipProcessStd`, (F_i - matched previous)/`o.forceProcessStd`. Unmatched candidates get the existing zero-mean `forceStd` prior. | Existing prior, previously sliced away in the fine experiments; handoff item B |
| `cfg.forcePriorTangentRatio=k` | Tangential rows of the contact prior use std k·forceStd (1 = original isotropic, Inf = normal only). Friction can flip by 2·mu·lambda within one frame at slip reversal. | Parameterization of the existing prior |
| `cfg.slipStd='stage1'` | Slip-history equations v_i = 0 / v_i = a_i d become residuals whitened by B_i' J_p C_w J_p' B_i + floor^2 I, where J_p C_w J_p' is the Stage 1 position covariance at s_i (the same propagation Stage 1 already uses) and the floor is `o.geometryPositionModelStdMm`. The previous contact point is an estimate, not exact history. | Handoff item D (history uncertainty) |
| `cfg.bvpTrust` | In `single_pass_bvp_solve`: if the BVP is unconverged, or its FBG RMS exceeds 1.5x the pre-BVP value + 0.25, the pre-BVP nonlinear state is kept and propagated. Forces are unchanged either way (fixed-load BVP). | Handoff request: explicit policy for untrusted corrected states |
| `o.batchedReconstruction=true` | Stage 1 sensitivities with the same central differences, integrated in one batched pass ([reconstruct_bishop_sensitivity.m](../../matlab/+tsfs/reconstruct_bishop_sensitivity.m)); Jacobian difference ~1e-10 relative. | Speed only |

Objective values used for selection include prior/slip rows when enabled (`selectionObjective`), so branch comparison stays consistent across modes. `modeViolation` measures only hard constraints; with soft history the nonlinear `mode_residual` slip rows are diagnostics, not feasibility conditions.
