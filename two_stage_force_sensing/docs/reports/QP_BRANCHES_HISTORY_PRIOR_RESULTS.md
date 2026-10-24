# Exact QP branch solves, uncertain slip history and the existing force prior

Date: 2026-10-25. Author: collaborator side (Tongyu, prepared with Claude Code). Branch `codex/two-stage-force-sensing`, base `8d6df5a`. Responds to items A–C of the [2026-10-13 handoff](../../handoff/2026-10-13_mentor-to-collaborator_current-method-and-next-steps.md) and to its proposed next discussion (history/slip uncertainty, nonconverged BVP states).

## 1. Summary

The framework is unchanged: Stage 1 geometry -> one rod linearization -> exhaustive full-cone modes -> one fixed-load BVP. Changes are switches inside it (all default off, so existing runs are unaffected):

1. **Branch optimizer.** `tsfs.qp_branch` replaces `mode_branch` (fmincon SQP) with exact convex QPs; sliding and cone-edge sticking are a one-angle profile of QPs. On identical expansion points it never returned a worse branch except one case where fmincon used its 1e-6 constraint tolerance, found 230 strictly better branches out of 3710, and removed all 294 iteration-cap hits.
2. **Uncertain slip history.** The previous estimated contact point is treated as an estimate: slip equations are weighted by the Stage 1 position covariance instead of imposed exactly.
3. **Existing force-tracking prior.** The random-walk prior already in `tsfs.initialize` (`o.tipProcessStd`, `o.forceProcessStd`), which the fine experiments sliced away, is kept in the branch objective.
4. **Untrusted BVP states.** An unconverged or fit-degrading BVP correction is not propagated as history.

Held-out seeds 0, 3, 4, 5 (never used for tuning), fine-step insertion/rotation and a time-varying tip-load insertion, RMS vector errors including every frame:

| Noise | Case | Reference (fmincon) tip / contact N | **Recommended default** `qp_trust` | Optional `qp_soft_tip0.2_f1` |
|---|---|---|---|---|
| 0.5 mm | insertion | 0.94 / 1.12 | 0.92 / 1.08 (qp) | **0.25 / 0.65** |
| 0.5 mm | rotation | 1.66 / 4.63 | 1.20 / 3.77 (qp) | **0.17 / 0.78** |
| 1 mm | insertion | 1.79 / 2.12 | 1.79 / 2.11 | **0.54 / 1.49** |
| 1 mm | rotation | 3.64 / 6.85 (max tip 21.8) | 2.16 / 5.55 (max 6.8) | **0.45 / 1.37** (max 1.0) |
| 2 mm | insertion | 3.86 / 4.90 (shape 3.37 mm) | **3.68 / 4.34** (shape 0.87 mm) | 6.09 / 7.85: lock-in failure |
| 2 mm | rotation | 11.57 / 10.27 (max tip 176, shape 11.1 mm) | **3.77 / 8.04** (max 11.9, shape 1.1 mm) | 7.55 / 10.69: lock-in failure |

- **Recommended default: `profiled-qp` + `bvpTrust`.** No new modeling assumption. Equal or better than the reference at every noise level and scenario; the gains are in rotation and at 2 mm (fewer local-solver failures, no propagation of failed BVP states). Insertion at 0.5/1 mm is unchanged because it is already at the single-frame observability limit.
- **Optional: uncertain history + existing force prior.** 3-9x lower tip error and 2-6x lower rotation contact error at 0.5-1 mm, with far smoother trajectories. It is not safe as a default in this fixture: at 2 mm it locks into a separation/tip-drift state (the prior makes the drifted state absorbing), it lags the friction flip at insertion reversal, it biases slow tip-load changes, and with noiseless data it raises tip RMSE from 0.015 to 0.20 N. These are reported, not tuned away.

Main limitations (measured, Sections 6-7): the prior cannot create observability. Along the weak tip/contact compensation direction it converts frame-to-frame noise into bias and lag (a 1 -> 2 N tip ramp and a 2 -> 0.5 N step are barely followed), and when the data are weak (2 mm) the recurrent prior can lock the estimate into separation with a drifting tip force. Exact slip history remains better than uncertain history for steady insertion sliding but fails in sticking-dominated rotation. Two extra mechanisms that were tried to fix these (a mode-switch penalty and an FBG-acceptance gate on the prior) made results worse and were removed.

## 2. What changed in code

| File | Change | Default behavior |
|---|---|---|
| `matlab/+tsfs/qp_branch.m` (new) | Drop-in replacement for `mode_branch` on the shared affine model | Not called unless `cfg.singlePassMethod='profiled-qp'` |
| `matlab/+tsfs/single_linearization_solve.m` | +4 lines: dispatch to `qp_branch`; select with `selectionObjective` when present | Identical |
| `matlab/+tsfs/single_pass_bvp_solve.m` | +11 lines: optional `cfg.bvpTrust` policy | Identical (`bvpTrust` absent) |
| `matlab/+tsfs/reconstruct.m`, `geometry.m`, `reconstruct_bishop_sensitivity.m` (new) | Optional batched Stage 1 central differences | Identical (`o.batchedReconstruction` absent) |
| `experiments/robust_pass/` (new) | Runner, paired suite, shadow comparison, analysis, plots | — |

Formulation and switch definitions: [PROFILED_QP_BRANCHES.md](../formulation/PROFILED_QP_BRANCHES.md). No physics, observation model, noise model, Stage 1 formulation, BVP, enumeration or selection rule was changed for the default path. `mode_branch`, `initialize`, `evaluate` and the collaborator algorithms are untouched.

## 3. Reproduction of the published fine-step reference

All runs here use MATLAB R2024a, single computational thread per process, and references regenerated with the existing builders (`run_fine_bvp_experiment(stamp,'truth')`, 379 s). The runner `run_robust_trajectory` follows `run_inclined_modes` frame logic but writes only under `results/runs/`.

Rerunning the unchanged reference on the published noise payload reproduces 0 mm (both motions), 0.5 mm insertion and 1 mm rotation to all printed digits. The remaining cases differ (e.g. 2 mm rotation tip RMSE 8.01 N published vs 4.96 N rerun) because the recurrent trajectory amplifies floating-point differences (R2025b vs R2024a, threading) into different branch/BVP outcomes; the rerun did not hit the published 2 mm BVP mesh failures. Single-seed 2 mm results are therefore not a stable ranking, which is why the comparison below uses several held-out seeds.

## 4. Branch optimizer on identical inputs (handoff item C)

`compare_branch_solvers` follows the reference trajectory and, at every frame, also solves all branches with `qp_branch` on the same frame, geometry, previous state, initializer and rod linearization (asserted identical). The QP result is never propagated. Seed 0 (published payload), all eight fine trajectories:

| Scenario | Noise mm | Frames | Branches | QP lower objective | QP higher objective | SQP at 100-iteration cap | SQP infeasible, QP feasible | Same selected mode | Frames with lower selected objective (QP) | Max selected force difference N | SQP branch s | QP branch s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| insertion | 0 | 201 | 567 | 4 | 0 | 0 | 4 | 201 | 0 | 0.00 | 550.6 | 27.0 |
| insertion | 0.5 | 201 | 567 | 9 | 0 | 66 | 2 | 198 | 6 | 2.94 | 180.9 | 22.1 |
| insertion | 1 | 201 | 569 | 15 | 0 | 74 | 8 | 192 | 8 | 6.63 | 173.0 | 47.5 |
| insertion | 2 | 201 | 567 | 26 | 0 | 66 | 11 | 192 | 13 | 18.69 | 144.8 | 32.0 |
| rotation | 0 | 120 | 360 | 50 | 1 | 1 | 1 | 119 | 10 | 0.97 | 11.6 | 31.5 |
| rotation | 0.5 | 120 | 360 | 49 | 0 | 27 | 20 | 98 | 21 | 14.66 | 26.6 | 17.7 |
| rotation | 1 | 120 | 360 | 46 | 0 | 26 | 24 | 104 | 14 | 14.07 | 29.8 | 35.1 |
| rotation | 2 | 120 | 360 | 31 | 0 | 34 | 20 | 106 | 14 | 31.48 | 52.4 | 34.0 |

"QP lower/higher objective" uses a 1e-6 relative tolerance. The single higher-objective branch (noiseless rotation frame 29, sticking) is a fmincon point violating the slip/gap rows by 7.3e-7 (inside its 1e-6 tolerance) with objective 0.026 versus 0.371 for the exactly feasible QP point, i.e. that branch is very sensitive to sub-micrometre constraint relaxation. When fmincon converges, both solvers agree to ~1e-11 in objective and ~1e-6 N in force. Branch time is 4–20x lower for insertion; for noiseless and 1 mm rotation the angle profiles (sliding and cone-edge sticking both need one) make the QP path slower than fmincon's quick convergences. One contact means every branch minimum is global up to the 36-point grid plus 1e-7 rad refinement (`certificate=true` on all frames); this is not claimed for several angle contacts.

## 5. Why noisy runs spike (handoff item A)

| Mechanism | Evidence | Remedy tested |
|---|---|---|
| Single-frame observability limit (tip/contact compensation) | Insertion errors at 0.5/1 mm are unchanged by exact branch solves (0.894 -> 0.914 N, 1.821 -> 1.835 N tip RMSE, seed 0) and match the earlier local sensitivity prediction (~0.9 N per 0.5 mm) | Only prior information helps: force-tracking prior |
| Suboptimal fmincon branch solves | Section 4: up to 21/120 rotation frames with a lower-objective QP selection, force differences up to 31 N; in 0.5 mm rotation a 20-frame ~10 N contact-error regime disappears with exact branches | `qp_branch` |
| Noisy previous shape treated as exact history | Exact sticking pins the contact point to the previous noisy estimate: insertion 1 mm seed 1, median sticking-branch objective 5.4e4 (max 1.5e6) with exact history vs 32 (max 90) with uncertain history; with the prior, rotation contact error stays at 6.7 N (seeds 1-2, 1 mm) vs 1.4 N with uncertain history | `slipStd='stage1'` |
| Unconverged BVP passed into history | 2 mm runs: BVP mesh-limit states with tens of mm shape error propagate | `bvpTrust` |
| Friction flip at slip reversal vs isotropic contact prior | With uncertain history the estimator keeps sticking with the old friction direction for ~5 frames after insertion reversal (true 13 N tangential jump) | Tangential ratio `r` trades insertion for rotation; not adopted |

| Case | Frame | Config | Mode | Tip err N | Contact err N | Shape RMS mm | FBG RMS (pre-BVP) | Terminal moment N mm | BVP conv./rejected | SQP-QP selected objective |
|---|---|---|---|---|---|---|---|---|---|---|
| insertion 0 mm | 99 | reference | 2 | 0.181 | 0.310 | 0.037 | 0.07 (0.07) | 6.25e-07 | 1/0 | -2.13e-10 |
| insertion 0 mm | 99 | qp | 2 | 0.181 | 0.310 | 0.037 | 0.07 (0.07) | 6.25e-07 | 1/0 | -2.13e-10 |
| insertion 0 mm | 99 | qp_tip0.2_f1 | 2 | 0.317 | 0.478 | 0.037 | 0.08 (0.08) | 6.04e-07 | 1/0 | -2.13e-10 |
| insertion 0 mm | 99 | qp_soft_tip0.2_f1 | 2 | 0.317 | 0.478 | 0.037 | 0.08 (0.08) | 6.04e-07 | 1/0 | -2.13e-10 |
| insertion 0 mm | 102 | reference | 2 | 0.059 | 0.007 | 0.174 | 0.12 (0.13) | 4.05e-07 | 1/0 | 3.69e-13 |
| insertion 0 mm | 102 | qp | 2 | 0.059 | 0.007 | 0.174 | 0.12 (0.13) | 4.05e-07 | 1/0 | 3.69e-13 |
| insertion 0 mm | 102 | qp_tip0.2_f1 | 2 | 0.261 | 0.188 | 0.164 | 0.13 (0.17) | 4.22e-07 | 1/0 | 3.69e-13 |
| insertion 0 mm | 102 | qp_soft_tip0.2_f1 | 1 | 0.417 | 8.602 | 0.101 | 0.75 (0.77) | 5.11e-08 | 1/0 | 3.69e-13 |
| rotation 0.5 mm | 91 | reference | 1 | 12.675 | 14.487 | 1.087 | 5.48 (5.67) | 4.39e-07 | 1/0 | 687 |
| rotation 0.5 mm | 91 | qp | 1 | 0.072 | 1.377 | 0.088 | 0.96 (0.96) | 8.51e-08 | 1/0 | 687 |
| rotation 0.5 mm | 91 | qp_tip0.2_f1 | 1 | 1.270 | 7.355 | 0.211 | 1.32 (1.32) | 2.46e-07 | 1/0 | 687 |
| rotation 0.5 mm | 91 | qp_soft_tip0.2_f1 | 1 | 0.040 | 0.773 | 0.100 | 0.93 (0.93) | 8.67e-08 | 1/0 | 687 |
| insertion 2 mm | 173 | reference | 2 | 18.159 | 19.434 | 49.553 | 12.28 (5.92) | 602 | 0/0 | 1.72e-10 |
| insertion 2 mm | 173 | qp | 2 | 1.895 | 1.677 | 0.237 | 0.89 (0.89) | 2.14e-07 | 1/0 | 1.72e-10 |
| insertion 2 mm | 173 | qp_trust | 2 | 1.811 | 1.592 | 0.105 | 0.90 (0.90) | 1.62e-07 | 1/0 | 1.72e-10 |
| insertion 2 mm | 173 | qp_tip0.2_f1 | 0 | 3.532 | 2.058 | 0.532 | 0.98 (0.98) | 3.46e-07 | 1/0 | 1.72e-10 |
| insertion 2 mm | 173 | qp_soft_tip0.2_f1 | 0 | 3.532 | 2.058 | 0.532 | 0.98 (0.98) | 3.46e-07 | 1/0 | 1.72e-10 |
| rotation 2 mm | 72 | reference | 2 | 0.937 | 3.204 | 0.644 | 1.14 (1.14) | 3.67e-08 | 1/0 | 0.000201 |
| rotation 2 mm | 72 | qp | 2 | 0.758 | 2.894 | 0.249 | 1.09 (1.09) | 2.92e-08 | 1/0 | 0.000201 |
| rotation 2 mm | 72 | qp_trust | 2 | 0.761 | 2.899 | 0.252 | 1.09 (1.09) | 2.92e-08 | 1/0 | 0.000201 |
| rotation 2 mm | 72 | qp_tip0.2_f1 | 2 | 11.097 | 9.781 | 8.824 | 2.67 (2.67) | 7.22e-07 | 1/0 | 0.000201 |
| rotation 2 mm | 72 | qp_soft_tip0.2_f1 | 1 | 5.402 | 9.124 | 0.523 | 1.05 (1.04) | 2.77e-07 | 1/0 | 0.000201 |
| rotation 2 mm | 78 | reference | 2 | 1.729 | 10.416 | 4.864 | 1.42 (1.42) | 2.19e-07 | 1/0 | -5.1e-12 |
| rotation 2 mm | 78 | qp | 2 | 1.844 | 5.202 | 0.652 | 0.89 (0.88) | 2.73e-08 | 1/0 | -5.1e-12 |
| rotation 2 mm | 78 | qp_trust | 2 | 1.843 | 5.202 | 0.653 | 0.89 (0.88) | 2.73e-08 | 1/0 | -5.1e-12 |
| rotation 2 mm | 78 | qp_tip0.2_f1 | 1 | 10.982 | 11.063 | 8.505 | 2.24 (2.24) | 4.96e-07 | 1/0 | -5.1e-12 |
| rotation 2 mm | 78 | qp_soft_tip0.2_f1 | 1 | 5.258 | 8.682 | 0.533 | 0.97 (0.97) | 3.04e-07 | 1/0 | -5.1e-12 |

Interpretation (seed 0 = published payload): rotation 0.5 mm frame 91, the 12.7 N spike, is a local-optimizer failure. On identical inputs the fmincon selection has an objective 687 units above the QP selection (FBG 5.48 vs 0.96), and the exact branch solve gives 0.07 N tip error. Insertion 2 mm frame 173 is a BVP mesh-limit state (49.6 mm shape error) in the reference; the QP trajectories do not reach that state. Noiseless insertion frames 99/102 are unchanged by the solver; with uncertain history plus prior, frame 102 (reversal) selects sticking and keeps the old friction direction (8.6 N contact error), the reversal lag of Section 6. At 2 mm rotation frames 72/78 the prior configurations are in their lock-in state.

## 6. Prior settings (handoff item B), tuning seeds 1–2, 1 mm

Settings were tuned only on seeds 1 and 2 and evaluated on held-out seeds 0, 3, 4, 5 (Section 7). `qp_tip<a>_f<b>` = hard history with the existing prior (tip std a, contact std b N per frame); `qp_soft_...` adds uncertain history; `r` = tangential std ratio. All prior/soft configurations include `bvpTrust`.

| Scenario | Noise mm | Config | Seeds | Tip RMSE N | Contact RMSE N | Shape RMS mm | Max tip N | Max contact N | Tip jitter N | Contact jitter N | Mode acc. | Mean s/frame | BVP not conv. | BVP rejected | FBG>1.24 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| insertion | 1 | qp | 1 2 | 1.717 | 2.041 | 0.295 | 7.06 | 7.01 | 2.410 | 2.794 | 0.90 | 0.809 | 0 | 0 | 2 |
| insertion | 1 | qp_tip0.1_f1 | 1 2 | 0.372 | 0.479 | 0.218 | 0.94 | 2.18 | 0.246 | 0.403 | 0.97 | 0.776 | 0 | 0 | 3 |
| insertion | 1 | qp_soft_tip0.05_f1 | 1 2 | 0.348 | 2.094 | 0.262 | 0.91 | 9.53 | 0.146 | 1.939 | 0.77 | 0.717 | 0 | 0 | 4 |
| insertion | 1 | qp_soft_tip0.1_f1 | 1 2 | 0.315 | 1.816 | 0.265 | 0.92 | 11.67 | 0.216 | 1.242 | 0.91 | 0.833 | 0 | 0 | 4 |
| insertion | 1 | qp_soft_tip0.2_f1 | 1 2 | 0.436 | 1.706 | 0.280 | 1.18 | 11.48 | 0.264 | 1.005 | 0.95 | 0.690 | 0 | 0 | 5 |
| insertion | 1 | qp_soft_tip0.1_f2 | 1 2 | 0.257 | 1.301 | 0.262 | 0.84 | 8.79 | 0.200 | 1.674 | 0.94 | 0.707 | 0 | 0 | 4 |
| insertion | 1 | qp_soft_tip0.1_f1_r2 | 1 2 | 0.292 | 1.481 | 0.263 | 0.90 | 9.59 | 0.201 | 1.555 | 0.93 | 0.705 | 0 | 0 | 3 |
| insertion | 1 | qp_soft_tip0.1_f1_r4 | 1 2 | 0.290 | 1.348 | 0.263 | 0.90 | 9.28 | 0.200 | 1.716 | 0.94 | 0.833 | 0 | 0 | 3 |
| insertion | 2 | qp | 1 2 | 3.578 | 4.227 | 1.167 | 12.27 | 15.90 | 4.957 | 5.694 | 0.80 | 0.496 | 0 | 0 | 5 |
| insertion | 2 | qp_soft_tip0.2_f1 | 1 2 | 5.172 | 6.487 | 0.775 | 9.66 | 15.57 | 0.523 | 1.376 | 0.48 | 0.873 | 0 | 0 | 21 |
| rotation | 1 | qp | 1 2 | 2.207 | 5.719 | 0.392 | 5.96 | 14.98 | 2.929 | 6.016 | 0.60 | 0.872 | 0 | 0 | 13 |
| rotation | 1 | qp_tip0.1_f1 | 1 2 | 1.459 | 6.676 | 0.463 | 2.02 | 13.75 | 0.478 | 3.800 | 0.52 | 0.741 | 0 | 0 | 49 |
| rotation | 1 | qp_soft_tip0.05_f1 | 1 2 | 0.371 | 3.122 | 0.308 | 0.76 | 10.03 | 0.122 | 2.387 | 0.51 | 0.689 | 0 | 0 | 4 |
| rotation | 1 | qp_soft_tip0.1_f1 | 1 2 | 0.403 | 1.726 | 0.296 | 0.79 | 5.54 | 0.203 | 1.265 | 0.55 | 0.851 | 0 | 0 | 2 |
| rotation | 1 | qp_soft_tip0.2_f1 | 1 2 | 0.500 | 1.362 | 0.301 | 0.96 | 3.66 | 0.264 | 0.760 | 0.56 | 0.758 | 0 | 0 | 2 |
| rotation | 1 | qp_soft_tip0.1_f2 | 1 2 | 0.282 | 2.481 | 0.290 | 0.72 | 9.01 | 0.151 | 2.443 | 0.55 | 0.671 | 0 | 0 | 2 |
| rotation | 1 | qp_soft_tip0.1_f1_r2 | 1 2 | 0.379 | 2.411 | 0.290 | 0.78 | 8.49 | 0.160 | 2.198 | 0.56 | 0.732 | 0 | 0 | 2 |
| rotation | 1 | qp_soft_tip0.1_f1_r4 | 1 2 | 0.377 | 2.753 | 0.288 | 0.79 | 9.85 | 0.142 | 2.723 | 0.52 | 0.855 | 0 | 0 | 2 |
| rotation | 2 | qp_soft_tip0.2_f1 | 1 2 | 5.574 | 8.904 | 0.672 | 9.16 | 14.21 | 0.627 | 1.460 | 0.55 | 0.819 | 0 | 0 | 6 |
| varying | 1 | qp | 1 2 | 1.786 | 2.096 | 0.290 | 7.65 | 7.60 | 2.489 | 2.858 | - | 0.811 | 0 | 0 | 2 |
| varying | 1 | qp_tip0.1_f1 | 1 2 | 0.674 | 0.716 | 0.225 | 1.29 | 2.14 | 0.271 | 0.419 | - | 0.691 | 0 | 0 | 4 |
| varying | 1 | qp_soft_tip0.05_f1 | 1 2 | 0.679 | 2.106 | 0.298 | 1.24 | 9.33 | 0.186 | 1.913 | - | 0.962 | 0 | 0 | 9 |
| varying | 1 | qp_soft_tip0.1_f1 | 1 2 | 0.641 | 1.953 | 0.271 | 1.20 | 11.69 | 0.241 | 1.235 | - | 0.778 | 0 | 0 | 5 |
| varying | 1 | qp_soft_tip0.2_f1 | 1 2 | 0.596 | 1.746 | 0.282 | 1.51 | 11.27 | 0.284 | 0.964 | - | 0.744 | 0 | 0 | 3 |
| varying | 1 | qp_soft_tip0.1_f2 | 1 2 | 0.656 | 1.573 | 0.267 | 1.18 | 8.97 | 0.227 | 1.616 | - | 0.978 | 0 | 0 | 4 |
| varying | 1 | qp_soft_tip0.1_f1_r2 | 1 2 | 0.629 | 1.601 | 0.267 | 1.19 | 9.72 | 0.228 | 1.567 | - | 0.708 | 0 | 0 | 4 |
| varying | 1 | qp_soft_tip0.1_f1_r4 | 1 2 | 0.628 | 1.591 | 0.268 | 1.19 | 9.42 | 0.226 | 1.680 | - | 0.775 | 0 | 0 | 4 |
| varying | 2 | qp_soft_tip0.2_f1 | 1 2 | 4.967 | 6.193 | 0.690 | 9.32 | 14.25 | 0.509 | 1.335 | - | 0.810 | 0 | 0 | 19 |

Observations: (i) every prior setting removes most frame-to-frame tip noise (tip jitter 2.4 -> 0.15–0.28 N); (ii) with hard history the prior gives the best insertion contact force (0.48 N) but rotation contact stays at 6.7 N; (iii) uncertain history fixes rotation (1.36–1.73 N) and loses insertion accuracy around reversal; (iv) a mode-switch penalty (tested, 10 objective units) was harmful: the first contact onset is itself a switch, and the estimator stayed in separation while the tip force absorbed the contact load (tip error 8.5 N). It was removed. A second fix attempt, using the prior only after a previous frame that passed the existing FBG acceptance (1.24), did not prevent the 2 mm lock-in (insertion tip 4.33 vs 3.58 N for `qp`) and erased most of the 1 mm gain (0.44 -> 1.31 N); it was removed as well. Setting carried to the held-out evaluation: `qp_soft_tip0.2_f1` (the original isotropic prior form), together with its exact-history counterpart `qp_tip0.2_f1` for the ablation.

Varying-tip reference (insertion with tip X = 1 N, ramp to 2 N over frames 61–80, step to 0.5 N at frame 131; [builder](../../experiments/robust_pass/build_varying_tip_truth.m)) measures lag: the prior estimate stays near 1.2–1.4 N through the ramp and the step, while the unregularized estimate follows the truth on average with ±3 N noise. The ramp/step lies largely along the weak tip/contact direction, so the data do not move the estimate against the prior. RMSE still improves (1.79 -> 0.60 N tip) because noise dominates, but this is bias, not tracking.

## 7. Held-out seeds 0, 3, 4, 5

| Scenario | Noise mm | Config | Seeds | Tip RMSE N | Contact RMSE N | Shape RMS mm | Max tip N | Max contact N | Tip jitter N | Contact jitter N | Mode acc. | Mean s/frame | BVP not conv. | BVP rejected | FBG>1.24 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| insertion | 0 | reference | 0 | 0.015 | 0.023 | 0.013 | 0.18 | 0.31 | 0.019 | 0.032 | 1.00 | 3.359 | 0 | 0 | 0 |
| insertion | 0 | qp | 0 | 0.015 | 0.023 | 0.013 | 0.18 | 0.31 | 0.019 | 0.032 | 1.00 | 0.915 | 0 | 0 | 0 |
| insertion | 0 | qp_tip0.2_f1 | 0 | 0.197 | 0.252 | 0.014 | 0.32 | 0.48 | 0.016 | 0.029 | 1.00 | 1.166 | 0 | 0 | 0 |
| insertion | 0 | qp_soft_tip0.2_f1 | 0 | 0.201 | 0.658 | 0.012 | 0.42 | 8.60 | 0.041 | 0.874 | 0.99 | 0.978 | 0 | 0 | 0 |
| insertion | 0.5 | reference | 0 3 4 5 | 0.940 | 1.119 | 0.137 | 4.21 | 4.56 | 1.213 | 1.441 | 0.92 | 1.444 | 0 | 0 | 5 |
| insertion | 0.5 | qp | 0 3 4 5 | 0.920 | 1.079 | 0.137 | 4.21 | 4.22 | 1.238 | 1.461 | 0.94 | 0.838 | 0 | 0 | 4 |
| insertion | 0.5 | qp_tip0.2_f1 | 0 3 4 5 | 0.250 | 0.301 | 0.117 | 0.64 | 1.00 | 0.140 | 0.198 | 0.99 | 0.864 | 0 | 0 | 5 |
| insertion | 0.5 | qp_soft_tip0.2_f1 | 0 3 4 5 | 0.251 | 0.645 | 0.139 | 0.66 | 9.05 | 0.145 | 0.847 | 0.98 | 0.771 | 0 | 0 | 5 |
| insertion | 1 | reference | 0 3 4 5 | 1.790 | 2.121 | 0.302 | 8.43 | 10.47 | 2.312 | 2.738 | 0.87 | 1.386 | 0 | 0 | 4 |
| insertion | 1 | qp | 0 3 4 5 | 1.790 | 2.114 | 0.300 | 8.43 | 10.47 | 2.414 | 2.852 | 0.89 | 0.874 | 0 | 0 | 4 |
| insertion | 1 | qp_trust | 0 3 4 5 | 1.790 | 2.114 | 0.300 | 8.43 | 10.47 | 2.414 | 2.852 | 0.89 | 0.856 | 0 | 0 | 4 |
| insertion | 1 | qp_tip0.2_f1 | 0 3 4 5 | 0.558 | 0.704 | 0.238 | 1.19 | 2.25 | 0.252 | 0.378 | 0.97 | 0.842 | 0 | 0 | 6 |
| insertion | 1 | qp_soft_tip0.2_f1 | 0 3 4 5 | 0.536 | 1.487 | 0.283 | 1.27 | 10.92 | 0.257 | 0.941 | 0.94 | 0.818 | 0 | 0 | 5 |
| insertion | 2 | reference | 0 3 4 5 | 3.862 | 4.897 | 3.367 | 21.94 | 42.42 | 4.685 | 5.691 | 0.74 | 1.210 | 1 | 0 | 16 |
| insertion | 2 | qp | 0 3 4 5 | 3.685 | 4.342 | 1.313 | 16.94 | 16.98 | 4.943 | 5.874 | 0.78 | 1.060 | 1 | 0 | 16 |
| insertion | 2 | qp_trust | 0 3 4 5 | 3.683 | 4.341 | 0.869 | 16.91 | 16.95 | 4.981 | 5.912 | 0.78 | 0.879 | 1 | 2 | 12 |
| insertion | 2 | qp_tip0.2_f1 | 0 3 4 5 | 6.132 | 7.539 | 0.803 | 15.90 | 24.16 | 0.438 | 0.944 | 0.51 | 1.041 | 0 | 0 | 86 |
| insertion | 2 | qp_soft_tip0.2_f1 | 0 3 4 5 | 6.087 | 7.845 | 0.820 | 15.90 | 24.16 | 0.380 | 1.260 | 0.33 | 1.060 | 0 | 0 | 85 |
| rotation | 0 | reference | 0 | 0.109 | 0.318 | 0.013 | 0.48 | 1.32 | 0.118 | 0.290 | 0.90 | 0.647 | 0 | 0 | 0 |
| rotation | 0 | qp | 0 | 0.116 | 0.328 | 0.014 | 0.48 | 1.32 | 0.126 | 0.305 | 0.88 | 1.511 | 0 | 0 | 0 |
| rotation | 0 | qp_tip0.2_f1 | 0 | 0.138 | 0.517 | 0.023 | 0.30 | 1.91 | 0.033 | 0.283 | 0.90 | 1.168 | 0 | 0 | 0 |
| rotation | 0 | qp_soft_tip0.2_f1 | 0 | 0.082 | 0.217 | 0.011 | 0.30 | 0.39 | 0.012 | 0.045 | 0.93 | 1.097 | 0 | 0 | 0 |
| rotation | 0.5 | reference | 0 3 4 5 | 1.663 | 4.632 | 0.693 | 12.67 | 14.49 | 1.975 | 4.132 | 0.55 | 0.859 | 0 | 0 | 132 |
| rotation | 0.5 | qp | 0 3 4 5 | 1.204 | 3.770 | 0.280 | 5.60 | 14.25 | 1.533 | 3.785 | 0.59 | 0.918 | 0 | 0 | 55 |
| rotation | 0.5 | qp_tip0.2_f1 | 0 3 4 5 | 1.059 | 4.869 | 0.176 | 1.99 | 13.56 | 0.300 | 2.896 | 0.51 | 0.905 | 0 | 0 | 86 |
| rotation | 0.5 | qp_soft_tip0.2_f1 | 0 3 4 5 | 0.171 | 0.775 | 0.139 | 0.54 | 2.30 | 0.146 | 0.588 | 0.62 | 0.855 | 0 | 0 | 3 |
| rotation | 1 | reference | 0 3 4 5 | 3.639 | 6.852 | 3.970 | 21.79 | 22.90 | 4.805 | 7.324 | 0.50 | 0.942 | 1 | 0 | 143 |
| rotation | 1 | qp | 0 3 4 5 | 2.159 | 5.552 | 0.418 | 6.79 | 15.44 | 2.941 | 6.001 | 0.56 | 1.001 | 0 | 0 | 25 |
| rotation | 1 | qp_trust | 0 3 4 5 | 2.159 | 5.552 | 0.418 | 6.79 | 15.44 | 2.941 | 6.001 | 0.56 | 0.823 | 0 | 0 | 25 |
| rotation | 1 | qp_tip0.2_f1 | 0 3 4 5 | 2.358 | 6.557 | 0.378 | 4.69 | 13.92 | 0.539 | 4.042 | 0.52 | 0.958 | 0 | 0 | 78 |
| rotation | 1 | qp_soft_tip0.2_f1 | 0 3 4 5 | 0.453 | 1.368 | 0.292 | 1.04 | 3.82 | 0.276 | 0.737 | 0.56 | 0.915 | 0 | 0 | 3 |
| rotation | 2 | reference | 0 3 4 5 | 11.574 | 10.268 | 11.089 | 176.24 | 38.69 | 14.249 | 12.692 | 0.49 | 1.211 | 2 | 0 | 155 |
| rotation | 2 | qp | 0 3 4 5 | 3.772 | 8.038 | 4.757 | 11.93 | 17.70 | 5.385 | 9.774 | 0.51 | 1.177 | 1 | 0 | 17 |
| rotation | 2 | qp_trust | 0 3 4 5 | 3.770 | 8.039 | 1.131 | 11.93 | 17.70 | 5.383 | 9.767 | 0.51 | 0.880 | 1 | 1 | 16 |
| rotation | 2 | qp_tip0.2_f1 | 0 3 4 5 | 11.370 | 14.320 | 5.733 | 18.93 | 24.16 | 0.978 | 3.898 | 0.39 | 1.132 | 0 | 0 | 384 |
| rotation | 2 | qp_soft_tip0.2_f1 | 0 3 4 5 | 7.553 | 10.694 | 1.349 | 18.00 | 24.16 | 0.635 | 1.343 | 0.59 | 1.051 | 0 | 0 | 106 |
| varying | 0 | reference | 0 | 0.010 | 0.012 | 0.012 | 0.09 | 0.12 | 0.009 | 0.016 | - | 3.393 | 0 | 0 | 0 |
| varying | 0 | qp | 0 | 0.010 | 0.012 | 0.012 | 0.09 | 0.12 | 0.009 | 0.016 | - | 1.215 | 0 | 0 | 0 |
| varying | 0 | qp_tip0.2_f1 | 0 | 0.343 | 0.358 | 0.012 | 1.28 | 1.33 | 0.104 | 0.109 | - | 0.827 | 0 | 0 | 0 |
| varying | 0 | qp_soft_tip0.2_f1 | 0 | 0.340 | 0.676 | 0.013 | 1.26 | 8.17 | 0.112 | 0.830 | - | 0.876 | 0 | 0 | 0 |
| varying | 0.5 | reference | 0 3 4 5 | 0.978 | 1.152 | 0.134 | 4.23 | 4.56 | 1.269 | 1.489 | - | 1.321 | 0 | 0 | 5 |
| varying | 0.5 | qp | 0 3 4 5 | 0.954 | 1.109 | 0.133 | 4.23 | 4.24 | 1.279 | 1.498 | - | 0.964 | 0 | 0 | 4 |
| varying | 0.5 | qp_tip0.2_f1 | 0 3 4 5 | 0.373 | 0.403 | 0.115 | 1.46 | 1.55 | 0.175 | 0.227 | - | 0.957 | 0 | 0 | 4 |
| varying | 0.5 | qp_soft_tip0.2_f1 | 0 3 4 5 | 0.369 | 0.673 | 0.137 | 1.45 | 8.54 | 0.180 | 0.810 | - | 0.859 | 0 | 0 | 4 |
| varying | 1 | reference | 0 3 4 5 | 1.843 | 2.161 | 0.290 | 8.46 | 10.39 | 2.360 | 2.773 | - | 1.205 | 0 | 0 | 4 |
| varying | 1 | qp | 0 3 4 5 | 1.849 | 2.158 | 0.288 | 8.46 | 10.39 | 2.480 | 2.900 | - | 0.943 | 0 | 0 | 4 |
| varying | 1 | qp_trust | 0 3 4 5 | 1.849 | 2.158 | 0.288 | 8.46 | 10.39 | 2.480 | 2.900 | - | 0.733 | 0 | 0 | 4 |
| varying | 1 | qp_tip0.2_f1 | 0 3 4 5 | 0.650 | 0.714 | 0.232 | 1.62 | 1.97 | 0.270 | 0.383 | - | 0.986 | 0 | 0 | 5 |
| varying | 1 | qp_soft_tip0.2_f1 | 0 3 4 5 | 0.604 | 1.459 | 0.275 | 1.51 | 10.79 | 0.276 | 0.909 | - | 0.835 | 0 | 0 | 5 |
| varying | 2 | reference | 0 3 4 5 | 3.765 | 4.432 | 1.331 | 16.96 | 16.93 | 4.802 | 5.648 | - | 1.168 | 1 | 0 | 14 |
| varying | 2 | qp | 0 3 4 5 | 3.750 | 4.380 | 3.797 | 16.96 | 16.92 | 5.047 | 5.943 | - | 0.941 | 1 | 0 | 15 |
| varying | 2 | qp_trust | 0 3 4 5 | 3.753 | 4.382 | 0.908 | 16.93 | 16.90 | 5.055 | 5.952 | - | 0.746 | 1 | 1 | 13 |
| varying | 2 | qp_tip0.2_f1 | 0 3 4 5 | 5.889 | 7.152 | 0.798 | 14.90 | 22.84 | 0.438 | 0.905 | - | 0.974 | 0 | 0 | 80 |
| varying | 2 | qp_soft_tip0.2_f1 | 0 3 4 5 | 5.840 | 7.466 | 0.814 | 14.90 | 22.84 | 0.386 | 1.206 | - | 1.077 | 0 | 0 | 79 |

Seeds 0, 3, 4, 5 for 0.5/1/2 mm; seed 0 only for 0 mm (noise-free inputs are seed independent). `qp_trust` was run at 1 and 2 mm only; at 0/0.5 mm no BVP state is rejected, so it equals `qp`. Jitter is the RMS of frame-to-frame changes of the force error. Mode accuracy compares the selected label with the reference label (`-` for the varying-tip reference, which has no stored labels). "BVP rejected" counts states withheld from history by `bvpTrust`; forces are unchanged by the fixed-load BVP, so rejection changes shape and history, not the reported force of that frame. The reference method diverged at 2 mm rotation seed 5 (frames 44-52: shape errors 80-104 mm and 176 N tip error with converged BVPs) and returned a non-finite state at frame 48, handled as a failed frame (Section 9).

Each step, holding everything else fixed:

1. `reference -> qp` (branch optimizer only): identical at 0 mm and for 0.5/1 mm insertion; rotation tip/contact 1.66/4.63 -> 1.20/3.77 N at 0.5 mm and 3.64/6.85 -> 2.16/5.55 N at 1 mm; 2 mm rotation max tip error 176 -> 11.9 N.
2. `qp -> qp_trust`: forces identical; shape RMS at 2 mm 1.31 -> 0.87 mm (insertion), 4.76 -> 1.13 mm (rotation), 3.80 -> 0.91 mm (varying).
3. `qp -> qp_tip0.2_f1` (existing prior, exact history): best insertion contact force at 0.5/1 mm (0.30/0.70 N), but rotation contact is not improved (4.87/6.56 N) and FBG exceedances rise.
4. `qp_tip0.2_f1 -> qp_soft_tip0.2_f1` (uncertain history): rotation contact 4.87 -> 0.78 N (0.5 mm) and 6.56 -> 1.37 N (1 mm); insertion contact gets worse around reversal; both prior variants fail at 2 mm.

## 8. Speed

Solo run (no other process), one computational thread, seed 0, 1 mm noise, same machine; seconds per frame, means unless stated:

| Config | Scenario | Stage 1 | Linearize | Branches | Audit IVP | BVP | Final IVP | Total mean | Median | p95 | Max | Tip / contact RMSE N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| reference | insertion | 0.241 | 0.020 | 0.467 | 0.019 | 0.075 | 0.019 | 0.852 | 0.492 | 1.951 | 4.746 | 1.821 / 2.066 |
| reference | rotation | 0.244 | 0.022 | 0.136 | 0.023 | 0.089 | 0.022 | 0.547 | 0.493 | 0.856 | 0.941 | 2.801 / 5.576 |
| qp_trust | insertion | 0.227 | 0.018 | 0.061 | 0.018 | 0.071 | 0.018 | 0.421 | 0.423 | 0.496 | 0.701 | 1.835 / 2.064 |
| qp_trust | rotation | 0.226 | 0.020 | 0.080 | 0.020 | 0.082 | 0.020 | 0.456 | 0.447 | 0.535 | 0.592 | 2.330 / 5.477 |
| qp_trust_fast | insertion | 0.089 | 0.018 | 0.080 | 0.018 | 0.069 | 0.017 | 0.299 | 0.298 | 0.378 | 0.440 | 1.835 / 2.064 |
| qp_trust_fast | rotation | 0.096 | 0.020 | 0.110 | 0.021 | 0.083 | 0.021 | 0.359 | 0.347 | 0.446 | 0.556 | 2.330 / 5.477 |
| qp_soft_tip0.2_f1_fast | insertion | 0.091 | 0.018 | 0.104 | 0.018 | 0.070 | 0.018 | 0.327 | 0.326 | 0.446 | 0.511 | 0.459 / 1.305 |
| qp_soft_tip0.2_f1_fast | rotation | 0.094 | 0.020 | 0.123 | 0.021 | 0.081 | 0.020 | 0.367 | 0.354 | 0.474 | 0.552 | 0.463 / 1.539 |

Insertion: 0.852 -> 0.421 s/frame with exact branches (branch stage 0.467 -> 0.061 s) and -> 0.299 s with batched Stage 1 sensitivities; the tail shrinks more than the mean (p95 1.95 -> 0.38 s, max 4.75 -> 0.44 s) because no branch runs to the iteration cap. Rotation: 0.547 -> 0.456 -> 0.359 s (branch stage 0.136 -> 0.080 s here; in the loaded noiseless and 1 mm shadow runs of Section 4 the two angle profiles made the QP branches slower than fmincon's quick convergences, so the rotation branch speedup is case dependent). `qp_trust_fast` reproduces the `qp_trust` trajectory errors exactly, confirming that the batched sensitivities do not change results. Stage 1 (about 0.23 s unbatched) is now the largest single stage; its remaining cost is the finite-difference Jacobian of the geometric residual inside lsqnonlin. Under six concurrent processes all stages are roughly 1.7x slower (Section 7 timings).

## 9. Limitations and open questions

- One fixture (single planar contact, mu=0.3 known, constant base-pose knowledge); multiple simultaneous angle contacts use coordinate descent without a certificate.
- The force prior is a modeling assumption (random walk per frame, frame spacing is bookkeeping). It biases slowly varying tip loads (Section 6). Process stds should be chosen from the expected load dynamics of the application, not from this fixture.
- Uncertain history uses the Stage 1 covariance of the current frame at the contact arc length; it ignores correlation between the previous estimate and current data and is conservative (it does not use mechanics information about the previous shape).
- `modeFeasible`/full-mode-success counts are not comparable for uncertain-history runs, because nonlinear slip rows are no longer equalities.
- Noise model unchanged (iid Bishop channels); no correlated/bias noise, no environment or mu noise.

## 10. Reproduction

```matlab
setup_tsfs; addpath('experiments/inclined_wall','experiments/robust_pass');
% references (fresh stamp), then noise inputs for seeds 0..5
run_fine_bvp_experiment('repro_20261024','truth');   % needs noise_payload.mat copied as in the 2026-10-13 handoff
build_varying_tip_truth(struct('stepMm',.4,'folder',fullfile('results','runs','varying_tip_insertion_truth_repro_20261024')));
run_robust_suite({},0:5,[0 .5 1 2],'prep',{'insertion','rotation','varying'});
% held-out comparison
run_robust_suite({'reference','qp','qp_tip0.2_f1','qp_soft_tip0.2_f1'},[0 3 4 5],[.5 1 2],'final',{'insertion','rotation','varying'});
% identical-input branch comparison (seed 0), after the reference runs exist
compare_branch_solvers('results/runs/robust_inputs_repro_20261024/seed0/insertion_1mm.mat','shadow_s0_insertion_1mm');
```

```text
python experiments/robust_pass/summarize_robust.py results/runs/summary_final final
python experiments/robust_pass/summarize_shadow.py results/runs/summary_final
python experiments/robust_pass/plot_robust.py results/runs/plots_final final 0 1 reference qp qp_soft_tip0.2_f1
```

Published compact outputs: [results/published/robust_pass_20261025/](../../results/published/robust_pass_20261025/).
