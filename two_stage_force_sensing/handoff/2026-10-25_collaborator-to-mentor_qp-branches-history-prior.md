# Research handoff: exact QP branch solves, uncertain slip history, existing force prior

Date: 2026-10-25. Author/direction: collaborator side (Tongyu, prepared with Claude Code) to mentor (Jia Shen). Branch `codex/two-stage-force-sensing`, base `8d6df5a` (this report is committed together with the implementation it describes). Responds to [2026-10-13 current method and next steps](2026-10-13_mentor-to-collaborator_current-method-and-next-steps.md) items A–C and to the proposed discussion in the [fine-step handoff](2026-10-13_mentor-to-collaborator_fine-step-bvp.md).

Full report with all tables: [QP_BRANCHES_HISTORY_PRIOR_RESULTS.md](../docs/reports/QP_BRANCHES_HISTORY_PRIOR_RESULTS.md). Switch definitions: [PROFILED_QP_BRANCHES.md](../docs/formulation/PROFILED_QP_BRANCHES.md).

## Scope and principle

The goal was to stabilize the existing simplified two-stage method, not to add modeling layers. The pipeline (Stage 1 -> one linearization -> exhaustive full-cone modes -> one fixed-load BVP) is unchanged; every change is an opt-in switch whose default reproduces the previous behavior. `mode_branch`, `initialize`, `evaluate`, Stage 1, BVP and the collaborator algorithms are unchanged. An earlier draft that propagated a Laplace posterior covariance was discarded in favor of the existing force-tracking prior.

## Changes

| Switch | Code | What it is |
|---|---|---|
| `cfg.singlePassMethod='profiled-qp'` | `tsfs.qp_branch` (new), 4-line dispatch in `single_linearization_solve` | Same branch problems solved exactly: QP for separation/interior sticking, one-angle profile of QPs for sliding and cone-edge sticking |
| `cfg.forcePriors=true` (+ `o.tipProcessStd`, `o.forceProcessStd`) | prior rows in `qp_branch`, from existing `tsfs.initialize` | The framework's random-walk force prior, previously sliced away |
| `cfg.slipStd='stage1'` | `qp_branch` | Slip-history equations weighted by Stage 1 position covariance J_p C_w J_p' instead of exact |
| `cfg.bvpTrust` | 11 lines in `single_pass_bvp_solve` | Unconverged or fit-degrading BVP state is not propagated; pre-BVP state kept |
| `o.batchedReconstruction=true` | `reconstruct`, `geometry`, `reconstruct_bishop_sensitivity` (new) | Same Stage 1 central differences in one batched pass (speed only) |
| `cfg.forcePriorTangentRatio` | `qp_branch` | Tangential std multiplier of the contact prior; tested, not recommended |

Recommendation after the held-out evaluation:

- **Default proposal: `qp_trust`** = `singlePassMethod='profiled-qp'` + `bvpTrust`. No new modeling assumption; equal or better than the reference everywhere.
- **Optional, needs your decision: `qp_soft_tip0.2_f1`** = default + uncertain history + existing prior (tip 0.2 N, contact 1 N per frame). Large gains at 0.5-1 mm, documented failures at 2 mm, at insertion reversal and for slowly varying tip loads.

Tested and removed: a mode-switch penalty (blocks contact onset; tip absorbs the contact load) and an FBG-acceptance gate on the prior (no fix at 2 mm, loses the 1 mm gain).

## Assumptions that need your review

- The force prior is a modeling assumption (per-frame random walk; frame spacing is bookkeeping). It is the existing prior form; only its use in the current method and the std values are new.
- Uncertain history models the previous contact point error by the current frame's Stage 1 covariance at s_i. This is conservative and ignores correlations.
- `bvpTrust` thresholds (1.5x pre-BVP FBG RMS + 0.25) are heuristic.
- Runner policy (all configurations equally): if a frame throws or returns a non-finite state, the last valid output is kept as history and scored for that frame (as in `tsfs.estimate`). Occurred once, in the reference method.

## Results (measured)

Held-out seeds 0, 3, 4, 5 (tuning used seeds 1, 2 only); every frame scored; RMS tip / contact force error in N:

| Noise | Case | Reference | Recommended default `qp_trust` | Optional `qp_soft_tip0.2_f1` |
|---|---|---|---|---|
| 0.5 mm | insertion | 0.94 / 1.12 | 0.92 / 1.08 | 0.25 / 0.65 |
| 0.5 mm | rotation | 1.66 / 4.63 | 1.20 / 3.77 | 0.17 / 0.78 |
| 1 mm | insertion | 1.79 / 2.12 | 1.79 / 2.11 | 0.54 / 1.49 |
| 1 mm | rotation | 3.64 / 6.85 | 2.16 / 5.55 | 0.45 / 1.37 |
| 2 mm | insertion | 3.86 / 4.90 | 3.68 / 4.34 | 6.09 / 7.85 (fails) |
| 2 mm | rotation | 11.57 / 10.27 | 3.77 / 8.04 | 7.55 / 10.69 (fails) |

(`qp` values at 0.5 mm, where `qp_trust` equals `qp`.)

- Identical-input branch comparison (seed 0, 8 trajectories, 3710 branches): QP lower objective in 230 branches, higher in 1 (fmincon point inside its 1e-6 constraint tolerance), fmincon at the 100-iteration cap in 294 and infeasible-where-QP-feasible in 90. Up to 21/120 rotation frames had a better QP selection; maximum selected force difference 31 N.
- The published 0.5 mm rotation frame 91 spike (12.7 N) is a local-optimizer failure: fmincon's selected objective is 687 units above the QP selection; with the QP branch the frame has 0.07 N tip error.
- `bvpTrust` leaves forces unchanged and reduces 2 mm shape RMS (rotation 4.76 -> 1.13 mm). The reference method diverged once (2 mm rotation, seed 5; 176 N tip error with converged BVPs) and returned a non-finite state.
- Exact slip history is the cause of the remaining rotation contact error once a prior is used (6.7 N vs 1.4 N with uncertain history, tuning seeds).
- Follow-up 2026-10-31, recorded 2-3 contact fixtures (report Section 8b): QP chain better or equal on all except the deferred S-channel first frame, where fmincon's lower objective uses its 1e-6 constraint tolerance (tightened to 1e-9 it is worse than QP). Examples: three contacts tip RMSE 13.0 -> 0.47 N and 9580 -> 22 s/frame; spatial three-channel 3080 -> 0.60 N. The multi-angle search now starts from a joint angle grid; frictionless cone edges are fixed. New regression test `tests/test_qp_branch.m`.
- Speed: branch time 0.69 -> 0.27 s/frame for insertion under equal load; solo timing in the report. Batched Stage 1 sensitivities (`fast`) give the same Stage 1 result to 1e-11 mm and are ~6x faster for that step.

## Failures and limits

- The prior cannot track slow tip-load changes along the weak tip/contact direction (varying-tip reference): bias instead of noise.
- With uncertain history the estimator lags the 13 N friction flip at insertion reversal by several frames and occasionally selects sticking during steady sliding; hard history plus prior is better for steady insertion, worse for rotation.
- Reproduction of the published 0.5 mm rotation, 1 mm insertion and 2 mm cases is not bitwise (recurrent amplification of floating-point differences, R2024a vs R2025b). Seed 0 differences are reported, not hidden.
- Trajectory statistics come from one fixture (single contact); multi-contact evidence is limited to short recorded fixtures. Several simultaneous friction angles use a joint grid plus coordinate descent without a global certificate.

## Reproduction

See report Section 10. Raw trajectories stay under ignored `results/runs/`; compact tables/plots are in `results/published/robust_pass_20261025/`. LCP-Continuum must be present beside `rod/` (a copy, not a junction: the dependency check resolves real paths).

## Proposed next actions

1. Mentor: decide whether uncertain history and the force prior are acceptable assumptions, and which process stds match intended experiments (owner: mentor).
2. Investigate the remaining sliding/sticking ambiguity under uncertain history (e.g. use the previous mechanics estimate's own covariance instead of Stage 1's), keeping it within the existing switches (owner: collaborator).
3. Repeat on a second fixture (two contacts or TDCR) before treating the settings as general (owner: collaborator).
4. Experimental noise model (correlated/bias) before choosing final priors (owner: both).
