# Robust single-pass experiments (2026-10-25)

Paired comparison of the fine-step single-pass estimator with the opt-in switches of [PROFILED_QP_BRANCHES.md](../../docs/formulation/PROFILED_QP_BRANCHES.md). Results and interpretation: [QP_BRANCHES_HISTORY_PRIOR_RESULTS.md](../../docs/reports/QP_BRANCHES_HISTORY_PRIOR_RESULTS.md).

| File | Purpose |
|---|---|
| `robust_configurations.m` | Named configurations (`reference`, `qp`, tokens `trust`, `soft`, `tip<v>`, `f<v>`, `r<v>`, `fast`) |
| `run_robust_trajectory.m` | One recurrent trajectory; same frame logic as `run_inclined_modes`, outputs only under `results/runs/<tag>/`; a frame that throws or returns a non-finite state keeps the last valid output (as `tsfs.estimate`) and is flagged |
| `run_robust_suite.m` | Noise inputs per seed (seed 0 = published payload) and insertion -> rotation (restored seed) -> varying-tip runs |
| `build_varying_tip_truth.m` | Insertion reference with a time-varying applied tip load (copy of `build_inclined_truth` with a tip schedule) |
| `compare_branch_solvers.m` | fmincon SQP vs `qp_branch` on identical inputs along the reference trajectory |
| `summarize_robust.py`, `summarize_shadow.py`, `audit_frames.py`, `make_report_tables.py`, `plot_robust.py` | Analysis (NumPy, Matplotlib) |

Requires `setup_tsfs` and `addpath('experiments/inclined_wall','experiments/robust_pass')`. References come from `run_fine_bvp_experiment(stamp,'truth')`; see report Section 10 for the full command sequence.
