# Handoff: noise-to-force sensitivity audit

- Date/author/direction: 2026-10-12, Codex, mentor to collaborator (saved locally, not sent).
- Branch/base: codex/two-stage-force-sensing, b376013. Existing work preserved; no commit/push.
- Request: explain why calibrated 2 mm geometric noise produces about 3 N tip-load error in insertion and rotation.
- Changes: added two offline experiment diagnostics and `docs/reports/NOISY_FORCE_SENSITIVITY.md`; outputs in `results/published/noisy_force_sensitivity_20261012/`. No estimator/physics/settings changes.
- Method: physical-unit nonlinear IVP Jacobians; eliminate terminal moment, enforce local full-cone mode constraints, compute Gaussian curvature-to-force gain conditional on fixed geometry/history/mode.
- Measured: clean-reference predicted tip RMS 3.599 N insertion / 2.725 N rotation. Observed correct-mode converged RMS 2.592 / 3.410 N; actual stored noise through clean gain predicts 2.688 / 2.331 N on those same subsets. X-error correlations 0.960 / 0.833. Full observed rotation 4.985 N includes a failed-correct-branch frame with 18.238 N error.
- Interpretation: unknown contact/tip compensation explains N-scale uncertainty even with correct modes. X is dominant here; rotation adds Y uncertainty (up to 1.543 N predicted std at 45 degrees). Remaining rotation error includes effects excluded by the fixed-history local analysis; no causal variance partition claimed.
- Validation: 130 saved states, algebraic constraint checks; eight exact nonlinear IVPs, max derivative relative error 1.60e-8. No inverse reruns.
- Reproduce: run `experiments/inclined_wall/audit_noisy_force_sensitivity.py`, then MATLAB `validate_noisy_force_sensitivity` after setup and adding the experiment folder.
- Open work/proposal (mentor decision): if causal attribution is needed, separately replay fixed truth geometry and fixed truth previous-contact history, and/or perform Monte Carlo inverse solves. Do not change the production estimator or add priors without approval.
