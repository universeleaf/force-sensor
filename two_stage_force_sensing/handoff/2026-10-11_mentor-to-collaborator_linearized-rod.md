# 2026-10-11 mentor to collaborator: successive rod linearization, full friction cone

Author: Codex for mentor. Intended recipient: collaborator; local handoff only, not sent. Branch codex/two-stage-force-sensing, base b376013. No commit or push.

User requested linearizing the rod, keeping the full friction cone, and comparing time/error with the full Cosserat solver. Added an opt-in affine mechanics callback and successive nonlinear correction around the existing fmincon mode solver. The original nonlinear mechanics evaluator and IVP are unchanged by hash, and full nonlinear mechanics remains the default. The new inner sliding problem remains nonlinear; this is not an MIQP, polygon approximation, or C3 implementation.

New reusable modules: linearized_model.m, linearized_evaluate.m, linearized_mode_branch.m, mode_residual.m under matlab/+tsfs. mode_branch.m and mode_solve.m accept the opt-in path; existing behavior is preserved. Our common experiment runner records outer iterations and affine callbacks, and writes MAT checkpoints through a temporary file before replacement.

Both full 41-frame insertion trajectories and both 25-state rotation trajectories were rerun for full and linearized mechanics. Rotation starts from the identical original saved loaded checkpoint; its restored frame is excluded from aggregates. Both solvers propagate their own estimates afterward. All timing includes losing modes.

Main mu=0.3 results: insertion mean time 0.390 -> 0.284 s, mean IVPs 21.95 -> 4.98, tip/contact RMSE 0.004407/0.003746 -> 0.007676/0.009175 N. All 41 fits pass the gate. Rotation mean time 1.536 -> 4.916 s despite IVPs 121.04 -> 16.63; tip/contact RMSE remains approximately 0.09755/0.34507 N, and both methods pass only 10/24 fit gates. Two rejected sliding searches dominate the slowdown. The selected sticking solves are cheap, but their strict-fit rejection triggers expensive alternatives. A separate profile attributes most time to fmincon, not the rod IVP. The final mu=0 rotation frame also hits the outer limit; its feasible but nonconverged return is retained.

The full [results report](../docs/reports/LINEARIZED_ROD_COMPARISON.md) includes all eight comparison groups, physical diagnostics, exact numerical settings, tests, failures and reproduction. The [formulation](../docs/formulation/LINEARIZED_ROD_MODES.md) distinguishes shared initial Jacobians from branch-specific nonlinear corrections and specifies immutable physical history. Published summary/plots/traces: results/published/linearized_modes_20261011_v2/.

A failed preliminary prototype is preserved separately. Its reversal seeding and trust-box bookkeeping were corrected before final runs. No physical assumptions, observation noise, prior terms, friction polygon, or acceptance thresholds were changed to obtain the reported results.

Validation: affine Taylor/derivative tests, circular-friction and mu=0 callbacks, nonlinear rotation/reversal regression tests, all 264 saved states and 1,195 outer iterations audited. Existing baseline estimates reproduce prior outputs. No collaborator folders were modified.

Recommendation / next owner: mentor and collaborator should review this as a useful integration-cost reduction, not a universal solver replacement. Future work could convexify/restructure the inner sliding subproblem or improve mode-search bounds while preserving the full cone; those changes require a separate agreed plan. Do not hide rejected branches or reinterpret strict FBG-gate failures as accepted fits.
