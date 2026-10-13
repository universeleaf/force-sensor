# Residual audit handoff: 22.5 and 41.25 degrees

Date: 2026-10-07. Author: Codex assisting the mentor. Direction: mentor to collaborator; prepared locally, not sent. Branch codex/two-stage-force-sensing; base commit b376013. No commit or push.

The mentor requested comparison of both initializations at the two problematic rotation frames, residual-block attribution and whether the outcomes imply multiple MAP solutions. See ../docs/reports/BISHOP_INITIALIZATION_RESIDUAL_AUDIT.md and ../results/published/base_rotation_bishop_audit_20261007/.

Both initializations were compared with identical Stage 1 geometry, observations, force priors and previous mechanical shape. Objective differences are almost entirely the Bishop sensor likelihood. At 22.5 degrees, previous-state initialization returns cost 404.056 (sensor 403.991), versus 16.8138 (sensor 16.8128) from the linear seed. The bad result slides with 0.00956 mm inferred slip; the better result sticks with 0.511 N cone margin. At 41.25 degrees both select sliding, but the linear result has cost 424136 and violates equilibrium/contact tolerances versus 0.133507 for the previous-state seed.

The 22.5-degree convergence flag is too permissive: the objective is scaled by about 2.1e-9, but stationarity uses an absolute 1e-5 threshold. The bad point still admits a substantive descent step when the solver stops. An offline stricter restart (threshold 1e-9, budgets 300/3000, original objective scaling preserved) reduces cost to 394.752 then stagnates with damping 1e4. A direct nonlinear-feasibility retraction gives lower-cost nearby points on the same sliding branch, with implemented constraint residuals below 1e-12. This is numerical evidence that these bad outputs are not local optima of the implemented MAP problem.

At 41.25 degrees a stricter restart from the failed linear output reaches the same low-cost solution as the refined previous-state output in 13 iterations. Restart resets trust radius, damping and merit penalty; do not attribute recovery to more iterations alone. No global uniqueness theorem is claimed.

Production estimator code, tolerances and benchmark results are unchanged. The restart solver is a generated instrumented copy in the ignored audit run folder, changing only explicit seed/objective-scale inputs; analysis entry points are audit_bishop_solutions, audit_bishop_descent and audit_bishop_geometry. Block tables, force vectors, traces and spatial sensor residuals are published. The generated copy is not a second production solver.

Next owner: mentor decides whether to authorize convergence-scaling and globalization improvements. Recommended review: stationarity normalization, damping/trust recovery and merit acceptance near contact feasibility. Keep the Bishop measurement model and two-stage design. More elaborate initialization should not mask inadequate convergence checks. Codex must not silently substitute diagnostic results into the prior benchmark.
