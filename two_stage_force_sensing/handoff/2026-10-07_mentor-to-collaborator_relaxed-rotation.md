# Direct-relaxation rotation handoff

Date: 2026-10-07. Author: Codex assisting the mentor. Direction: mentor to collaborator; prepared locally, not sent. Branch `codex/two-stage-force-sensing`, base commit `b376013`. No commit or push.

## Authorized scope and implementation

The mentor requested direct epsilon=1e-6, comparing MATLAB fmincon SQP and IPOPT at 3.75-degree and 1-degree base-rotation increments. Implemented separately without replacing `tsfs.step` or the custom SQP. All changes are in `two_stage_force_sensing/`; collaborator sources are read-only.

Added generic `tsfs.relaxed_solve`, `relaxed_constraints`, and `RelaxedCallback`. Normal contact uses scalar Scholtes relaxation. Circular friction uses two smooth squared cones and a relaxed conic duality gap with one auxiliary slip-norm variable/contact. This preserves the exact zero-relaxation physics and permits nonzero slip at zero force without dividing by either. See `docs/formulation/RELAXED_MECHANICS_SOLVERS.md` for exact equations, units and degeneracy caveats. No physical assumptions, Bishop sensor model, geometry stage, priors, or diagnostic feedback policy were changed.

Both solvers use the same analytic augmented-IVP derivatives and fixed objective multiplier 1e-4. Targets: constraint 1e-9, optimality 1e-8, 300 iterations; fmincon step tolerance 1e-12; IPOPT limited-memory Hessian and no additional NLP scaling. The production custom objective scaling is not reused. Full solver settings/provenance are published.

CasADi 3.8.0 with IPOPT 3.14.19 was downloaded from its official release into ignored package-local storage. License notices are retained. No global installation or MATLAB path persistence. Runtime MATLAB R2025b.

## Evidence, including failures

All 118 frames completed and remain in results. Tip RMSE / mean seconds per frame / mean iterations / mean augmented IVPs:

- IPOPT, 3.75 degrees: 0.00435 N / 10.39 s / 166.3 / 742.2.
- fmincon, 3.75 degrees: 1.148 N / 9.28 s / 101.4 / 735.0.
- IPOPT, 1 degree: 1.233 N / 11.84 s / 184.5 / 863.4.
- fmincon, 1 degree: 0.0637 N / 2.17 s / 39.2 / 171.5.

Native success/positive flags: 5/13, 12/13, 23/46, 46/46 respectively. These are not equivalent convergence certifications: coarse fmincon has eleven small-step exits and only one first-order exit; all fine fmincon exits are small-step. IPOPT also returns acceptable-level outcomes and failed-step/iteration-limit outcomes. Raw statuses, objectives and optimality metrics are preserved.

Coarse fmincon fails severely at 33.75 degrees after an earlier 26.25-degree outlier. Fine fmincon has a 0.432 N outlier at 27 degrees. Fine IPOPT has a 0.619 N outlier at 24 degrees and a failure chain from 41 degrees onward, reaching 4.74 N tip error. The existing recurrence keeps finite failed states as history/priors; numerical state initialization only reuses native-success states. No silent restarts or filtering were introduced.

Original projection diagnostics can exceed 1e-6 N even where relaxed constraints pass. Epsilon bounds a work-like friction duality gap, not force error; its angular implication depends on slip magnitude. The late fine IPOPT failures additionally violate equilibrium and the friction cone. Diagnostics remain reporting-only. Stage 1 maximum arclength/normal errors are 0.00150 mm / 0.00233 degrees (coarse), 0.00166 mm / 0.00287 degrees (fine), identical between solvers.

Validation: analytic constraint Jacobian checks passed at three rotation angles; the zero-force/nonzero-slip cone case passed. The regenerated 46-frame independent reference passed every physical audit. Both step sizes retain 0.02 s sampling and identical per-step priors; they are different discrete friction trajectories, not equal-speed replays. Benchmarks were serial, one timing pass, with checkpoint I/O excluded.

## Files and reproduction

- Report: `docs/reports/RELAXED_ROTATION_COMPARISON.md` (all frames, status/physics tables, plots).
- Procedure: `experiments/base_rotation/RELAXED_COMPARISON.md`.
- Published data: `results/published/base_rotation_relaxed_375deg_20261007`, `base_rotation_relaxed_1deg_20261007`, `base_rotation_relaxed_comparison_20261007`.
- Raw snapshots/logs and dependency archive: ignored `results/runs/`.
- Entry points: `prepare_relaxed_rotation`, `run_relaxed_rotation`, `export_relaxed_rotation`, `test_relaxed`; Python summary/report scripts alongside.

## Interpretation and next owner

Direct 1e-6 is not uniformly robust with this formulation/scaling. Smaller increments help fmincon here but worsen IPOPT overall; positive solver exits are insufficient. Do not replace the default estimator on this evidence. Proposed next controlled work: inspect sticking-cone and optimality scaling, diagnose the outlier frames at fixed history, then test a short 1e-4 -> 1e-6 continuation. These results do not prove four continuation stages are needed. Any changes to physical assumptions or history acceptance policy need the mentor's approval. Mentor owns selection of the next test; collaborator can reproduce these modular experiments independently.
