# Inclined-wall bidirectional experiment handoff

Date: 2026-10-08. Author: Codex assisting the mentor. Direction: mentor to collaborator, local report only, not sent. Branch codex/two-stage-force-sensing; base b376013. Changes uncommitted and unpushed.

Approved geometry: 200 mm rod, 80 straight + 120 curved, 90-degree intrinsic bend; 45-degree wall rising toward +X; base translates +Z then reverses; closest perpendicular base-wall distance 40 mm, explicitly confirmed by the user; tip load +X 1 N; no injected noise. New experiment stays under our package. Previous simulations and collaborator algorithms are preserved.

Implemented independent forward-fixture wrapper, current-estimator runner, scientific video/figure publisher, and reproduction instructions in experiments/inclined_wall. Baseline solve_planar_contact_shooting is called read-only, not copied. The reference exploits planar symmetry while inverse unknowns remain full 3D. Existing fmincon relaxed-v2 settings are retained with no force priors and sigma=1e-4 /mm.

Measured: reference completed 41 samples, perpendicular distance 80 -> 40 -> 80 mm by 2 mm steps. Contact migrates s=134.849 -> 100.214 mm, maximum normal force 23.1454 N. Reference physical checks pass. Friction-aware estimator returns all 41: 19 optimality exits, 15 small-step exits, 6 budget exits, 1 infeasible. Mean time 8.889 s/frame, 85.07 iterations, 853.07 augmented IVPs; median tip error 0.002258 N but RMS 3.73488 N because failures remain included. Maximum Stage 1 loaded-contact arc/normal errors 0.04725 mm / 0.001345 degrees.

The unchanged mu=0 attempt aborts at first actual contact, frame 4, tsfs:RHSBudget (100000 RHS evaluations on an optimizer trial); only 3 non-contact estimates exist. Later frames were not attempted. Video labels absent estimates explicitly, rather than holding or substituting estimates. The comparison is incomplete and no loaded-contact superiority claim is supported yet.

[Full report](../docs/reports/INCLINED_WALL_BIDIRECTIONAL_RESULTS.md). [Reproduction](../experiments/inclined_wall/README.md). Published outputs, including MP4, CSVs and settings: results/published/inclined_wall_bidirectional_20261008/unreduced_attempt. Raw MAT and preserved attempt copies: results/runs/inclined_wall_bidirectional_20261008/inclined_wall.

Pending user decision: approve an exact mu=0 algebraic reduction (tau=0, eliminate tangential force and redundant slip/friction constraints) before implementing or rerunning. This avoids the zero-gradient squared-cone representation without changing frictionless physics, but changes the solver formulation. No approval has yet been received; no such changes are present. Keep original failed attempt as evidence if approved. Proposed next owner: mentor decides, then Codex/collaborator implements the isolated reduced adapter and completes all 41 comparison frames. No baseline edits, commit, push, or further solver retuning is authorized.
