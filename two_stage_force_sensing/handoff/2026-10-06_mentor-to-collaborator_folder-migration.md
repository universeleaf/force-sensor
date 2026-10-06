# Folder migration handoff

Date: 2026-10-06. Author: Codex assisting the mentor. Intended recipient: project collaborator. This is a prepared handoff, not an externally sent message.

Branch: `codex/two-stage-force-sensing`. Base revision: `13bdb7d4d65b819229905781e6c4aede1f9ab64a`. This report ships in the migration commit; identify it with `git log -1 -- two_stage_force_sensing/handoff/2026-10-06_mentor-to-collaborator_folder-migration.md`.

## Changes

Our implementation is now under `two_stage_force_sensing/` in a separate checkout of the shared repository. Root AGENTS.md is active. Source modules, baseline callers, experiments, tests, formulations, decisions, reports, and handoffs have distinct locations. All 15 estimator core files are byte-identical to the old workspace. No physics, prior, solver tolerance, or measurement channel was changed.

The collaborator's algorithms remain in their original folders. Our setup resolves those sources directly. The machine-local .local.json points to the existing baseline checkout and is ignored by Git; future users configure their own dependency location. The old standalone workspace and collaborator checkout remain intact. No copied rod/LCP source tree or shadow solver is included in our folder.

Historical compact evidence is retained under results/published. Large full outputs remain local under results/runs and can be regenerated. Historical source paths and original instrumentation labels are preserved. The three duplicated formulation/audit notes now have a canonical copy in our branch; old local originals remain as backup.

The old shadow-copy preparation script and modified frozen-geometry solvers are deliberately not migrated. New baseline calls use unchanged algorithms. Exact unavailable internal counters are not invented; profiling is a separate labelled run. Replaying historical frozen-geometry ablations would require approved upstream hooks.

## Reproduce

From two_stage_force_sensing in MATLAB: `setup_tsfs; test_tsfs; test_spatial_derivatives;`. Configure .local.json or TSFS_BASELINE_ROOT if dependencies are in another existing checkout. See README for all entry points. Outputs stay in results/runs/<TSFS_RUN_ID>.

Migration checks also ran `run_smoke`, `audit_fbg_integration`, first-frame baseline calls for sliding_clean and s_channel_two_contact, a separately profiled planar call, source-hash capture, and both Python report generators. These are migration checks, not a replacement full benchmark. The unchanged single-contact baseline retained its own solver-stage termination warnings; they are preserved in its trace.

## Evidence and limitations

The 12 existing estimator checks and spatial derivative test passed. All 96 reconstruction-comparison rows matched the previous workspace exactly. The original repository status was unchanged. No copied baseline algorithm bodies were found in our package. Actual provenance capture recorded 240 MATLAB source files.

The sliding smoke case returned no diagnostic flags on its first frame. The S-channel smoke case still reports its known iteration limit, penetration, tangency, FBG mismatch, unobserved twist, and one-channel observability warnings. Folder migration does not fix these scientific issues. Detailed checks are in [validation.json](../results/published/migration_20261006/validation.json).

## Decisions and next owners

The user approved this organization and direct reuse of upstream algorithms. Newly documented curvature/frame mathematics and shape-objective alternatives are not new implementation approvals. Sensor-centered reconstruction remains an isolated experiment. The fixed-tip-Fy objective investigation remains deferred by the user.

Mentor: choose the next scientific investigation and approve important new assumptions. Collaborator: review entry points, interface compatibility, and provenance, then decide which modules to adopt. Codex: continue only within this folder and the agreed plan; request approval before changes to other source folders or unplanned scientific decisions.
