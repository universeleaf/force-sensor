# Two-stage force sensing

Our modular research contribution to `universeleaf/force-sensor`. The collaborator's source stays in its existing folders; this folder contains our estimator, wrappers, experiments, evidence, and handoffs. No copied `rod/` or `LCP-Continuum/` algorithms are included here.

## Start

MATLAB R2025b and Optimization Toolbox were used. From this folder:

```matlab
[root, paths] = setup_tsfs;
test_tsfs;
test_spatial_derivatives;
run_baseline('s_channel_two_contact');
run_benchmarks({'sliding_clean','s_channel_two_contact'},2);
```

Default dependency locations are `../rod/` and `../LCP-Continuum/`. The latter remains the original external dependency and is not bundled in this contribution. To use an existing baseline checkout with its dependency already installed, set `TSFS_BASELINE_ROOT` to that checkout, or create the Git-ignored `.local.json`:

```json
{"baselineRoot": "D:/path/to/existing/force-sensor"}
```

On this machine `.local.json` points to the existing collaborator checkout. Its source is read-only; baseline runners redirect outputs and checkpoints to our results folder. No junction is required. Other collaborators should configure their own path rather than copy this local setting. Setup checks actual function resolution; it does not install dependencies or change saved MATLAB paths.

Set `TSFS_RUN_ID` to a simple name to separate runs (default `current`). Generated output goes to `results/runs/<run-id>/`, ignored by Git. `results/published/` holds selected historical evidence and its original provenance. The old standalone workspace is retained as a backup, not used at runtime.

## Entry points

| Purpose | Entry point |
|---|---|
| One inference step / sequence | `tsfs.step` / `tsfs.estimate` |
| Original single-contact baseline | `run_baseline('sliding_clean')` |
| Original planar S-channel baseline | `run_baseline('s_channel_two_contact')` |
| Original general window baseline | `run_baseline('two_contact')` / `run_baseline('three_contact')` |
| Baseline inputs | `prepare_inputs` (recorded), `prepare_inputs('regenerate')` (original builders) |
| Separate profiler run | `run_baseline('s_channel_two_contact',struct('profile',true))` |
| Our benchmark / geometry ablations | `run_benchmarks` / `run_ablations` |
| S-channel diagnosis | `audit_s_channel`, `audit_s_channel_two_axis` |
| Plane sensitivity, then condition analysis | `audit_plane_sensitivity`, `audit_plane_condition` |
| Noiseless sliding / friction ablation / Aloi | `run_single_contact_comparison`, `analyze_single_contact_mu`, `export_single_contact_evidence` |
| Reconstruction-only comparison | `audit_fbg_integration` |
| Historical bottleneck report | `experiments/bottleneck_audit/README.md` |

`options.frames` selects baseline frames for a quick check; omitted means the complete replay. The window baseline requires the complete packet. `valid=true` is not a claim of convergence or physical correctness: inspect solver status and diagnostic flags. Diagnostics remain reporting-only.

## Design and status

- [Governing formulation](docs/formulation/2_STAGE_FORCE_SENSING.md)
- [Historical implementation plan](docs/formulation/2_STAGE_FORCE_SENSING_IMPLEMENTATION_PLAN.md)
- [FBG bending, material frames, and twist](docs/formulation/FBG_CURVATURE_AND_FRAMES.md)
- [Curvature fitting versus reconstructed shape fitting](docs/formulation/CURVATURE_VS_SHAPE_OBJECTIVE.md)
- [Current decisions and pending ideas](docs/decisions/STATUS.md)
- [Deferred physics](docs/decisions/DEFERRED_WORK.md)
- [Noiseless single-contact comparison and friction sensitivity](docs/reports/SINGLE_CONTACT_MU_COMPARISON.md)
- [Historical benchmark results](docs/reports/IMPLEMENTATION_AND_RESULTS.md)
- [Migration handoff](handoff/2026-10-06_mentor-to-collaborator_folder-migration.md)

This migration changes organization, dependency lookup, and output paths. It does not change the estimator equations, solver settings, reconstruction choice, or measured channels. The specialized planar baseline and generic 3D estimator do not have identical information/constraints; their timings are not equivalent-accuracy speedups.

## Collaboration

Root `AGENTS.md` defines scope. Our work stays here and on our branch; existing algorithms are called directly. Future changes to their source need explicit authorization. Reports in `handoff/` record evidence and decisions without silently replacing the collaborator's implementation.
