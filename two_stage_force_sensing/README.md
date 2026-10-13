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
| Axial base rotation / friction and twist | `build_rotation_truth`, `run_rotation_comparison`, `analyze_rotation` |
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
- [Base-rotation comparison, twist and friction sensitivity](docs/reports/BASE_ROTATION_RESULTS.md)
- [Noiseless single-contact comparison and friction sensitivity](docs/reports/SINGLE_CONTACT_MU_COMPARISON.md)
- [Historical benchmark results](docs/reports/IMPLEMENTATION_AND_RESULTS.md)
- [Migration handoff](handoff/2026-10-06_mentor-to-collaborator_folder-migration.md)

This migration changes organization, dependency lookup, and output paths. It does not change the estimator equations, solver settings, reconstruction choice, or measured channels. The specialized planar baseline and generic 3D estimator do not have identical information/constraints; their timings are not equivalent-accuracy speedups.

## Collaboration

Root `AGENTS.md` defines scope. Our work stays here and on our branch; existing algorithms are called directly. Future changes to their source need explicit authorization. Reports in `handoff/` record evidence and decisions without silently replacing the collaborator's implementation.

Bishop geometric sensor rotation update (2026-10-07): [results](docs/reports/BASE_ROTATION_BISHOP_RESULTS.md), [observation contract](docs/formulation/BISHOP_OBSERVATION_CONTRACT.md), and [handoff](handoff/2026-10-07_mentor-to-collaborator_bishop-rotation.md). Previous material-observation experiments are preserved.

Direct-relaxation solver comparison (2026-10-07): [all results](docs/reports/RELAXED_ROTATION_COMPARISON.md), [solver contract](docs/formulation/RELAXED_MECHANICS_SOLVERS.md), and [reproduction](experiments/base_rotation/RELAXED_COMPARISON.md). The default solver is unchanged.

Follow-up: [objective and constraint scale audit](docs/reports/RELAXED_SCALING_AUDIT.md), with numerical evidence from all saved initial and final states; solver settings unchanged.

Revised relaxation comparison (2026-10-07): [all four reruns and Jacobian audit](docs/reports/RELAXED_ROTATION_V2_RESULTS.md), [configuration](experiments/base_rotation/RELAXED_COMPARISON_V2.md), and [handoff](handoff/2026-10-07_mentor-to-collaborator_relaxed-v2.md). Force-prior penalties are disabled in this experiment; older/default configurations are preserved.

Directional force audit: [rotation versus planar sliding](docs/reports/ROTATION_DIRECTION_SENSITIVITY.md), with matched Bishop derivatives, contact-branch sensitivities and measured XYZ errors.

Nonlinear coupling follow-up: [tip/contact load transfer and close shapes](docs/reports/LOAD_TRANSFER_SHAPE_COUPLING.md), including physical validity and FBG distinguishability checks.

Inclined-wall experiment (2026-10-08): [results and video](docs/reports/INCLINED_WALL_BIDIRECTIONAL_RESULTS.md), [reproduction](experiments/inclined_wall/README.md), and [handoff](handoff/2026-10-08_mentor-to-collaborator_inclined-wall.md). The unchanged mu=0 run crashed at first contact; missing estimates are explicitly marked.

Inclined-wall follow-up: [why solving remains difficult](docs/reports/INCLINED_WALL_DIFFICULTY_AUDIT.md), including unchanged-problem initialization tests and logged mu=0 failure. Production formulation and original results are unchanged.

Explicit integer contact modes (2026-10-10): [full inclined-wall results](docs/reports/INCLINED_WALL_INTEGER_MODES_RESULTS.md), [formulation and solver settings](docs/formulation/INTEGER_MODE_MECHANICS.md), and [handoff](handoff/2026-10-10_mentor-to-collaborator_integer-modes.md). Both 41-frame trajectories complete; all competing-mode costs and failures are retained. Default solver unchanged.

Conditional mode search (2026-10-10): [full timing/accuracy comparison](docs/reports/INCLINED_WALL_CONDITIONAL_MODES_RESULTS.md). With the user-selected raw FBG RMS gate of 1e-6 /mm, mu=0.3 retains identical estimates while reducing total time 13.94-fold. Mu=0 cannot meet the fit gate on 33 frames; flagged fallbacks remain saved.

Loaded-state rotation follow-up: [0-45-0 degree inclined-wall rotation and force-component videos](docs/reports/INCLINED_WALL_ROTATION_RESULTS.md). Starts from saved maximum-insertion frame 21, preserves each estimator's own history, and reports strict FBG-gate failures.

Successive rod linearization: [full-cone formulation](docs/formulation/LINEARIZED_ROD_MODES.md) and [matched insertion/rotation comparison](docs/reports/LINEARIZED_ROD_COMPARISON.md). Optional implementation reduces IVPs; insertion is faster, but difficult rejected modes make rotation slower. Full nonlinear mechanics remains the default.

Gaussian FBG noise follow-up: [2 mm and 5 mm integrated tip-noise experiments](docs/reports/NOISY_LINEARIZED_ROD_RESULTS.md). Full insertion/rotation trajectories show frequent wrong-mode acceptance and long SQP outliers despite mostly converged feasible solutions.

2026-10-11: [Noisy exhaustive-mode and iteration-budget audit](docs/reports/NOISY_EXHAUSTIVE_MODE_AUDIT.md) compares all four recurrent trajectories, reports residual force errors and identifies repeated subproblems. No solver-limit change.

Five-times-finer actuation (2026-10-13): [accuracy, convergence and failure comparison](docs/reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md). Same one-pass rod plus fixed-load BVP, all four noise levels and both motions; no outer correction loop.

Fine-step media (2026-10-13): [all eight videos and diagnostic plots](results/published/fine_bvp_comparison_20261012_step5/media/README.md). Includes failed frames; compact rendering inputs and exact noise payload are published.
