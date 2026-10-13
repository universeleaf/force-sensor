# Inclined-wall bidirectional experiment

This separate experiment uses a 200 mm rod, 80 mm straight proximal section and 120 mm curved distal section with intrinsic bend angle 90 degrees (radius 76.394372684 mm). The base translates along world +Z, then reverses; exposed length stays 200 mm. The fixed plane is z-x=113.13708499 mm, with allowed side z-x <= 113.13708499. Its inward contact normal is [1,0,-1]/sqrt(2); positive wall tangent is [1,0,1]/sqrt(2).

The user approved perpendicular base-to-wall distance. Distances are 80: -2: 40, then 42: 2: 80 mm (41 frames). Base travel is 0 to 56.56854249 to 0 mm. Tip load is [1,0,0] N. The same frictional truth (mu=0.3) supplies both estimators; mu=0 is an inference-model ablation, not a different physical simulation. No measurement or environment noise is injected. Estimator covariance remains finite: Bishop sigma=1e-4 /mm and environment position/normal-component sigma=0.03 mm / 0.001.

## Reference and estimator

`build_inclined_truth` calls the existing baseline `solve_planar_contact_shooting` read-only. The forward fixture exploits this experiment's planar symmetry. The inverse estimator retains all 3D force/shape variables. No algorithm is copied from the baseline.

The reference independently enforces equilibrium, contact tangency, nonpenetration, and contact complementarity. A local wrapper chooses the Coulomb branch using the displacement of the same material arclength from the preceding true shape; it verifies dissipative friction work and the disk-projection relation. A sticking branch is available but no sampled frame sticks at the selected 2 mm distance increment. The reversal is therefore not a resolved study of a short sticking transient.

Ideal Bishop measurements are generated at the existing 24 FBG stations. In this planar fixture w1=u2 and w2=0. The forward model and inverse use the same piecewise intrinsic curvature and stiffness. Truth is only used after solving for scoring. Stage 1 uses the existing fast geometry MAP; Stage 2 uses the latest fmincon SQP relaxed adapter, sensor-only cost, epsilon=1e-6, nonlinear feasibility tolerance=1e-6, optimality tolerance=1e-8, step tolerance=1e-12, max 300 iterations / 5000 objective evaluations, and the existing scaling. Diagnostics only report warnings. Failed returned iterates remain recorded and may influence subsequent history according to the existing implementation.

Times spaced 0.02 seconds are history bookkeeping for a quasistatic sequence, not a validated dynamic insertion velocity. Video playback is illustrative, not measured solver speed.

## Reproduction

From the two_stage_force_sensing package in MATLAB, with the original read-only baseline and LCP dependencies configured:

```matlab
setenv('TSFS_RUN_ID','inclined_wall_bidirectional_NEW_RUN');
setup_tsfs;
addpath('experiments/inclined_wall');
build_inclined_truth;
run_inclined_comparison;
```

Use a fresh run ID to preserve prior artifacts. Raw MAT files and intermediate logs are in results/runs. The publisher reads MATLAB v7.3 files with h5py and renders scientific figures/video using matplotlib and imageio_ffmpeg:

```text
python experiments/inclined_wall/publish_inclined.py --run-id inclined_wall_bidirectional_NEW_RUN
```

The original mu=0 attempt on 2026-10-08 failed at frame 4 (first actual contact) with tsfs:RHSBudget. Its first 3 frames, the full mu=0.3 trajectory, and the crash log are preserved. Do not interpret a missing mu=0 frame as a zero error or a valid estimate. An exact mu=0 elimination of tangential force and redundant friction variables was proposed for user approval; it is not part of the unchanged formulation.

## Exact integer-mode follow-up (2026-10-10)

The user subsequently authorized explicit contact modes. The separate runner run_inclined_modes uses exhaustive separation/sticking/sliding enumeration and retains full nonlinear mechanics. Mu=0 eliminates tangential force exactly. Previous mode is tried first; all attempted modes are included in runtime. See [formulation and reproduction](../../docs/formulation/INTEGER_MODE_MECHANICS.md) and [full results](../../docs/reports/INCLINED_WALL_INTEGER_MODES_RESULTS.md). This follow-up preserves the original relaxed experiment and its mu=0 crash.

Conditional follow-up: run_inclined_modes_conditional(Inf,'inclined_wall_modes_conditional_NEW_RUN',0.01) uses the user-approved whitened FBG RMS 0.01 (raw 1e-6 /mm). See [results and acceptance/fallback semantics](../../docs/reports/INCLINED_WALL_CONDITIONAL_MODES_RESULTS.md). Original exhaustive outputs are preserved.

## Loaded-state axial rotation follow-up (2026-10-10)

The separate save_inclined_rotation_seed, build_inclined_rotation_truth and run_inclined_rotation_modes wrappers start from insertion frame 21 and run 0 -> 45 -> 0 degrees at fixed 40 mm perpendicular distance. The forward fixture calls the existing full 3D collaborator map read-only. publish_force_video.py adds XYZ force plots for either insertion or rotation; analyze_inclined_rotation.py audits checkpoint continuity and all search costs. See [full configuration, reproduction and failures](../../docs/reports/INCLINED_WALL_ROTATION_RESULTS.md). These wrappers use the dated rotation tag; change it before a new experiment to preserve outputs.

## Rod-linearization comparison (2026-10-11)

run_linearized_comparison('NEW_STAMP') replays insertion and rotation with full and successive-linearized mechanics for mu=0.3 and mu=0, using the same full circular friction and thresholds. Analyze with analyze_linearized_comparison.py NEW_STAMP and export_linearized_trace.py NEW_STAMP. [Results, failures, exact settings and outputs](../../docs/reports/LINEARIZED_ROD_COMPARISON.md). Final benchmark tag suffix is 20261011_v2; preserve the preliminary unsuffixed failure evidence.

Noise experiments: calibrate_noisy_modes.py NEW_STAMP, then run_noisy_linearized('NEW_STAMP'), then analyze_noisy_linearized.py NEW_STAMP. Preserves the full circular law; uses calibrated Gaussian Bishop noise and normalized gate 1.24. [Results and reproduction](../../docs/reports/NOISY_LINEARIZED_ROD_RESULTS.md).

Exhaustive noisy follow-up: run_noisy_exhaustive('NEW_STAMP') reuses the 20261011 noisy packets and compares all modes for mu=0.3 without changing limits. Analyze with analyze_noisy_mode_search.py NEW_STAMP. [Full results, cycle audit and budget recommendations](../../docs/reports/NOISY_EXHAUSTIVE_MODE_AUDIT.md). Counterfactual budget savings are trace estimates, not guarded solver reruns.


## Single-pass rod and octagonal QP comparison (2026-10-12)

All 16 trajectories at 0/0.5/1/2 mm noise are complete. See [full results and physical consistency limits](../../docs/reports/SINGLE_PASS_ROD_POLYGON_COMPARISON.md) and [formulation](../../docs/formulation/SINGLE_PASS_POLYGON_QP.md). Final stamp: 20261012_unbounded; preserve earlier bounded trials separately.

Run run_single_pass_comparison('FRESH_STAMP',{'direct','polygon'}) for the requested two methods. Optional zero-noise exhaustive reference: run_single_pass_comparison('FRESH_STAMP',{'current'},0). Then analyze_single_pass_comparison.py FRESH_STAMP and publish_single_pass_report.py FRESH_STAMP. Existing 2 mm exhaustive references are reused. The default runner also requests optional slow 0.5/1 mm controls; these were not completed in the final approved reporting scope. Native affine convergence is not nonlinear mechanical feasibility; the final IVP is diagnostic only.


## Fixed-load BVP correction (2026-10-12)

The separate single_pass_bvp_solve wrapper exhausts full circular modes, then calls bvp4c once with all forces fixed. See [results and failures](../../docs/reports/SINGLE_PASS_FIXED_LOAD_BVP_RESULTS.md) and [BVP formulation](../../docs/formulation/FIXED_LOAD_BVP_CORRECTION.md). Terminal moment is corrected; wall gap and friction consistency can worsen. Outputs preserve both the pre-correction and corrected states.

Run run_single_pass_bvp('FRESH_STAMP'), then analyze_single_pass_bvp.py FRESH_STAMP. Final benchmark stamp: 20261012_fixedload_v2. Prior single-pass and preliminary BVP results are preserved. All eight trajectories (260 new solves) are complete; three selected SQP branches hit the 100-iteration cap and remain marked unconverged. Every BVP converged; each corrected frame uses three mechanics IVPs plus BVP collocation work.

Five-times-finer actuation (2026-10-13): run run_fine_bvp_experiment('20261012_step5'), then analyze_fine_bvp.py 20261012_step5. Insertion 0.4 mm; rotation 0.75 degrees; noise targets 0/0.5/1/2 mm. See [results including failed corrections](../../docs/reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md). Existing completed outputs are skipped; fresh stamps need the preserved noise payload and protocol copied to their input folder.

Fine-step videos: python experiments/inclined_wall/publish_fine_bvp_media.py, then python experiments/inclined_wall/validate_fine_bvp_media.py. Both default to 20261012_step5. Rendered shapes.npz sources are published so rendering can be repeated without local MAT checkpoints. The exact Gaussian noise payload is in results/published/fine_bvp_comparison_20261012_step5/reproduction; copy into the corresponding results/runs/fine_bvp_inputs_STAMP folder before a solver rerun.
