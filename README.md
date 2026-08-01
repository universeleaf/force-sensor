# Continuum-Robot Force Sensing

This repository contains MATLAB simulations for estimating external forces on
a continuum robot from sparse shape measurements. The current rod-plane study
compares two inverse methods against a displacement-aware frictional contact
simulation:

1. the constrained EKF/MAP formulation in `Formulation.pdf`; and
2. a shape-only Gaussian-load baseline based on Aloi et al. and `force.m`.

The upstream `LCP-Continuum/` working tree is used as a dependency and has not
been modified.

## New Rod-Plane Work

All current rod-plane scripts and notes are now grouped under
`rod_plane_force_sensing/`. Add that folder to the MATLAB path before running:

```matlab
addpath(fullfile(pwd, 'rod_plane_force_sensing'));
```

The main additions are:

- `rod_plane_force_sensing/simu_rod_plane_displacement_force_sensing.m`
  - Defines the current push-then-slide scenario.
  - Uses a 150 mm rod, zero true tip load, and `mu = 0.5` during lateral
    motion.
  - Solves 18 physical force/shape frames and renders two 60-frame videos.
- `rod_plane_force_sensing/run_rod_plane_force_sensing_experiment.m`
  - Contains the copied displacement-aware forward contact loop.
  - Generates sparse FBG-like measurements.
  - Solves the constrained EKF/MAP inverse problem.
  - Fits the Aloi Gaussian position baseline.
  - Writes MAT, CSV, text, figures, and video.
- `rod_plane_force_sensing/rerun_aloi_saved_result.m`
  - Recomputes only the corrected Aloi baseline from an existing result MAT.
  - Avoids rerunning the forward LCP and constrained formulation.
- `rod_plane_force_sensing/validate_aloi_gaussian_baseline.m`
  - Checks the Aloi implementation on a known in-domain transverse Gaussian
    load before it is used as a contact-scenario comparison.
- `rod_plane_force_sensing/validate_rod_plane_displacement_forward.m`
  - Checks the copied forward solve for finite shapes, body contact, friction,
    Coulomb-cone feasibility, and nonlinear formulation residuals.
- `rod_plane_force_sensing/validate_rod_plane_displacement_results.m`
  - Checks inverse force feasibility, all three complementarity products,
    shape reconstruction, and forward-truth consistency.
- `rod_plane_force_sensing/validate_rod_plane_displacement_inverse.m`
  - Runs the strict six-frame `fmincon` regression used before the formal run.
- `rod_plane_force_sensing/ROD_PLANE_FORCE_SENSING_EXPERIMENT.md`
  - Gives the method-to-equation mapping and the latest numerical results.
- `rod_plane_force_sensing/PROJECT_SUMMARY.md`
  - Records implementation and debugging details for future work.

## Run

A formal run requires Optimization Toolbox because the MAP subproblem is
solved with `fmincon`:

```matlab
results = simu_rod_plane_displacement_force_sensing(false);
```

A shorter pipeline smoke test uses the approximate projected solver:

```matlab
results = simu_rod_plane_displacement_force_sensing(true);
```

The quick mode is useful for checking data flow, figures, and video. Its force
error is not a result of the strict formulation and should not be reported as
one.

Forward-only validation:

```matlab
report = validate_rod_plane_displacement_forward();
```

The low-level `run_rod_plane_force_sensing_experiment(false)` command now uses
the same physical defaults as the formal displacement wrapper. It no longer
runs the older 180 mm/nonzero-tip-load scenario shown in previous outputs.

Senior-video geometry diagnostic:

```matlab
results = simu_rod_plane_senior_geometry_force_sensing(false);
```

That geometry retains contact at the rod tip and is kept as an
identifiability diagnostic, not as the reportable body-contact result.

To recompute the corrected Aloi comparison from the saved senior result:

```matlab
analysis = rerun_aloi_saved_result([], 'all');
```

To verify the baseline inside the load model assumed by Aloi et al.:

```matlab
report = validate_aloi_gaussian_baseline();
```

## Current Scenario

- Rod length: `150 mm`
- Integrated intrinsic precurvature: `88.49 deg` (scaling disabled)
- Plane point: `[20, 0, 0] mm`
- Plane normal: `[-1, 0, 0]`
- Push: `45 mm`, internal step at most `0.1 mm`, `mu = 0`
- Lateral command: `1 mm`, internal step at most `0.02 mm`, `mu = 0.5`
- True tip load: `[0, 0, 0] N`
- Output frames: `18`
- Sparse shape points: `24`
- Injected sensing noise: none
- Force upper bounds: disabled

The lateral phase is a commanded base displacement. In this run the contact
remains inside the friction cone and is classified as sticking by the inverse
constraints; the phase name `slide` does not imply gross slip at every frame.

## Latest Formal Result

The values below were regenerated on 2026-07-23 with the command shown above.
All validation assertions passed.

```text
Final true contact force:       [-33.9467,  0.0000, -8.0862] N
Final estimated contact force:  [-33.8324, -0.0081, -8.1451] N

Final true tip load:            [  0.0000,  0.0000,  0.0000] N
Final estimated tip load:       [ -0.1238,  0.0069,  0.0668] N

Final true total load:          [-33.9467,  0.0000, -8.0862] N
Final estimated total load:     [-33.9563, -0.0011, -8.0783] N
Legacy one-pass Aloi load:      [-25.4481,  0.0000,  6.3671] N
```

```text
Constrained EKF/MAP contact-force RMSE:      0.0893 N
Constrained EKF/MAP tip-load RMSE:           0.0751 N
Constrained EKF/MAP total-load RMSE:         0.0598 N
Validation total-load trajectory RMSE:       0.0546 N
Constrained EKF/MAP final total-load error:  0.0358 %
Maximum reconstructed-shape RMSE:            0.0156 mm
Maximum inverse complementarity residual:    1.15e-8

Legacy one-pass Aloi total-load RMSE:        11.0220 N
Legacy one-pass Aloi final mismatch:         48.0472 %
Legacy Aloi final sparse-position RMSE:       0.3141 mm
```

The formal output above predates the nonlinear Aloi correction described
below. Its formulation result remains valid, but its Aloi fields should be
treated as a legacy implementation result. The true final contact load in
this scenario is `24.59%` axial in the material frame, so it is also outside
the paper's zero-local-axial load assumption.

The low EKF/MAP error is a same-model, noiseless consistency result. The
forward data and inverse prediction use the same rod discretization, the plane
and friction coefficient are exact, and 24 sparse shape samples are supplied.
This result does not predict experimental accuracy. Sensor noise, calibration
error, uncertain stiffness, plane error, friction uncertainty, and model
mismatch still need separate tests.

The small total-load error also should not hide the estimated nonzero tip
load: contact and tip errors partially cancel in the total. The component RMSE
values above are therefore reported with the total-load metric.

## Formulation Implementation

The inverse state is

```text
x = [p1; eta1; s1; f1n; beta1; lambda1; fe]
```

where `p1` and `eta1` parameterize the plane, `s1` is contact arclength,
`f1n` is normal force, `beta1` contains polyhedral friction coefficients,
`lambda1` is the friction-cone complementarity variable, and `fe` is the
unknown tip force.

The formal path implements the random-walk prior, nonlinear Cosserat
measurement map, finite-difference measurement Jacobian, iterated constrained
MAP update, posterior covariance update, and the normal, tangential, and cone
complementarity constraints in `Formulation.pdf` eqs. (13)-(29). The process
covariance is scaled by the number of skipped internal forward steps between
two output frames. Equation (7) uses the immediately preceding internal
forward shape, not the previous sparsely sampled output shape.

No force upper bound is active in the reported result. Optional bounds remain
available only as a numerical diagnostic and are not part of the formal run.

## Aloi Baseline

The current comparison no longer fits an internal bending-moment field. It
fits one local transverse Gaussian load directly to sparse centerline
positions by bounded nonlinear least squares. For every candidate load, the
loaded material frame, geometric Jacobian, curvature, and centerline are
iterated to nonlinear equilibrium. Multiple optimizer starts are evaluated.

One Gaussian is used because the simulated truth has one body contact. The
baseline does not receive the plane, contact location, contact normal,
friction coefficient, or complementarity constraints. Aloi et al. Eq. (9)
sets the third, axial component of the local distributed load to zero. Every
comparison now reports the true material-frame axial fraction and flags a
frame when that assumption is violated. A full-vector mismatch from such a
frame is a scenario diagnostic, not an in-domain accuracy result for the
paper.

The independent in-domain consistency test uses a known local-transverse
Gaussian with parameters `[4, -8, 110, 12]`. The estimator recovered those
parameters and the resultant force with `6.05e-8%` mismatch, a sparse-position
RMSE of `5.60e-10 mm`, and a nonlinear-equilibrium residual of `3.12e-8 mm`.
This is intentionally a noiseless same-model implementation check, not a
claim about experimental accuracy.

`force.m` remains the separate historical reproduction used for its original
test cases.

## Outputs

Formal outputs are under `force_outputs/rod_plane_displacement_force_sensing/`:

- `rod_plane_force_sensing_results.mat`
- `rod_plane_force_sensing_trajectory.csv`
- `rod_plane_force_sensing_summary.txt`
- `rod_plane_force_sensing_overview.png`
- `rod_plane_final_force_comparison.png`
- `rod_plane_aloi_error_analysis.png`
- `rod_plane_displacement_force_prediction.mp4`
- `rod_plane_displacement_aloi_prediction.mp4`

Both MP4 files contain 60 rendered frames at 10 fps and last 6 seconds. The
formulation video shows the true and estimated shape and force together with
the force history. The Aloi video and error-analysis figure show its fitted
shape, resultant force, contact center, width, and trajectory errors.

The senior-video geometry writes the same eight primary output types under
`force_outputs/rod_plane_senior_geometry_force_sensing/`. In that run the
true contact is at the 150 mm tip. The formulation recovers the final total
load to `0.6951%`, but its contact-force and tip-load RMSE values are
`8.8814 N` and `8.9013 N`: the two estimated loads cancel in the total. This
is an identifiability diagnostic, not evidence that the individual contact
force was recovered.

The original one-pass Aloi implementation reported `130.7261%`. After
updating the load transformation and Jacobian to the loaded shape and solving
nonlinear equilibrium with three optimizer starts, the raw mismatch is
`98.1337%` and the active-frame force RMSE is `6.8736 N`. The fitted shape RMSE
is only `0.00494 mm`, but the force direction error is `53.2834 deg`. The true final contact force is
`[0.7633, 0, -8.8327] N` in the material frame, or `99.6287%` axial. This
directly violates Aloi Eq. (9), so neither percentage is a fair in-domain
paper-performance result. Four `rod_plane_aloi_reanalysis.*` files retain the
corrected trajectory, summary, MAT data, and diagnostic figure.

## Requirements

- MATLAB
- Optimization Toolbox for formal `fmincon` runs
- `LCP-Continuum/` dependency in the repository root

The current forward copy is based on the stateful contact update introduced in
`Jia0Shen/LCP-Continuum@14806b3` and the push/lateral displacement sequence in
the later `simu_rod_plane.m` on `origin/main@2f40feb`.
