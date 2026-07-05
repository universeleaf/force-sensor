# Force Sensing for Continuum Robots

This repository contains MATLAB experiments for continuum-robot force
estimation with Cosserat rod models. It includes the original Gaussian
load-estimation baseline and a newer rod-plane contact validation with a
known forward tip load and inverse force estimation from the measured shape.

## Repository Contents

### Aloi-Style Gaussian Baseline

- `force.m`
  - Reproduces a Cosserat-rod version of the Aloi-style Gaussian load
    parameterization.
  - Runs three two-load test cases.
  - Produces figures in `force_outputs/`.

- `force_rucker.m`, `force_ferguson.m`
  - Additional comparison implementations.

- `cosseratHelpers.m`
  - SE(3)/SO(3), strain integration, and helper routines for the Cosserat
    rod tests.

- `test_cosserat.m`
  - Basic validation cases for the Cosserat implementation.

### Rod-Plane Tip-Load Validation

The newer experiment is documented in:

- `ROD_PLANE_FORCE_SENSING_EXPERIMENT.md`

Main files:

- `simu_rod_plane_force_sensing_copy.m`
- `run_rod_plane_force_sensing_experiment.m`

This experiment uses Jia Shen's `LCP-Continuum` rod-plane model as a local
dependency. The original files under `LCP-Continuum/` are not modified. The
new script copies the rod-plane workflow into this repository and adds a local
tip-load extension for the forward simulation.

The forward pass generates:

- a rod shape trajectory,
- the contact-force trajectory,
- a known external tip load,
- and the total external load.

The inverse pass estimates the full formulation state

```text
x = [p1; eta1; s1; f1n; beta1; lambda1; fe]
```

where `p1` and `eta1` describe the plane, `s1` is the contact arclength,
`f1n` and `beta1` describe the contact/friction force, `lambda1` is the
friction-cone slack variable, and `fe` is the unknown tip load. The current
script implements the iterated constrained EKF/MAP update from
`Formulation.pdf`: random-walk prior covariance, nonlinear Cosserat forward
map `F(s1, f1, fe)`, measurement linearization `H = dh/dx`, posterior
covariance update, and the normal/friction/cone complementarity constraints.

Formal runs use MATLAB `fmincon` to solve the constrained MAP subproblem in
eq. (26), with the complementarity constraints from eqs. (20)-(23). The
short `quickMode` smoke test uses `cfg.forceSensor.solver = 'projected'` only
to check that the pipeline executes; that approximate path should not be used
for final force-error claims.

The default configuration does not impose artificial upper bounds on the
unknown contact or tip forces, because those bounds are not part of
`Formulation.pdf` eqs. (19)-(23). The optional
`cfg.forceSensor.useForceBounds = true` path is kept only as a numerical
diagnostic/regularized variant and should be reported separately.

The Aloi-style method is used as a shape-only total-load baseline for the same
rod-plane trajectory. It does not use the plane, contact, or friction
constraints.

## Running the Code

Run the Aloi-style baseline:

```matlab
force()
```

Run the rod-plane tip-load experiment:

```matlab
results = simu_rod_plane_force_sensing_copy(false);
```

The default rod-plane case is the separated-contact validation summarized
below. It assumes the simulated shape/plane are known, while the MAP estimator
still uses finite measurement covariance as a numerical weighting term.

Run a shorter smoke test:

```matlab
results = simu_rod_plane_force_sensing_copy(true);
```

The smoke test writes to `force_outputs/rod_plane_force_sensing_smoke_tmp/`
so it does not overwrite the formal validation outputs.

## Latest Rod-Plane Result

The values below are from `simu_rod_plane_force_sensing_copy(false)` after the
full constrained EKF/MAP implementation was checked against `Formulation.pdf`.
This is a separated-contact case chosen because the original high-friction
180-degree setup put the contact too close to the tip and made the contact/tip
split poorly identifiable.

Scenario:

- Rod length: `180 mm`
- Integrated precurvature: `270 deg`
- Plane: `z = 10 mm`
- Friction coefficient: `mu = 0.8`
- Base insertion: `0 mm` to `35 mm`
- Forward tip load: `[0, 0, -3.5] N`
- Frames: `16`
- Force bounds: disabled
- Artificial sensing noise: disabled

Final-frame force estimates:

```text
True contact force:       [-8.1373, 0, -10.1716] N
Estimated contact force:  [-8.4170, 0, -10.5234] N

True tip load:            [0, 0, -3.5000] N
Estimated tip load:       [0.2787, 0, -3.1418] N

True total load:          [-8.1373, 0, -13.6716] N
Estimated total load:     [-8.1383, 0, -13.6652] N
Aloi-style total load:    [-20.2654, 0, -10.4372] N
```

Trajectory metrics:

```text
Shape + environment contact-force RMSE: 1.2748 N
Shape + environment tip-load RMSE:      1.0604 N
Shape + environment total-load RMSE:    1.2464 N
Shape + environment final total error:  0.0406 %

Aloi total-load RMSE:                   12.0957 N
Aloi final total error:                 78.8943 %
```

The constrained method recovers the final total load very closely, but the
contact/tip split is not exact. The remaining contact and tip errors are real
and should be reported as part of the validation. The Aloi-style comparison is
much worse here because it fits a shape-only Gaussian load distribution and
does not receive the plane, contact, or friction constraints.

The earlier 200 mm, 180-degree, high-friction setup was kept as a diagnostic
case, but it should not be used as the main validation result. In that case the
forward LCP contact and the strict single-contact complementarity formulation
are not cleanly aligned, and the inverse can move force between a near-tip
contact and the unknown tip load.

## Output Files

General Aloi/Cosserat outputs are written under `force_outputs/`.

Rod-plane validation outputs are written under:

```text
force_outputs/rod_plane_force_sensing/
```

Key files:

- `rod_plane_force_sensing_results.mat`
- `rod_plane_force_sensing_trajectory.csv`
- `rod_plane_force_sensing_summary.txt`
- `rod_plane_force_sensing_overview.png`
- `rod_plane_final_force_comparison.png`

## Requirements

- MATLAB
- Local `LCP-Continuum/` dependency for the rod-plane contact experiment
- No changes to the upstream `LCP-Continuum` source files are required

## References

1. Rucker and Webster, "Statics and Dynamics of Continuum Robots With General
   Tendon Routing and External Loading", 2011.
2. Aloi et al., Gaussian load parameterization for continuum robot force
   estimation.
3. Shen et al., frictional contact modeling of continuum robots with a linear
   complementarity formulation.
