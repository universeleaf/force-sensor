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

The inverse pass assumes the shape is measured and estimates the force state

```text
x = [p1; eta1; s1; f1n; beta1; lambda1; fe]
```

where `p1` and `eta1` describe the plane, `s1` is the contact arclength,
`f1n` and `beta1` describe the contact/friction force, `lambda1` is the
friction-cone slack variable, and `fe` is the unknown tip load.

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

Run a shorter smoke test:

```matlab
results = simu_rod_plane_force_sensing_copy(true);
```

## Latest Rod-Plane Result

Scenario:

- Rod length: `200 mm`
- Integrated precurvature: about `180 deg`
- Plane: `z = 20 mm`
- Friction coefficient: `mu = 2.8`
- Base insertion: `0 mm` to `30 mm`
- Forward tip load: `[0, 0, -3.5] N`
- Frames: `120`

Final-frame force estimates:

```text
True contact force:       [-50.8779, 0, -21.5140] N
Estimated contact force:  [-47.6958, 0, -20.6914] N

True tip load:            [0, 0, -3.5000] N
Estimated tip load:       [-0.1437, 0.0032, -3.4260] N

True total load:          [-50.8779, 0, -25.0140] N
Estimated total load:     [-47.8394, 0.0032, -24.1173] N
Aloi total-load estimate: [-96.3181, 0.5677, -12.6541] N
```

Trajectory metrics:

```text
Shape + environment contact-force RMSE: 1.4986 N
Shape + environment tip-load RMSE:      0.4149 N
Shape + environment total-load RMSE:    1.6406 N
Shape + environment final total error:  5.5879 %

Aloi total-load RMSE:                   29.1297 N
Aloi final total error:                 83.0672 %
```

The shape + environment formulation is much better constrained in this
rod-plane case because it uses the measured shape together with the plane and
friction-cone information. The remaining error mainly comes from noisy
curvature interpolation, plane offset bias, and the force-decomposition
ambiguity between a near-tip contact and an actual tip load.

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
