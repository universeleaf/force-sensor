# Rod-plane force-sensing validation

This note records the copied rod-plane experiment used for the tip-load
force-sensing check. The original `LCP-Continuum` files are used as a
dependency only and are not modified.

## Entry point

```matlab
results = simu_rod_plane_force_sensing_copy(false);
```

Use `true` for a shorter smoke run.

The main implementation is in:

- `simu_rod_plane_force_sensing_copy.m`
- `run_rod_plane_force_sensing_experiment.m`

The generated result files are written to:

```text
force_outputs/rod_plane_force_sensing/
```

## Scenario

- Copied rod-plane contact setup based on `LCP-Continuum/simulations/simu_rod_plane.m`.
- Rod length: `200 mm`.
- Precurvature scaled to an integrated bend of about `180 deg`.
- Plane at `z = 20 mm`, friction coefficient `mu = 2.8`.
- Base insertion: `0 mm` to `30 mm` over `120` frames.
- Additional known forward tip load: `[0, 0, -3.5] N`.

The forward pass produces the contact-force trajectory and the full rod
shape trajectory. The inverse pass only receives the noisy shape/curvature
measurement and the plane estimate; it does not read the forward contact
force or contact index.

## Methods Compared

1. Forward reference

   A local copied contact solve extends the rod-plane LCP solve with the
   known tip load. This produces the reference shape, contact force, and
   total external load.

2. Shape + environment full EKF/MAP formulation

   The inverse state follows the state definition in the formulation note:

   ```text
   x = [p1; eta1; s1; f1n; beta1; lambda1; fe]
   ```

   The implementation now follows `Formulation.pdf` eqs. (19)-(29): each
   frame uses the previous posterior as a random-walk prior, predicts the
   curvature/shape through the nonlinear Cosserat map
   `[p(s;x), u(s;x)] = F(s1, f1, fe)`, linearizes the measurement model
   `h(x)`, solves the constrained MAP subproblem, and updates the approximate
   posterior covariance. The constraints include normal contact
   complementarity, tangential friction complementarity, the Coulomb cone
   slack complementarity, and `0 <= s1 <= L`.

   The rod-plane model stores `p(s)` on the rod centerline, so the plane gap
   is evaluated at the radius-shifted surface point. This is the geometry
   conversion needed to apply the PDF's plane-contact gap equation to this
   simulation.

   Formal runs use MATLAB `fmincon` for the constrained MAP subproblem in
   eq. (26). The projected solver is kept only for short smoke tests, where
   it is useful for checking data flow but is not treated as the formulation
   result.

   Force upper bounds are disabled by default because they are not part of
   `Formulation.pdf` eqs. (19)-(23). They can be enabled only as a separate
   bounded regularization diagnostic, not as the main formulation result.

3. Aloi-style baseline

   The comparison baseline follows the Gaussian distributed-load idea used
   in `force.m`. It is run as a shape-only total-load fit and does not use
   the measured plane, contact geometry, or friction constraints.

## Latest Full Run

Output files:

- `rod_plane_force_sensing_results.mat`: full MATLAB result struct.
- `rod_plane_force_sensing_trajectory.csv`: per-frame insertion, tip pose,
  true/estimated contact force, true/estimated tip load, total load, and
  error norms.
- `rod_plane_force_sensing_summary.txt`: numeric summary.
- `rod_plane_force_sensing_overview.png`: trajectory and force-error plots.
- `rod_plane_final_force_comparison.png`: final-frame shape and force plot.

The values below are the earlier 120-frame reduced-solver reference. They are
kept to show the scenario and output format; re-run
`simu_rod_plane_force_sensing_copy(false)` after the full EKF/MAP change to
regenerate formal full-run numbers.

Final-frame values from the earlier reduced-reference run:

```text
True contact force:       [-50.8779, 0, -21.5140] N
Estimated contact force:  [-47.6958, 0, -20.6914] N

True tip load:            [0, 0, -3.5000] N
Estimated tip load:       [-0.1437, 0.0032, -3.4260] N

True total load:          [-50.8779, 0, -25.0140] N
Estimated total load:     [-47.8394, 0.0032, -24.1173] N
Aloi-style total-load estimate: [-96.3181, 0.5677, -12.6541] N
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

The shape + environment result is relatively accurate because the inverse
problem is given the same Cosserat rod model, the measured shape, and the
plane/friction constraints. The remaining error mostly comes from noisy
curvature interpolation, the biased plane measurement, and the ambiguity
between a near-tip contact force and a tip load.

The Aloi-style baseline is much less constrained here, so it tends to overfit
a distributed load that matches the shape but gives a poor total force in this
contact case. In the final frame the normalized shape-fit residual is only
`0.0604`, but the body Gaussian resultant is
`[-94.2041, 0.4996, -4.2274] N` and the tip Gaussian resultant is
`[-2.1140, 0.0682, -8.4267] N`. The large `83.0672 %` total-force error should
therefore be interpreted as the error of this shape-only Gaussian baseline on
the rod-plane contact/tip-load case, not as a general performance claim about
the Aloi paper.
