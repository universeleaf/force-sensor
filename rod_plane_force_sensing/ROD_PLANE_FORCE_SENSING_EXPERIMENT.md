# Rod-Plane Displacement Force-Sensing Experiment

## Purpose

This experiment checks whether the force-sensing formulation can recover a
single frictional body-contact force from a known rod shape. The reference
shape and force are generated first. The inverse estimator then receives only
sparse shape data, the measured plane, the friction coefficient, and its
temporal prior. A Gaussian shape-only estimator is run on the same sparse
positions as a comparison.

The true tip load is zero. The tip-force state is kept in the inverse problem,
so the estimator must distinguish a body contact from a possible tip load
instead of being told that the tip load is zero.

## Files

- Scenario entry point: `simu_rod_plane_displacement_force_sensing.m`
- Forward, inverse, Aloi fit, plots, and video:
  `run_rod_plane_force_sensing_experiment.m`
- Forward assertions: `validate_rod_plane_displacement_forward.m`
- End-to-end assertions: `validate_rod_plane_displacement_results.m`
- Formal outputs: `force_outputs/rod_plane_displacement_force_sensing/`

The original `LCP-Continuum/` working tree is not edited.

## Forward Simulation

The forward path is a local copy of the updated stateful rod-plane contact
workflow. At every internal displacement step it retains the previous
`p`, `R`, `u`, and base transform, updates contact location, and solves the
frictional LCP. This is necessary because tangential friction depends on the
incremental displacement between consecutive states.

The formal trajectory uses:

```text
rod length                  150 mm
integrated precurvature      88.49 deg
plane point                 [20, 0, 0] mm
plane normal                [-1, 0, 0]
push command                 45 mm
lateral command               1 mm
push friction                 0
lateral-phase friction        0.5
true tip load                [0, 0, 0] N
internal push step            0.1 mm maximum
internal lateral step         0.02 mm maximum
saved output frames          18
```

There are 501 internal forward states. Only 18 are retained as measurement
frames, but each retained frame also stores its immediate internal predecessor
for the displacement term in Formulation eq. (7).

The final contact lies at `s = 125.88 mm`, sufficiently far from the 150 mm
tip to avoid the near-tip body-force/tip-force ambiguity of the older setup.

## Simulated Measurements

Twenty-four arclength locations are used as FBG-like samples. The inverse
measurement vector contains sparse curvature, one plane point, and the plane
normal. Sparse positions are stored for the Aloi comparison.

The formal consistency run injects no random sensing noise or plane bias.
The nonzero measurement covariance is still required in the MAP objective: it
sets the relative weighting of curvature and plane measurements. It must not
be described as injected noise in this run.

## Constrained EKF/MAP Mapping

The state follows `Formulation.pdf`:

```text
x = [p1(3); eta1(2); s1; f1n; beta1(m); lambda1; fe(3)]
```

with `m = 16` friction-cone directions.

The implementation maps to the formulation as follows:

1. **Contact force, eq. (3).**
   `f1 = n1*f1n + D1*beta1`, where the columns of `D1` span the tangent
   plane and approximate the Coulomb cone.
2. **Rod prediction, eq. (4).**
   Contact and tip forces are converted to nodal loads and moments. The
   Cosserat strain and centerline are integrated to obtain
   `[p(s;x), u(s;x)] = F(s1,f1,fe)`.
3. **Plane gap, eqs. (5)-(6).**
   The gap is evaluated at the predicted contact centerline point using the
   same centerline convention as the copied LCP contact code.
4. **Incremental tangential displacement, eq. (7).**
   `v1 = (I-n1*n1') * (p(s1;x)-p_previous(s1))`. `p_previous` is the
   immediately preceding internal forward state.
5. **Random-walk process, eqs. (13)-(14).**
   The previous posterior is the next prior. When output frames skip internal
   states, `Q` is multiplied by the number of skipped steps.
6. **Complementarity, eqs. (15)-(17) and (20)-(23).**
   The solver enforces unilateral normal contact, tangential friction
   complementarity, Coulomb-cone slack complementarity, nonnegative
   `f1n`, `beta1`, and `lambda1`, and `0 <= s1 <= L`.
7. **Iterated EKF/MAP, eqs. (19) and (24)-(29).**
   The nonlinear measurement function is finite-difference linearized at the
   current iterate. `fmincon` solves the constrained quadratic MAP
   subproblem, a damped nonlinear merit check accepts the update, and the
   posterior covariance is updated from the accepted linearization.

No forward force or forward contact index is supplied to the estimator.
Forward truth is used only after estimation for diagnostics and error metrics.
Force upper bounds are disabled.

The active complementarity branch is selected from observed shape motion and
known friction. In particular, the final frames are classified as sticking.
Using the reduced force seed to classify the mode previously selected sliding
incorrectly and forced the friction cone to saturation.

## Aloi Gaussian Comparison

The comparison uses one Gaussian local transverse load because there is one
true body contact. For each frame it:

1. builds the unloaded precurved reference shape;
2. parameterizes the local load as
   `f_local(s) = [a1*g(s), a2*g(s), 0]`, matching Aloi et al. Eq. (9);
3. transforms that load through the current loaded material frame;
4. updates the geometric Jacobian, curvature, material frame, and centerline
   to nonlinear fixed-point equilibrium for every candidate load;
5. fits amplitude, center, and width to the 24 sparse centerline positions by
   bounded, multi-start nonlinear least squares; and
6. integrates the fitted distributed load to obtain the global resultant.

This follows the Gaussian parameterization and sparse-position objective used
by Aloi et al. and `force.m`. It is a paper-inspired comparison rather than a
claim of exact reproduction of every estimator detail in that paper. It does
not use the plane or friction model.

The old implementation transformed the load with the unloaded material frame
and applied only one `J'F -> curvature -> shape` update. That approximation
has been removed. The unloaded linearization is now used only to rank initial
guesses; every optimizer residual uses the loaded nonlinear equilibrium.

Equation (9) also creates a hard applicability limit: local axial load is
fixed to zero because it is difficult to identify from shape alone. The code
therefore transforms the true contact force into the forward material frame,
reports its axial fraction, and marks results outside a 5% diagnostic
threshold. A full-vector mismatch is still saved, but it is not interpreted
as paper accuracy when this flag is false.

The earlier baseline directly fitted a bending-moment field derived from the
known shape. That gave the baseline information unavailable in a real inverse
problem and has been removed.

## Formal Result

Command:

```matlab
results = simu_rod_plane_displacement_force_sensing(false);
```

Final frame:

```text
true contact force       [-33.9467,  0.0000, -8.0862] N
estimated contact force  [-33.8324, -0.0081, -8.1451] N
true tip load            [  0.0000,  0.0000,  0.0000] N
estimated tip load       [ -0.1238,  0.0069,  0.0668] N
true total load          [-33.9467,  0.0000, -8.0862] N
estimated total load     [-33.9563, -0.0011, -8.0783] N
legacy one-pass Aloi     [-25.4481,  0.0000,  6.3671] N
```

Trajectory metrics:

```text
contact-force RMSE                         0.08925 N
tip-load RMSE                              0.07508 N
total-load RMSE                            0.05985 N
validation total-load RMSE                 0.05463 N
final total-load relative error            0.03580 %
maximum shape RMSE                         0.01561 mm
maximum inverse complementarity residual   1.15e-8
maximum truth inequality violation         1.09e-3
maximum truth equality residual            2.43e-2

legacy Aloi total-load RMSE               11.0220 N
legacy Aloi final mismatch                48.0472 %
legacy Aloi final position RMSE            0.3141 mm
```

All assertions in `validate_rod_plane_displacement_results.m` passed.

The formal formulation and Aloi videos each contain 60 rendered frames at
10 fps and last 6.0 seconds. The 18 saved simulation frames are interpolated
only for rendering; the inverse problem is not solved on interpolated data.

## Interpretation

The constrained result is a deterministic same-model consistency test, not an
experimental accuracy result. It is reasonable for the total-load error to be
small because the synthetic measurements are noiseless, the plane and
friction coefficient are exact, and forward and inverse share the same rod
model. The nonzero estimated tip load and the larger contact/tip component
errors show that the inverse split is not exact even when the total is nearly
exact.

The stored formal output predates the nonlinear Aloi correction. Its `48%`
value is retained only as a legacy one-pass result. The true formal-scenario
contact force is `24.59%` axial in the local material frame, so that scene is
also outside the paper's zero-axial load model. Its value must not be used as
a general Aloi accuracy claim.

The next validation step should introduce controlled curvature noise,
stiffness mismatch, plane error, and friction uncertainty one at a time, then
report sensitivity curves rather than a single noiseless percentage.

## Horizontal-Plane Tip-Contact Diagnostic

The second entry point reproduces the senior-video plane direction and motion
while setting the true tip load to zero and reducing friction to 0.5:

```matlab
results = simu_rod_plane_senior_geometry_force_sensing(false);
```

It uses a 40 mm push along `+z`, followed by 20 mm lateral motion along `+x`.
The final forward contact remains at `s = 150 mm`, exactly at the rod tip.

```text
true contact force       [-3.9648,  0.0000, -7.9297] N
estimated contact force  [ 4.1008,  3.4422, -17.9580] N
estimated tip load       [-8.1266, -3.4396,  10.0196] N
true total load          [-3.9648,  0.0000, -7.9297] N
estimated total load     [-4.0258,  0.0026, -7.9384] N
legacy one-pass Aloi     [ 7.6200,  0.0000, -8.2653] N
corrected nonlinear Aloi [ 4.6395,  0.0000, -9.2173] N
```

```text
formulation contact-force RMSE       8.8814 N
formulation tip-load RMSE            8.9013 N
formulation total-load RMSE          0.0411 N
formulation final total-load error   0.6951 %
legacy Aloi final mismatch         130.7261 %
corrected Aloi final mismatch       98.1337 %
corrected active-frame RMSE           6.8736 N
corrected magnitude mismatch        16.3946 %
corrected direction mismatch        53.2834 deg
true local axial fraction           99.6287 %
corrected sparse-position RMSE       0.00494 mm
nonlinear-equilibrium residual       3.59e-5 mm
```

The formulation constraints are satisfied, but the individual contact and
tip loads are not recovered. This is mechanically expected: when `s1 = L`,
the body-contact force and `fe` act at the same centerline point, so sparse
shape data mainly identifies their sum. The estimated components therefore
become large and opposite while their total stays close to truth. The case is
useful as an identifiability failure demonstration, not as a successful
contact-force estimate.

The original `130.73%` was partly inflated by the old unloaded-frame,
one-pass approximation. The corrected three-start nonlinear fit lowers it to
`98.13%` and converges tightly, but the force direction remains wrong despite
an excellent shape fit. At final contact the true material-frame force is
`[0.7633, 0, -8.8327] N`; `99.63%` of its norm is axial. Aloi's load model
cannot represent that component. The result therefore demonstrates an
out-of-domain shape-only ambiguity, not a `98%` general error rate for the
paper.

The implementation itself is checked with:

```matlab
report = validate_aloi_gaussian_baseline();
```

This generates a known in-domain local-transverse Gaussian with parameters
`[4, -8, 110, 12]`, then estimates it from 24 sparse positions. The recovered
resultant mismatch is `6.05e-8%`, sparse-position RMSE is `5.60e-10 mm`, and
equilibrium residual is `3.12e-8 mm`. This is a noiseless same-model code
check, not an experimental performance result.

The corrected senior trajectory can be regenerated from the saved MAT file
without rerunning forward LCP or the formulation estimator:

```matlab
analysis = rerun_aloi_saved_result([], 'all');
```
