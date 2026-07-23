# Rod-Plane Force-Sensing Handoff

Updated: 2026-07-23

## Current Status

The displacement-aware rod-plane forward simulation, constrained EKF/MAP
inverse, Aloi Gaussian comparison, validation scripts, plots, and video all
run end to end. The formal 18-frame run passed its numerical assertions.
The senior-video horizontal-plane diagnostic also completed, including both
videos, and exposed the expected contact-at-tip identifiability failure.

The upstream `LCP-Continuum/` working tree was not modified. Its local `main`
is at `56bfd08` and the fetched `origin/main` is at `2f40feb`. The copied
contact update is based on `14806b3`; the later displacement trajectory is
visible in `origin/main:simulations/simu_rod_plane.m`.

No commit or push was made in this work session.

## Main Entry Point

```matlab
results = simu_rod_plane_displacement_force_sensing(false);
```

Formal output directory:

```text
force_outputs/rod_plane_displacement_force_sensing/
```

The formal run takes several minutes on the current machine. Progress is
printed for every saved forward frame, inverse frame, EKF iteration, selected
`fmincon` iterations, Aloi frame, and video frame.

## Implemented Pipeline

1. Build a 150 mm precurved rod and a plane.
2. Run 501 stateful internal contact steps: 45 mm push followed by 1 mm
   lateral command.
3. Save 18 output frames and each frame's immediate internal predecessor.
4. Sample 24 sparse curvature/position measurements.
5. Estimate plane, contact arclength, normal force, friction coefficients,
   cone variable, and tip force with the constrained iterated EKF/MAP.
6. Fit one Gaussian local transverse load to the sparse positions as an Aloi
   comparison.
7. Write MAT, CSV, summary, three figures, and two 60-frame MP4 files.
8. Assert force-cone feasibility, complementarity, finite shapes, and forward
   truth consistency.

## Important Fixes

### 1. Forward displacement history

The old forward loop effectively recomputed frames without preserving the
immediately preceding displacement state. Friction therefore did not reflect
the commanded lateral motion correctly.

The current forward loop retains `p`, `R`, `u`, and `T_base` at every internal
step, following the updated stateful contact API. Push and lateral motion use
maximum steps of 0.1 mm and 0.02 mm respectively.

### 2. Eq. (7) temporal reference

The inverse originally used the previous saved output frame as `p_previous`.
With many internal steps between outputs, this made the tangential
displacement and complementarity constraints inconsistent with the forward
solver.

Each output now stores its immediate internal predecessor in
`forward.previousState`. The measurement struct exposes it as
`measurements.previousShape`, and Eq. (7) uses that state.

### 3. Process covariance across skipped steps

`measurements.processStepCount` stores how many internal states lie between
two output frames. The random-walk prediction uses

```text
Pminus = Pplus_previous + processStepCount * Q
```

instead of adding one `Q` regardless of the elapsed forward steps.

### 4. Contact-mode classification

The force seed is useful for initializing contact magnitude but was not a
reliable source of sticking/sliding state. At the final strict-smoke frame the
true tangential displacement was about `0.0009 mm`, while the reduced seed
predicted about `0.139 mm`. The old logic selected sliding and forced cone
saturation, which moved body force into the tip-force state.

The mode is now selected from the measured shape, measured previous internal
state, known plane, and candidate contact arclength. The formal lateral frames
are selected as sticking.

### 5. Forward truth diagnostics

The true force is converted into the formulation state and passed through the
same nonlinear measurement and complementarity functions. End-to-end
validation rejects a run if forward truth violates the formulation thresholds.

### 6. Aloi baseline

The previous rod-plane baseline directly fitted a bending-moment field
computed from the known shape and used separate body/tip Gaussian terms. It
was not a fair shape-only comparison.

The current baseline fits one Gaussian local transverse load directly to the
24 sparse positions by nonlinear least squares. It estimates two amplitudes,
center, and width. The plane and friction data are withheld. One Gaussian is
used because the simulated truth has one contact.

This is still a paper-inspired baseline, not a complete reproduction of every
detail of Aloi et al.'s estimator. The README and experiment note state this
explicitly.

## Formulation Mapping

State:

```text
x = [p1(3); eta1(2); s1; f1n; beta1(16); lambda1; fe(3)]
```

- Eq. (3): `f1 = n1*f1n + D1*beta1`
- Eq. (4): nonlinear Cosserat force-to-shape prediction
- Eqs. (5)-(6): plane gap at `p(s1;x)`
- Eq. (7): tangential displacement from the immediate previous internal state
- Eqs. (13)-(14): random-walk state and covariance prediction
- Eqs. (15)-(17), (20)-(23): normal, friction, and cone complementarity
- Eqs. (19), (24)-(29): iterated finite-difference EKF/MAP and posterior
  covariance update

Formal runs use `fmincon`. The projected solver is allowed only in quick mode.
Force upper bounds are disabled in the reported run.

## Formal Scenario and Result

```text
rod length                    150 mm
actual integrated bend        88.49 deg
plane point                   [20, 0, 0] mm
plane normal                  [-1, 0, 0]
push/lateral command           45 / 1 mm
friction during push/lateral    0 / 0.5
true tip load                 [0, 0, 0] N
saved frames                   18
FBG-like samples               24
injected noise                  0
```

Final forces:

```text
true contact       [-33.9467,  0.0000, -8.0862] N
estimated contact  [-33.8324, -0.0081, -8.1451] N
true tip           [  0.0000,  0.0000,  0.0000] N
estimated tip      [ -0.1238,  0.0069,  0.0668] N
true total         [-33.9467,  0.0000, -8.0862] N
estimated total    [-33.9563, -0.0011, -8.0783] N
Aloi total         [-25.4481,  0.0000,  6.3671] N
```

Metrics:

```text
contact RMSE                        0.08925 N
tip RMSE                            0.07508 N
total RMSE                          0.05985 N
validation trajectory RMSE          0.05463 N
final total relative error          0.03580 %
max shape RMSE                      0.01561 mm
max inverse complementarity         1.15e-8
max forward truth inequality        1.09e-3
max forward truth equality          2.43e-2

Aloi trajectory RMSE               11.0220 N
Aloi final relative error          48.0472 %
Aloi final sparse-position RMSE     0.3141 mm
```

Both formal MP4 files were decoded end to end: 60 frames, 1770 x 930, 10 fps,
6.0 seconds each.

## Senior-Geometry Diagnostic

Entry point:

```matlab
results = simu_rod_plane_senior_geometry_force_sensing(false);
```

This retains the senior's horizontal plane, 40 mm push, and 20 mm lateral
motion while using `mu = 0.5` and zero true tip load. The final true contact
is exactly at the 150 mm tip.

```text
true contact       [-3.9648,  0.0000, -7.9297] N
estimated contact  [ 4.1008,  3.4422, -17.9580] N
estimated tip      [-8.1266, -3.4396,  10.0196] N
true total         [-3.9648,  0.0000, -7.9297] N
estimated total    [-4.0258,  0.0026, -7.9384] N
Aloi total         [ 7.6200,  0.0000, -8.2653] N
```

```text
contact-force RMSE                 8.88143 N
tip-load RMSE                      8.90126 N
total-load RMSE                    0.04109 N
final total relative error         0.69510 %
Aloi total-load RMSE               9.12503 N
Aloi final relative error        130.72610 %
```

The small total error does not mean the contact force is correct. At
`s = L`, contact force and the unknown tip-force state enter the rod model at
the same point. Their individual values can move in opposite directions with
almost no change in predicted shape or total load. This case is retained to
demonstrate the limitation and should not be used as the reportable
body-contact validation.

Both diagnostic videos were also decoded end to end: 60 frames,
1770 x 930, 10 fps, 6.0 seconds each.

## Interpretation Limits

The `0.0358%` result is not an experimental accuracy claim. It is a noiseless,
same-model consistency test with exact environment geometry and friction. It
is useful because it shows that the implementation can invert data generated
by its own model while satisfying the formulation constraints.

The estimated tip force is not exactly zero. Contact and tip errors partially
cancel in the total, so contact RMSE and tip RMSE must always be reported with
the total-load percentage.

The Aloi width reaches its 3 mm lower bound and its final center is 142.23 mm,
compared with the true contact at 125.88 mm. Its 48% result is specific to this
baseline implementation and trajectory. Do not present it as the general
error of the Aloi paper.

The current formal scenario follows the updated 150 mm displacement case and
has 88.49 degrees of intrinsic bend. It is not the old 200 mm/180-degree/high-
friction case. That older geometry caused near-tip ambiguity and formulation
mismatch and should remain a separate diagnostic unless it is redesigned and
revalidated.

Only two output directories are retained:

```text
force_outputs/rod_plane_displacement_force_sensing/
force_outputs/rod_plane_senior_geometry_force_sensing/
```

Each contains exactly eight files: MAT, CSV, text summary, three PNG figures,
one formulation MP4, and one Aloi MP4.

## Verification Commands

```matlab
report = validate_rod_plane_displacement_forward();
```

```matlab
results = validate_rod_plane_displacement_inverse();
```

```matlab
results = simu_rod_plane_displacement_force_sensing(false);
```

The strict six-frame smoke produced `0.0142%` final total-load error,
`0.0443 N` trajectory RMSE, and `3.23e-8` maximum complementarity residual.

## Recommended Next Experiment

Keep the formal deterministic run as a regression test. Add separate sweeps
for curvature noise, plane offset/normal error, stiffness mismatch, friction
coefficient mismatch, and fewer FBG points. Save each sweep under a different
output directory and report median/percentile error over multiple random
seeds. Do not tune covariance or Gaussian bounds against the known force of a
single test trajectory.
