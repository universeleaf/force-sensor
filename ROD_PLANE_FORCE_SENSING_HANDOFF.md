# Rod-Plane Force Sensing Handoff

Last updated: 2026-07-05

This note is for continuing the rod-plane force-sensing work in a new window. It records the honest current state, including the bugs found, the fixes already made, and the results that should not be overclaimed.

## Goal

Use a local copy of the senior student's rod-plane simulation to:

1. Generate a forward rod-plane trajectory with known contact force and known tip load.
2. Treat the simulated shape as known/measured.
3. Estimate the force using the constrained formulation in `Formulation.pdf`.
4. Compare against the Aloi-style Gaussian baseline implemented from `force.m`.

Important constraints:

- Do not modify the original `LCP-Continuum` source.
- Use only copied/local experiment code in this repository.
- Do not fake or polish the result. If the estimate fails or is only partially identifiable, say that directly.
- The formulation in `Formulation.pdf` does not contain force upper bounds or a mandatory active-contact lower bound; do not add those as if they were part of the paper formulation.

## Main Files

- `simu_rod_plane_force_sensing_copy.m`: local entry point. Calls the copied experiment runner.
- `run_rod_plane_force_sensing_experiment.m`: main copied forward simulation, inverse solver, diagnostics, plotting, and output writer.
- `Formulation.pdf`: actual target formulation.
- `force.m`: Aloi-style comparison reference.
- `README.md`: updated with the 16-frame representative run and the current caveats.
- `force_outputs/rod_plane_force_sensing/`: default output folder for the final representative run.

## Implemented Formulation

The inverse state follows `Formulation.pdf`:

`x = [p1; eta1; s1; f1n; beta1; lambda1; fe]`

The implemented model uses:

- Contact force: `f1 = n1*f1n + D1*beta1`.
- Forward map: `[p(s;x), u(s;x)] = F(s1, f1, fe)`.
- Contact point: `pc(x) = p(s1;x)`.
- Gap: `g = n1'*(pc - p1)`.
- Tangential displacement: `v = (I - n*n')*(p(s1;x) - p_prev(s1))`.
- Measurement model: `z = [u_measured_at_FBG; p1; n1]`, `h(x) = [u_model_at_FBG; p1; N(eta1)]`.
- Constraints: normal complementarity, friction complementarity, cone complementarity, and `0 <= s1 <= L`.
- Solver: constrained EKF/MAP with finite-difference linearization and `fmincon` SQP for the constrained MAP subproblem.

The code currently has no default force upper bound and no active-contact lower bound.

## Important Fixes Already Made

1. Fixed the inverse contact gap to use the centerline point `p(s1;x)`, matching `Formulation.pdf` and the copied LCP rod-plane code. Earlier code incorrectly used a radius-shifted surface point.
2. Fixed the same stale radius-shifted gap inside the reduced initializer.
3. Added known-tip-force support to the copied forward rod-plane solver without editing the senior student's original files.
4. Added storage of forward LCP contact variables in the copied contact structs:
   - `normal_force`
   - `friction_beta`
   - `friction_lambda`
   - `friction_directions`
5. Added candidate MAP diagnostics for:
   - truth
   - measurement-init tip-only
   - reduced seed
   - projected reduced seed
   - zero-force seed
6. Changed the inverse friction update to use the measured previous shape for `p_prev(s1)` instead of feeding back the previously estimated shape. This matches the intended "known shape" setting better and prevents estimated-force errors from contaminating the friction displacement term.
7. Added progress logging for long `fmincon`/EKF runs.

## Key Evidence

### High-friction 180 degree case is not a clean validation case

The original intended high-friction setup was approximately:

- rod length: 200 mm
- bend: 180 deg
- wall distance: 20 mm
- friction: high, e.g. `mu = 2.8`
- tip load: `[0, 0, -3.5] N`

This case generated large contact forces, but it was not a good strict-formulation validation case. Even after fixing the centerline gap, the forward LCP truth had noticeable friction/complementarity mismatch under the strict PDF constraints. The inverse often found a no-contact or tip-dominated explanation because unknown contact force and unknown tip load are not well separated in this geometry.

Do not present this setup as a successful force split.

### Better separated contact diagnostic

A more useful diagnostic case is now the default formal run:

- rod length: 180 mm
- bend: 270 deg
- wall distance: 10 mm
- `betaMax = 35 mm`
- `mu = 0.8`
- tip load: `[0, 0, -3.5] N`
- no artificial sensing noise

This case moves contact away from the tip and makes contact/tip separation more meaningful.

For the latest 16-frame no-noise run:

- true final contact: about `[-8.137, 0, -10.172] N`
- estimated final contact: about `[-8.417, 0, -10.523] N`
- true final tip: `[0, 0, -3.5] N`
- estimated final tip: about `[0.279, 0, -3.142] N`
- final total-load error: about `0.0406 %`
- contact-force RMSE: about `1.275 N`
- tip-force RMSE: about `1.060 N`
- total-load RMSE: about `1.246 N`
- Aloi-style final total-load error: about `78.9 %`

Interpretation: the constrained formulation can recover the total load very well in this diagnostic case, but the contact/tip split still has a real error near the 1 N scale. That should be reported, not hidden.

### Aloi baseline

The Aloi-style Gaussian/shape-only baseline can look reasonable in tip-like contact cases, but it performs badly in the separated rod-plane contact case. The roughly 79% final total-load error is plausible for this scenario because the baseline does not use the plane/friction/contact constraints.

## Current Interpretation

The code is now much closer to `Formulation.pdf` than earlier versions, but there is still an identifiability issue:

- If contact is near the tip, an unknown tip load can explain much of the shape.
- Complementarity permits the no-contact branch.
- The current single-contact formulation does not impose global nonpenetration along the whole rod.
- Shape-only measurements strongly constrain total load, but they do not always uniquely split contact force and tip force.

This means a very small total-load error can be real while the contact/tip decomposition is still imperfect.

## Next Steps

1. The final representative validation has already been run after the latest code changes. The simple command now uses the same default configuration:

```matlab
results = simu_rod_plane_force_sensing_copy(false);
```

2. If the code changes again, rerun the same command and check:
   - `force_outputs/rod_plane_force_sensing/rod_plane_force_sensing_summary.txt`
   - `force_outputs/rod_plane_force_sensing/rod_plane_force_sensing_overview.png`
   - `force_outputs/rod_plane_force_sensing/rod_plane_final_force_comparison.png`
3. Stage only the meaningful files:
   - `run_rod_plane_force_sensing_experiment.m`
   - `simu_rod_plane_force_sensing_copy.m`
   - `README.md`
   - `ROD_PLANE_FORCE_SENSING_HANDOFF.md`
   - final validated files in `force_outputs/rod_plane_force_sensing/`, if they are referenced by the README
4. Do not stage `.matlab_pref/`, `tmp/`, or diagnostic `force_outputs/*_tmp/` directories.

## MATLAB Notes

Batch runs often print repeated shutdown warnings like:

`Unable to load ApplicationService for command client-v1`

These warnings can appear after successful runs. Check the generated metrics and output files before treating the run as failed.

Do not kill the user's visible MATLAB GUI. If a batch run times out, check for orphaned blank MATLAB batch processes before killing anything.

## Git Notes

The worktree currently contains many generated diagnostic outputs. Before committing or pushing, re-run:

```powershell
git status --short
```

Then stage selectively. The goal is to commit the reproducible experiment and honest documentation, not every temporary diagnostic folder.
