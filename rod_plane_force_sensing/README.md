# Rod-Plane Force Sensing

This folder contains every MATLAB script used by the current rod-plane
forward/inverse experiment. `LCP-Continuum/` is an external dependency at the
repository root and is not modified.

## Add the folder to MATLAB

From the repository root:

```matlab
addpath(fullfile(pwd, 'rod_plane_force_sensing'));
```

The simplest way to start in MATLAB is to set Current Folder to
`C:\Users\wty05\Desktop\gatech\force sensor\rod_plane_force_sensing` and run
one of the two `simu_*.m` files below. Each file can also be opened and run
with the Editor Run button because both default to `quickMode = false`.

## Main scenario

```matlab
results = simu_rod_plane_displacement_force_sensing(false);
```

This is the reportable deterministic scenario: 150 mm rod, vertical plane,
45 mm push, 1 mm lateral displacement, friction coefficient 0.5 during the
lateral phase, and zero true tip load. The final body contact is separated
from the tip at approximately 125.9 mm.

Results are written to:

```text
C:\Users\wty05\Desktop\gatech\force sensor\force_outputs\rod_plane_displacement_force_sensing\
```

The lower-level command below now uses the same defaults and therefore solves
the same physical problem:

```matlab
results = run_rod_plane_force_sensing_experiment(false);
```

## Senior-video geometry

```matlab
results = simu_rod_plane_senior_geometry_force_sensing(false);
```

This reproduces the horizontal-plane geometry and 40 mm push plus 20 mm
lateral motion in the senior's updated simulation, with friction reduced to
0.5 and zero true tip load. It is a diagnostic case: contact remains at the
150 mm tip, so body-contact force and tip load are not separately identifiable.

Results are written to:

```text
C:\Users\wty05\Desktop\gatech\force sensor\force_outputs\rod_plane_senior_geometry_force_sensing\
```

The 2026-07-23 run recovered the final total load to `0.6951%`, but contact
and tip RMSE were `8.8814 N` and `8.9013 N`, respectively. These component
loads should not be reported as successful recovery; they cancel because a
body force at `s = L` is mechanically indistinguishable from a tip load in
the shape model. The Aloi final total-load error was `130.7261%`.

## Quick checks

```matlab
results = simu_rod_plane_displacement_force_sensing(true);
report = validate_rod_plane_displacement_forward();
results = validate_rod_plane_displacement_inverse();
```

Quick mode uses the projected approximate solver. It checks code flow and
rendering but its force error is not a formal result.

## Files

- `simu_rod_plane_displacement_force_sensing.m`: validated scenario entry.
- `simu_rod_plane_senior_geometry_force_sensing.m`: senior-geometry diagnostic.
- `run_rod_plane_force_sensing_experiment.m`: forward LCP, constrained EKF/MAP,
  output, plotting, and video pipeline.
- `estimate_aloi_gaussian_baseline.m`: independent shape-only Aloi baseline.
- `validate_rod_plane_displacement_forward.m`: forward physics assertions.
- `validate_rod_plane_displacement_inverse.m`: strict six-frame inverse check.
- `validate_rod_plane_displacement_results.m`: end-to-end result assertions.
- `ROD_PLANE_FORCE_SENSING_EXPERIMENT.md`: formulation and numerical details.
- `PROJECT_SUMMARY.md`: implementation handoff and debugging record.

## Outputs

Each retained scenario has one directory under `force_outputs/`. A completed
run writes exactly these eight files:

1. `rod_plane_force_sensing_results.mat`
2. `rod_plane_force_sensing_trajectory.csv`
3. `rod_plane_force_sensing_summary.txt`
4. `rod_plane_force_sensing_overview.png`
5. `rod_plane_final_force_comparison.png`
6. `rod_plane_aloi_error_analysis.png`
7. one 6-second constrained-formulation MP4
8. one 6-second Aloi MP4

The videos contain 60 rendered frames at 10 fps. Interpolated render frames
are for visualization only; force estimation is performed at the saved
simulation frames.
