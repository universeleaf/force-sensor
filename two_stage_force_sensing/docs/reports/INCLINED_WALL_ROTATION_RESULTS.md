# Loaded-state bidirectional rotation and force videos

Date: 2026-10-10. Branch: codex/two-stage-force-sensing, base b376013. Local, uncommitted results.

## Requested experiment

The insertion video now plots reference and estimated world XYZ components of both tip load and contact force for mu=0.3 and mu=0. It uses the existing conditional-mode results without rerunning or changing insertion mechanics. The earlier video remains available.

Rotation starts from **insertion frame 21**, the largest insertion: perpendicular wall distance 40 mm, base Z = 56.56854249 mm. Both the rendered frame and full MAT checkpoint are saved. The reference retains its contact preload; each estimator resumes its own previous force, shape, contact geometry, and mode. Neither estimator is initialized with reference forces. Assertions confirm exact equality of the restored estimator states and reference frame with their saved insertion states.

The new actuation is axial base rotation about local Z: **0 -> 45 -> 0 degrees**, in 3.75-degree increments (25 recorded states, 24 new solves per method). Base translation, wall, and the world tip load [1,0,0] N remain fixed. The rod is 200 mm long: 80 mm straight and 120 mm intrinsically curved through 90 degrees. Both estimators receive the same noiseless frictional reference, true mu=0.3. Mu=0 is an estimator ablation.

The independent forward wrapper calls the collaborator's existing full 3D `solve_cosserat_multi_contact_map` read-only, with local stick/slip branch root finding. The saved planar reference and 3D map agree to 2.21e-8 mm at the starting state. Sensor generation converts the full material curvature through the existing Bishop map on a refined 0.1 mm mesh, then samples the existing 24 FBG stations. Reference forces and shapes are only used by the simulator and after estimation for scoring. Geometry and inverse mechanics receive the standard observation packet.

Existing assumptions remain: centerline point-force Coulomb friction, no separate surface-friction torque, no tip contact or edge/corner geometry. Time increments of 0.02 s are quasistatic history bookkeeping. These videos are not real-time solver playback.

## Measured results

Aggregates below exclude the restored frame, which takes no new solve. Timing includes Stage 1, all attempted modes, and diagnostics; reference simulation and video rendering are excluded. RMSE uses full force-vector errors and all newly actuated frames, including flagged fallbacks.

| Metric, 24 new states | Ours, mu=0.3 | Ours, mu=0 |
|---|---:|---:|
| Positive solver exit + branch feasibility | 24/24 | 24/24 |
| Also passes raw FBG RMS <= 1e-6 /mm | 10/24 | 0/24 |
| Tip-force RMSE [N] | 0.09755 | 1.55852 |
| Contact-force RMSE [N] | 0.34507 | 5.50778 |
| Shape RMS [mm] | 0.01554 | 0.29630 |
| Maximum tip-force error [N] | 0.36882 | 2.16363 |
| Maximum contact-force error [N] | 1.17727 | 7.73436 |
| Mean total time per step [s] | 4.797 | 2.044 |
| Median / maximum time [s] | 3.186 / 36.811 | 2.022 / 3.852 |
| Mean iterations, all branches | 57.67 | 38.33 |
| Mean IVP evaluations, all branches | 121.04 | 46.17 |
| Total inverse time [s] | 115.128 | 49.048 |
| Failed IVPs | 0 | 0 |

The friction-aware estimate chooses sliding for all 12 forward steps and sticking for all 12 return steps, matching the physical reference modes. It passes the FBG gate at 10/12 forward steps and 0/12 return steps. Flagged frames are 8, 13, and 14-25. Mean forward/return times are 1.955/7.639 s. These are completed, feasible local solutions but **not successful FBG fit acceptance** at the selected threshold.

There are 14 sticking attempts. All 14 converge and satisfy branch constraints; none passes the strict FBG gate. The worst total time is frame 23, return angle 7.5 degrees: the preferred sticking branch takes 0.880 s, 23 iterations, and 24 IVPs, with raw FBG RMS 3.296e-6 /mm. That triggers alternatives. The subsequently rejected sliding branch takes 34.659 s, 105 iterations, and 1,028 IVPs. Thus this long step is dominated by an alternative branch after fit-gate rejection, not by the selected sticking solve.

The Stage 1 estimates are identical between the two friction models. Maximum arc-length error is 0.04726 mm; maximum normal-direction error is 0.001340 degrees; maximum geometric shape RMS is 0.04773 mm. These are small but nonzero despite noiseless sampled observations. This run does not isolate whether the rejected fits are dominated by discretization, frozen geometry, previous-shape history, or local optimization. No threshold adjustment, geometry repair, truth-assisted restart, or hidden removal of outliers was performed.

## Reference physics and hysteresis

All 25 reference states pass the independent physics audit. Maximum absolute gap: 1.35e-9 mm; tangency: 3.79e-11; circular-cone projection residual: 7.24e-13 N; terminal moment: 5.20e-12 N mm. No dense-mesh penetration is detected. Maximum integrated material twist magnitude is 3.755 degrees, even though base rotation reaches 45 degrees; these are different quantities.

Reference contact force [Fx,Fy,Fz] N:

- Start at 0 degrees: [11.45639, 0, -21.27616].
- At 45 degrees: [10.91478, -1.20449, -20.04525].
- Return to 0 degrees: [13.95227, -0.00865, -15.92707].

The return path sticks and preserves frictional history, so the same final base pose does not imply the same contact load or shape as the initial state. The maximum reference force cone ratio is respected by the stick/slip audit.

For the inverse solutions, the maximum projection residual is 1.58e-12 N (mu=0.3), but Stage 2 shape/geometry mismatch remains visible: maximum dense penetration is 2.40e-5 mm and maximum tangency residual is 7.99e-4. Mu=0 reaches 2.97e-4 mm penetration and 2.68e-3 tangency. Diagnostics report these without terminating or modifying the trajectory. Videos retain fit-failure and physics-warning labels.

## Solver settings and provenance

The conditional scheduler and mode-branch solver are identical by SHA-256 to the prior insertion run. It tries the previous matched mode first and stops at the first positive native exit with scaled branch violation <= 1e-6 and whitened FBG RMS <= 0.01 (raw <= 1e-6 /mm). Otherwise it tries all modes and returns the recorded fallback. Full nonlinear 3D mechanics and the exact circular friction cone remain. Local SQP does not certify a global solution.

Unchanged settings: sensor standard deviation 1e-4 /mm (covariance 1e-8); sensor-only MAP, no force priors; objective multiplier 1e-4; fmincon SQP; 300 iterations / 5,000 evaluations per branch; optimality tolerance 1e-8; step tolerance 1e-12; nonlinear constraint tolerance 1e-6; IVP relative/absolute tolerances 2e-8/2e-10. Normal/gap/friction row scaling is unchanged. The observation distribution is noiseless; finite estimation covariance does not imply injected noise.

## Outputs and reproduction

Paths relative to `two_stage_force_sensing/`:

- Updated insertion video: `results/published/inclined_wall_modes_conditional_1e6_20261010/video_forces/insertion_force_comparison.mp4`.
- Rotation video and preview frames: `results/published/inclined_wall_rotation_20261010/video/`.
- Saved insertion image: `results/published/inclined_wall_rotation_20261010/maximum_insertion_frame.png`.
- Full loaded checkpoint: `results/runs/inclined_wall_rotation_20261010/maximum_insertion_seed.mat`.
- Rotation estimates, all branch attempts, reference audit, summary and comparison plot: `results/published/inclined_wall_rotation_20261010/`.
- Full input/reference/estimator states: `results/runs/inclined_wall_rotation_20261010/`.

MATLAB, from this package, using the existing read-only baseline dependency:

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
save_inclined_rotation_seed;
build_inclined_rotation_truth;
run_inclined_rotation_modes;
```

These wrappers target the recorded run tag; choose a new tag in the wrappers before another experiment to preserve these artifacts. The first rotation state is restored, not re-solved.

```text
python experiments/inclined_wall/analyze_inclined_rotation.py
python experiments/inclined_wall/publish_force_video.py --case insertion
python experiments/inclined_wall/publish_force_video.py --case rotation
```

Analysis checks checkpoint continuity, 50 saved states, all branch iteration/IVP totals, previous-mode-first ordering, immediate stopping on accepted fits, exhaustive fallback after rejection, and raw/normalized FBG residual consistency. The renderer verifies saved shape and force errors against the CSV before encoding. Both videos plot all three force components, with a moving frame cursor.

Publication validation: both H.264 videos decoded completely at 1920x1440 and 12 fps. Insertion: 328 encoded frames / 27.33 s. Rotation: 200 encoded frames / 16.67 s. The maximum-insertion, 45-degree, and final return frames were visually inspected; force panels, state labels and warnings are legible. Decode checks are saved beside each video.
