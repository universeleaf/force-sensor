# Multi-contact software benchmark

Generated from `comparison.json` by `scripts/summarize_multi_contact_benchmark.py`.
State: `complete`; 396 estimates; 0 failed estimates.
Forward truth is independently solved planar, frictionless rod equilibrium in fixed two- or three-contact channels. Every comparison pair uses the same sparse curvature packet, stiffness and base pose. Both methods are told contact count/order; only EnFiRCE receives plane points/normals. These are internally generated scenes, not original-paper benchmark datasets.

Contact force RMSE is the root mean square Euclidean vector error over contacts and frames; total force RMSE uses the resultant including the independent tip load. Brackets are a descriptive 95% bootstrap interval resampling the three random seeds as clusters; two frames per seed are correlated and three seeds do not establish population confidence or superiority.
Review flags come from each method's own numerical/physical checks and are not an equivalent accuracy classifier across methods.

## Nominal noise: 24 FBG observations, 2.5e-5 /mm

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 0.490 [0.424, 0.539] | 0.006 | 0.620 | 0.929 | 0/18 |
| Shape-only point loads | 18/18 | 2.967 [2.431, 3.518] | 0.521 | 3.764 | 0.017 | 0/18 |

### s_channel_two_contact

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 6/6 | 0.529 [0.060, 0.856] | 0.004 | 0.667 | 0.836 | 0/6 |
| Shape-only point loads | 6/6 | 2.505 [1.203, 3.403] | 0.556 | 2.021 | 0.015 | 0/6 |

### tapered_channel_two_contact

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 6/6 | 0.573 [0.204, 0.741] | 0.008 | 0.750 | 0.462 | 0/6 |
| Shape-only point loads | 6/6 | 1.986 [1.680, 2.375] | 0.447 | 2.511 | 0.022 | 0/6 |

### serpentine_three_contact

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 6/6 | 0.336 [0.199, 0.448] | 0.007 | 0.379 | 1.489 | 0/6 |
| Shape-only point loads | 6/6 | 4.023 [3.669, 4.461] | 0.553 | 5.667 | 0.015 | 0/6 |

## Sensor density and noise

### Noise 0 /mm; 8 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 5.90e-10 [5.90e-10, 5.90e-10] | 0.000 | 0.000 | 0.748 | 0/18 |
| Shape-only point loads | 18/18 | 26.746 [26.746, 26.746] | 3.829 | 45.449 | 0.038 | 6/18 |

### Noise 0 /mm; 16 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 9.13e-12 [9.13e-12, 9.13e-12] | 0.000 | 0.000 | 0.272 | 0/18 |
| Shape-only point loads | 18/18 | 1.509 [1.509, 1.509] | 0.103 | 2.396 | 0.022 | 0/18 |

### Noise 0 /mm; 24 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 4.88e-12 [4.88e-12, 4.88e-12] | 0.000 | 0.000 | 0.235 | 0/18 |
| Shape-only point loads | 18/18 | 1.088 [1.088, 1.088] | 0.085 | 2.140 | 0.015 | 0/18 |

### Noise 2.5e-05 /mm; 8 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 13.576 [12.776, 14.256] | 0.102 | 16.722 | 1.534 | 6/18 |
| Shape-only point loads | 18/18 | 26.762 [26.685, 26.852] | 4.597 | 45.499 | 0.033 | 6/18 |

### Noise 2.5e-05 /mm; 16 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 0.837 [0.536, 1.097] | 0.010 | 1.001 | 0.987 | 0/18 |
| Shape-only point loads | 18/18 | 2.998 [2.504, 3.619] | 0.651 | 3.306 | 0.022 | 0/18 |

### Noise 2.5e-05 /mm; 24 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 0.490 [0.424, 0.539] | 0.006 | 0.620 | 0.929 | 0/18 |
| Shape-only point loads | 18/18 | 2.967 [2.431, 3.518] | 0.521 | 3.764 | 0.017 | 0/18 |

### Noise 5e-05 /mm; 8 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 15.904 [12.539, 17.795] | 0.178 | 18.942 | 1.902 | 8/18 |
| Shape-only point loads | 18/18 | 28.765 [27.217, 31.519] | 5.372 | 47.386 | 0.036 | 6/18 |

### Noise 5e-05 /mm; 16 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 1.813 [1.201, 2.199] | 0.020 | 2.200 | 1.545 | 4/18 |
| Shape-only point loads | 18/18 | 5.469 [4.496, 6.681] | 0.949 | 5.198 | 0.018 | 0/18 |

### Noise 5e-05 /mm; 24 observations

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 1.730 [1.589, 1.971] | 0.022 | 2.087 | 1.568 | 6/18 |
| Shape-only point loads | 18/18 | 5.321 [4.332, 5.987] | 1.046 | 6.081 | 0.020 | 0/18 |

## Geometry ablations at nominal noise

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| EnFiRCE (environment) | 18/18 | 0.490 [0.424, 0.539] | 0.006 | 0.620 | 0.929 | 0/18 |
| Shape-only point loads | 18/18 | 2.967 [2.431, 3.518] | 0.521 | 3.764 | 0.017 | 0/18 |
| No contact-gap penalty | 18/18 | 1.169 [0.417, 1.621] | 0.030 | 1.435 | 1.536 | 18/18 |
| No tangency penalty | 18/18 | 1.525 [0.754, 1.992] | 0.292 | 1.876 | 2.306 | 18/18 |
| No gap/tangency penalties | 18/18 | 1.499 [0.858, 1.938] | 0.311 | 1.830 | 2.180 | 18/18 |
| One plane offset by 1 mm | 18/18 | 31.092 [30.558, 31.556] | 1.476 | 44.259 | 2.472 | 18/18 |

At nominal noise, EnFiRCE has lower contact-vector error on 17/18 paired frames. This is an internal comparison, not evidence of SOTA against published systems.

## Interpretation boundary

These tests do not reproduce published multi-contact systems or compare them under identical sensing, hardware, contact-mode uncertainty and ground-truth conditions. The estimator assumes known contact count/order, planarity and frictionless point contacts. A one-millimetre plane-offset run probes calibration sensitivity; it does not establish a general robustness threshold. Solver times exclude forward-truth generation, MATLAB startup, video rendering and file I/O. Full run inputs and numerical outputs are saved in each scene's `truth.mat` and `trials.mat`.

## Plane-calibration sensitivity replay

The first plane point is displaced by +1 mm along its normal in the inverse input; the forward truth and curvature packets are unchanged. For the latent variants, the plane point offset is an estimated state with a zero-mean Gaussian prior, not the ground-truth displacement. These prior scales were fixed before scoring.

| Method | Completed | Contact force RMSE (N) | Contact arc RMSE (mm) | Total force RMSE (N) | Mean solver time (s) | Review |
|---|---:|---:|---:|---:|---:|---:|
| Shifted plane, treated as exact | 18/18 | 31.092 [30.558, 31.556] | 1.476 | 44.259 | 2.575 | 18/18 |
| Latent plane offset, prior SD 0.1 mm | 18/18 | 21.163 [20.698, 21.823] | 0.936 | 29.445 | 2.700 | 18/18 |
| Latent plane offset, prior SD 1 mm | 18/18 | 1.341 [0.412, 2.220] | 0.026 | 1.625 | 1.929 | 2/18 |

Latent plane offset, prior SD 0.1 mm: mean inferred first-plane offset -0.283 mm (true correction −1 mm).
Latent plane offset, prior SD 1 mm: mean inferred first-plane offset -1.003 mm (true correction −1 mm).
