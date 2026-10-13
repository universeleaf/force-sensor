# Why 2 mm geometric FBG noise produces N-scale force uncertainty

Date: 2026-10-12. Author: Codex, mentor-side collaborator. Branch: `codex/two-stage-force-sensing`, base b376013. Offline diagnostic only; no solver, physics, acceptance threshold, or saved trajectory was changed.

## Main finding

The calibrated curvature noise predicts **3.60 N insertion and 2.73 N rotation tip-force RMS uncertainty**, even conditional on the clean trajectory's fixed contact geometry, mode, and previous shape. Tip/contact compensation weakens the observation response substantially. Rotation also has additional mode/history/geometry effects: its full observed 4.99 N RMS is not explained entirely by this fixed-mode Gaussian calculation.

The 2 mm label means RMS 3D endpoint displacement after integrating noisy Bishop curvature. It is not a force accuracy specification or independent 2 mm noise at each shape sample. The injected noise is independent Gaussian in 24 stations x 2 channels, with sigma = 3.7072566884243936e-4 mm^-1 per channel. No environmental or friction-coefficient noise was injected.

## Method and mathematical interpretation

Saved `Jprediction`, `JtipMoment`, and contact/shape position Jacobians are derivatives of the **nonlinear Cosserat IVP**, although the optimization used successive linearization. All dimensional Jacobians were converted from scaled decision coordinates into physical units.

Write x = [base moment; tip force; contact force], with contact force expressed in world coordinates. Linearizing the zero terminal-moment boundary condition eliminates the nuisance base moment:

    delta m0 = -E_m^{-1} E_f delta f
    delta w = G delta f,
    G = J_f - J_m E_m^{-1} E_f,
    delta f = [delta F_tip; delta F_contact].

Contact constraints are linearized at the saved solution, with normal, arclength, and previous shape held fixed. Let P = d p_contact / d f, v = B' (p_contact - previous_p_contact), d = v / ||v||. For sliding, the tangent constraints are

    n' P delta f = 0,
    delta tau + mu d delta lambda
      + mu lambda / ||v|| (I - d d') B' P delta f = 0.

For sticking, P delta f = 0. For separation, delta F_contact = 0. Let C collect these constraints, N span null(C), and H = G N. The local least-squares response and covariance are

    delta f_hat = K epsilon,  K = N pinv(H),
    Cov(delta f_hat) = sigma^2 K K'.

Reported tip RMS = sqrt(trace(tip covariance)); aggregate trajectory predictions are the square root of the mean squared per-frame RMS. The force predictions use the actual stored 48-channel noise vectors, not new random draws. This is a conditional Gauss-Newton/local observability calculation, not a full nonlinear posterior or a repeated Monte Carlo inverse-solver experiment. Other inactive inequality boundaries and finite-noise mode changes are not modeled by this tangent covariance. A sliding/sticking transition can invalidate it. No priors were added.

## Trajectory comparison

Rotation excludes its restored insertion seed, so there are 24 new rotation states versus 41 insertion states. Observed results are the previous exhaustive-mode runs with 2 mm noise.

| Quantity | Insertion | Rotation |
|---|---:|---:|
| Clean-run tip force RMS error | 0.00768 N | 0.0976 N |
| Predicted local tip RMS, fixed clean geometry/history/mode | **3.599 N** | **2.725 N** |
| Linear response to the actual noise draw, all frames | 3.132 N | 2.301 N |
| Observed full noisy tip force RMS | 3.335 N | 4.985 N |
| Observed RMS, correct-mode and converged subset | 2.592 N (29 frames) | 3.410 N (14 frames) |
| Clean-Jacobian prediction for the same subset's noise draw | 2.688 N | 2.331 N |
| Predicted vs observed X error correlation on that subset | 0.960 | 0.833 |
| Known-contact-force observation-only local tip RMS | 0.697 N | 0.653 N |

The known-contact calculation fixes contact load and uses FBG plus terminal moment; it is a diagnostic counterfactual, not another implemented estimator. It omits contact-position constraints and thus is not an exactly matched experiment. Even this comparison shows that estimating the unknown contact load together with tip load increases uncertainty strongly.

The correct-mode subset is selected after scoring against truth; it does not isolate a causal noise-only contribution. The remaining error in rotation cannot be assigned numerically to geometry versus history versus optimization from this calculation alone.

## Which force direction is weak?

At clean reference states, median conditional component standard deviations (N) are:

| Scenario | X | Y | Z |
|---|---:|---:|---:|
| Insertion | 3.334 | 0.118 | 0.554 |
| Rotation | 2.570 | 0.806 | 0.361 |

Y uncertainty reaches 1.543 N at 45 degrees, but **X remains the largest uncertainty direction in this fixture**. This differs from earlier rotation experiments with another geometry. In the observed correct-mode subset the X/Y/Z force-error RMS values are [2.554, 0.106, 0.427] N for insertion and [3.222, 1.025, 0.442] N for rotation.

## Concrete load-compensation directions

For each axis, minimize ||G delta f|| subject to the local mode constraints and a unit change of that tip-force component. Other force components can compensate. These are infinitesimal tangent directions; a finite change is not certified as a fully nonlinear feasible solution.

Insertion frame 24, reference contact arclength 106.856 mm (11 distal FBG stations):

    delta F_tip     = [1, 0, 0.07335] N
    delta F_contact = [-0.89690, 0, 0.48294] N

This changes observation RMS by only **1.744e-5 mm^-1 per N**, versus 1.792e-4 for a tip-X-only perturbation. The tangent direction respects zero tip moment, contact gap, and sliding friction to first order. Predicted whole-shape RMS change is only 0.01958 mm per N. The unconstrained tip-X/contact-X Jacobian cosine similarity is 0.9904.

The full 48-channel signal norm per N is sqrt(48) x 1.744e-5 = 1.2085e-4 mm^-1/N. Thus the profiled tip-X standard deviation is

    sigma_Fx = 3.7073e-4 / 1.2085e-4 = 3.068 N.

A 3 N shift along this weak direction has full-vector signal-to-noise norm about 0.98. Do not compare only per-channel RMS to sigma and forget the sqrt(48) factor.

Rotation frame 13 (45 degrees), reference contact arclength 104.504 mm:

    delta F_tip     = [1, 0.67086, 0.11277] N
    delta F_contact = [-0.49490, -0.76795, 1.18891] N

Observation RMS response is 2.344e-5 mm^-1 per unit tip-X N, versus 3.615e-4 for tip-X alone. Shape RMS response is 0.02509 mm per unit tip-X N. The corresponding profiled sigma_Fx is 2.283 N. Rotation requires a genuinely 3D compensation direction; same-axis X correlation alone (0.683) does not describe the coupling.

## An actual correctly solved large-error frame

Insertion frame 24: mode sliding, correct, feasible and converged.

| Force (N) | Truth | Noisy estimate |
|---|---|---|
| Tip | [1, 0, 0] | [9.7361, -0.0145, 0.3846] |
| Contact | [12.6079, 0, -6.7888] | [4.4998, 0.0637, -2.4249] |

Tip error = 8.7446 N, contact error = 9.2080 N, yet shape RMS error = 0.3262 mm and normalized FBG residual RMS = 1.0755. The actual noise vector predicts a tip error of 8.2428 N using the clean-state Jacobian, and 8.5868 N using the noisy-state Jacobian. This example is strong evidence of force observability limits, not merely a wrong-mode failure. Its estimate differs from the clean observation by a full-vector whitened norm 2.836, so this particular large excursion should not be described as below one sigma.

## Why rotation has extra error

At rotation frame 3, physical sliding was replaced by a feasible separation solution after the correct branch failed to return feasible. Tip error is 18.238 N; local noise response around the clean, correct-mode state predicts only 0.720 N for that draw. This is a branch/solver failure contribution, not ordinary local Gaussian amplification. Its residual RMS 1.306 exceeds the 1.24 gate; the returned fallback should not be called an accepted accurate estimate. Excluding this one frame, rotation's full noisy tip RMS is 3.387 N.

Even on correct-mode converged rotation frames, observed RMS is 3.410 N versus the clean local actual-draw prediction of 2.331 N. Stage 1 arclength RMS error was approximately 0.514 mm, and previous estimated shape enters the friction displacement. Clean rotation slip is as small as 0.000660 mm at frame 3 and 0.008567 mm at 45 degrees. These very small increments make slip direction/history vulnerable to geometric errors; the local fixed-history covariance excludes that feedback. This is a mechanism consistent with the results, not a measured causal partition of the remaining variance.

## Mechanical interpretation

Contact load acts upstream of contact through internal force/moment jumps; tip load acts over the whole rod. Their effects are therefore not identical. However, upstream bending can nearly cancel under a coupled load change, and downstream bending has a smaller remaining moment arm. Additionally, a force component nearly along the distal tangent/chord produces little bending; the inextensible model and Bishop bending observations provide no axial-strain measurement. The remaining observation difference is small compared with the injected curvature noise. Full contact constraints reduce the admissible directions but do not remove all weak directions.

Insertion's active contact arcs span roughly 100.17-134.87 mm; rotation's span 100.19-104.50 mm. This is not simply a case of having only one distal sensor: the representative frames have 11 distal stations, yet compensation still makes the net bending signature weak. Accurate shape reconstruction does not imply accurate decomposition into tip and contact load, and integrating the same FBG data into shape adds no independent information.

## Validation and reproduction

Run from the package root:

    python experiments/inclined_wall/audit_noisy_force_sensitivity.py
    matlab -batch "setup_tsfs; addpath(fullfile(pwd,'experiments','inclined_wall')); validate_noisy_force_sensitivity"

The Python audit covers 130 saved states (clean/noisy, 41 insertion + 24 rotation each), enforces tangent-constraint algebra numerically, and produces frame/direction tables, covariance summaries, and the figure. Eight exact nonlinear IVP calls verify two representative X profile derivatives at steps 0.01 and 0.001 N. Maximum relative finite-difference error is **1.60e-8**. No inverse solves were rerun.

Outputs: `results/published/noisy_force_sensitivity_20261012/`: `sensitivity.png`, `frame_sensitivity.csv`, `direction_sensitivity.csv`, `summary.json`, `derivative_inputs.json`, `derivative_checks.csv`, `validation.log`.

Limitations: local linear response; fixed mode, geometry, and history; no environment-noise propagation; no nonlinear posterior or force Monte Carlo; force positivity/cone transitions can truncate Gaussian predictions. These qualifications do not alter the central finding that N-scale uncertainty already exists within the correct smooth branch.
