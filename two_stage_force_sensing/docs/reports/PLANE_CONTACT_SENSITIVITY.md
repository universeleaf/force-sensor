# Contact sensitivity to environmental geometry: single contact and S-channel

Historical report from the pre-migration workspace. Current entry points are in [README](../../README.md); preserved evidence is in [results](../../results/README.md). Original path references and timings describe that historical run.

2026-10-05. Both existing estimators are unchanged. This is a forward physical sensitivity experiment, supplemented by observation sensitivities, not another inverse-estimator benchmark.

## Method and interpretation

I re-solved three-dimensional Cosserat equilibrium while holding the calibrated rod, base pose, and applied tip force fixed. All contact arc lengths, contact points, normal reactions, and base moment were allowed to respond. The equations impose terminal moment zero, closed active contacts, and smooth contact tangency. The single-contact case retains its prescribed sliding branch; the S-channel case is frictionless. This does not test friction-mode transitions or establish global mechanical stability.

At each of the 12 frames of both datasets, the unknowns are q=(base moment, normal reactions, contact arc lengths). For equilibrium residual R(q,theta)=0, sensitivities use dq/dtheta=-(dR/dq)^(-1)(dR/dtheta), with full contact-output differentiation. Independent positive and negative re-equilibrations validate the first frame, including full +/-1 mm normal wall displacement. Contact branches remained admissible in those tests. Tip force is fixed during geometry perturbations; tip-y force is varied only in a separate compliance/observability test.

These forward force changes are NOT inverse force-estimation errors with a fixed measured shape. They quantify how the real equilibrium changes when the actual wall moves. The previous inverse-geometry audit addresses the complementary experiment.

Geometry changes are defined precisely:

- d_j: translate plane j by d_j*n_j, along its inward free-space normal, in mm. Positive d intrudes into the free region.
- alpha_j: tilt its unit normal toward +y: n_j(alpha)=cos(alpha)*n_j+sin(alpha)*e_y, in radians, keeping its reference point fixed.
- F_tip,y: additional applied out-of-plane tip force, in N.

The single plane has normal -z. S-channel walls are x=-10 and x=+10 mm, with normals +x and -x. Their spacing is 20 mm. Both are infinite planes in the model: the channel confines x, not y.

## Translating an infinite plane in y has no physical effect

For g=n^T(p-p0), a reference-point perturbation changes g by -n^T delta_p0. Here n_y=0. Translating either plane in y changes only its parameterization, not the plane itself. Direct +1 mm y-translation tests gave exactly zero change in contact positions and residuals for both scenarios.

Thus dp_c/d(p0_y)=0 does not diagnose instability or poor force observability. To investigate the meaningful y response, use plane-normal tilt or actual y loading. A finite wall/channel with y boundaries would require a different geometry model.

## Normal wall-displacement sensitivity

First-frame derivatives below include the response of ALL contacts. Position components and arc derivatives are mm/mm; force derivatives are N/mm.

| Scenario / perturbed wall | Contact | ds/dd | dp_x/dd | dp_y/dd | dp_z/dd | dF_n/dd |
|---|---:|---:|---:|---:|---:|---:|
| Single plane | 1 | -0.7732 | +1.4718 | 0 | -1 | +0.6158 |
| S left wall | 1 | -1.8325 | +1 | 0 | -1.5692 | +57.2711 |
| S left wall | 2 | -1.7621 | 0 | 0 | -1.0901 | +6.5593 |
| S right wall | 1 | +0.2344 | 0 | 0 | +0.2404 | +6.5593 |
| S right wall | 2 | -1.0281 | -1 | 0 | -0.6796 | +2.0411 |

Equivalently, for the S-channel:

    ds/dd  = [ -1.8325   +0.2344
               -1.7621   -1.0281 ]  mm/mm

    dFn/dd = [ 57.2711    6.5593
                6.5593   2.0411 ]  N/mm

The S-channel is exceptionally sensitive in REACTION FORCE, especially at the left wall. The dominant force gain is 93.0 times the single-contact value. Contact positions and arc lengths are more sensitive, but by factors of order one to a few, not 93. High reaction sensitivity should not be confused with extremely large displacement compliance.

Across all 12 frames, the single reaction slope is 0.538–0.616 N/mm; the S first-reaction/left-wall slope is 46.73–57.27 N/mm. The S cross-wall force slope is 6.559–6.672 N/mm. The first S normal force ranges from 118.74 down to 64.77 N, compared with 3.679–5.978 N for the single case.

The nonlinear +1 mm checks confirm the scale:

- Single plane: normal force increases by 0.6046 N, from 3.6791 to 4.2836 N; arc shifts -0.7622 mm.
- S left wall: reactions increase by 63.6163 and 6.5389 N, from [118.7404,17.2364] to [182.3567,23.7753] N; arcs shift [-1.9456,-1.7224] mm.
- S right wall: reactions increase by 6.6342 and 2.1059 N; arcs shift [+0.2418,-1.0316] mm.

Under a local linear approximation, independent 1 mm standard deviations in the two normal wall offsets would produce approximately 57.6 N and 6.87 N reaction standard deviations; the single value would be 0.616 N. This is illustrative propagation, not a nonlinear Monte Carlo noise result. FBG observations could supply information that reduces geometric/force uncertainty in a joint inverse estimate.

## Out-of-plane normal tilt

For a +1 mrad (=0.0573 degree) tilt toward +y, first-order contact y shifts are:

| Tilted plane | Single contact dy mm | S contact 1 dy mm | S contact 2 dy mm |
|---|---:|---:|---:|
| Single | 0.02626 | — | — |
| S left | — | 0.006236 | 0.023344 |
| S right | — | 0.003389 | 0.031183 |

These are comparable displacement scales. The S-channel does not show a dramatic y-displacement amplification. By symmetry about the x-z plane, arc lengths and normal-force magnitudes have zero first derivatives with respect to this y tilt; they change at second order. The finite perturbations verify that behavior.

However, high normal loads amplify the y COMPONENT of force: delta_F_y approximately equals F_n*alpha. At 1 mrad, the single contact changes by 0.003679 N in y; S left and right reactions change by 0.118740 and 0.017236 N respectively. That creates a potentially important geometry/load ambiguity even though contact y motion is modest. It does not demonstrate y buckling.

## Applied tip-y compliance and sensing

With planes unchanged, the first-frame contact y response to applied tip F_y is:

| Case/contact | dp_c,y/dF_tip,y mm/N |
|---|---:|
| Single | 6.6913 |
| S contact 1 | 0.2908 |
| S contact 2 | 2.5580 |

The S contacts are LESS compliant in y under this loading. Across 12 frames the single value is 6.691–6.952 mm/N; S values are 0.291–0.335 and 2.543–2.558 mm/N. This does not support explaining the inverse method's large F_y error by unusually high physical y compliance.

The observation result is more revealing. For the original S packet (one bending channel), the first-order measured curvature response to F_y is exactly zero. With both bending channels on the same equilibrium, its response norm is 0.001718 /mm/N. The single packet already measures both channels and has norm 0.001846 /mm/N. Thus the two-channel signals are comparable; the original S input discards the channel that reveals this out-of-plane response.

The zero derivative is a local planar symmetry result. Higher-order effects, priors and other measurements can constrain F_y; this is not a proof that every y load is globally indistinguishable.

## Conditioning: the weak forward mode is in-plane

For the dimensionless equilibrium Jacobian, moments are scaled by EI/L, forces by EI/L^2, arcs and gaps by L, and tangency remains dimensionless. Condition numbers depend on scaling; the physical derivatives above are the primary comparison.

| Forward Jacobian block, first frame | Single | S-channel |
|---|---:|---:|
| In-plane coupled equilibrium/contact block | 183.9 | 9960.9 |
| Out-of-plane boundary-moment block | 1.414 | 1.323 |

The weakest full mode is in the in-plane normal-force/base-moment/contact-arc subsystem. There is no near-singular out-of-plane boundary map at the tested equilibrium. These are forward boundary-value sensitivities, distinct from the approximately 102 condition number of the earlier planar FBG-to-force inverse Jacobian. Neither number alone establishes mechanical stability; no energy second-variation or buckling analysis was performed.

## What this says about the two-stage estimator

1. The S-channel is a demanding geometry-to-force problem: millimeter wall uncertainty can mean tens of newtons of reaction uncertainty. Freezing a biased offset/arc estimate is consequently expensive.
2. The major spurious y tip estimate is not explained by unusually large forward y compliance. The original one-channel packet leaves that force direction unobservable to first order, and our generic 3-D optimization can exploit it to accommodate in-plane geometric mismatch.
3. Normal-direction uncertainty can generate substantial y force because F_n is large, even if contact motion remains small. This should be propagated or retained as uncertainty, rather than equating a Stage 1 point estimate with exact geometry.
4. A fair follow-up uses the same bending channels for both cases and separates displacement sensitivity, force sensitivity and observation identifiability. The simulations also differ in rod length, load and friction, so the numerical ratios are case-specific, not universal channel penalties.
5. This audit does not change the agreed two-stage structure. It identifies why better geometric uncertainty treatment and adequate out-of-plane sensing matter; it does not establish that another within-step correction loop is necessary.

## Verification and files

All 24 nominal roots reproduce independent saved truth to better than 3e-8 mm maximum position error in the first frames. First-frame positive/negative re-equilibrations retain nonnegative reactions and the same ordered contacts; maximum tested scaled residual is 1.7e-11 and sampled penetration below 4.5e-10 mm. Central-difference contact-position derivatives agree with the implicit results to approximately 1.3e-6 relative or better. Nonpenetration checks are sampled, and no global branch/stability claim is made.

Reproduce with `audit_plane_sensitivity` and `audit_plane_condition` in the sibling project's MATLAB path. `results/plane_sensitivity/local_derivatives.csv` contains every contact/parameter/frame derivative. The MAT file retains local models and all sensitivity records; JSON files retain finite perturbations, conditioning blocks and validation. No estimator or student-repository source was changed.
