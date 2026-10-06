# S-channel investigation: baseline versus two-stage implementation

Historical report from the pre-migration workspace. Current entry points are in [README](../../README.md); preserved evidence is in [results](../../results/README.md). Original path references and timings describe that historical run.

Audit date: 2026-10-05. This audit changes neither estimator. It adds isolated analysis scripts and outputs in this sibling project. All geometry-error statistics use the original 12-frame S-channel fixture; factorial mechanical ablations use its first frame without recurrent priors. Corrected geometry is an explicit oracle diagnostic, never an estimator input in the main replay.

## The baseline advantage

The planar baseline knows exact wall normals/points, contact count/order, a frictionless contact model, and planar motion/forces. It does **not** know contact arc lengths or tip force. Its seven unknowns are base moment, two normal forces, two arc lengths, and two tip-force components. Arc lengths are jointly refined with the forces through the mechanical model. Sparse PCHIP reconstruction is only an initializer.

It uses bounded `lsqnonlin`, with residuals for curvature (sigma 1e-7 /mm), terminal moment (scale 1e-3 N mm), contact gap (scale 1e-3 mm), and tangency (scale 1e-4). Gap/tangency are weighted residuals, not hard MPCC equalities. Normal forces have nonnegative bounds; the active contact set is assumed. There is no general friction-cone or separation search.

Our Stage 1 estimates arc lengths, normals and offsets from reconstructed curvature and an environment prior, then freezes them. Stage 2 estimates a fully 3-D force state, with exact contact projection equalities and sampled nonpenetration. It does not enforce tangency as an equality; tangency is a post-step diagnostic. The one-channel adapter adds no fictitious bending measurement, but this makes the comparison much less constrained than the planar baseline.

## Actual noise versus assumed covariance

The S-channel scene has injected curvature noise standard deviation **zero**. Comparing all 288 saved measurements with sampled truth gives exactly zero mean, standard deviation and maximum difference. Environment points and normals, base pose, and stiffness are also supplied without injected noise. True normals are +x/-x and wall x positions are -10/+10 mm.

Both curvature likelihoods use a numerical/model floor of **1e-7 /mm** despite noise-free observations. Our adapter additionally assigns zero-mean environmental deviation priors: point standard deviation **0.05 mm per Cartesian component**, normal standard deviation **0.001 per component**, approximately **0.0573 degrees per tangent angular coordinate**. No random perturbation was drawn from those priors. These values were introduced as modeling uncertainty; they are not actual scene errors. They make it possible for the geometric MAP to move correct walls to compensate for reconstruction bias.

The profiled Stage 1 also declares reconstruction floors of 0.001 mm for closure and 1e-5 for tangency. These are too optimistic to cover the observed sparse interpolation bias in this clean fixture. They do not imply that a 0.05 mm environment perturbation was injected.

## Stage 1 geometric errors

| Quantity | First contact, frame 1 | Second contact, frame 1 |
|---|---:|---:|
| True arc length mm | 31.4186986 | 99.4141872 |
| Estimated arc length mm | 31.5064139 | 99.3157654 |
| Arc error mm | +0.0877154 | -0.0984218 |
| Normal angular error degrees | 0.0006792 | 0.0261875 |
| Estimated plane offset shift mm | -0.0052737 | +0.0944898 |
| Estimated gap at true contact mm | +0.0053412 | -0.0999107 |

Across 12 frames, arc RMSE is 0.05638 / 0.04644 mm for the two contacts; maximum absolute arc error is 0.08969 / 0.09842 mm. Angular errors are at most 0.000679 / 0.026187 degrees. Offset-shift RMSE is 0.003656 / 0.045318 mm, with a second-wall peak of 0.09449 mm.

The negative gap at the true second contact is not actual penetration of the true wall: it means the estimated wall disagrees with the true shape by almost 0.1 mm. That disagreement enters the frozen Stage 2 model.

Noiseless sparse samples do not specify the continuous curvature between sensors. Reconstructing from the 24 sparse samples gives **0.08397 mm shape RMS error** on frame 1. Integrating that same sparse interpolation on a 0.1 mm grid gives **0.08583 mm**: a finer integration grid does not fix it. Supplying exact curvature at all original rod nodes reduces error to **0.001263 mm**. This isolates sparse interpolation as the dominant reconstruction error, rather than random noise or ODE tolerances. The original baseline starts from a similarly approximate reconstruction but can correct its arc lengths through mechanics.

## First-frame factorial ablation

The same first-frame observation and force prior are used in each row. Only the indicated frozen geometry values are restored to truth. Original-input tip error includes its unconstrained out-of-plane component. The two-channel control adds the synthetic rod's actual zero first-bending channel with the same 1e-7 /mm likelihood floor; it is additional information and is not presented as the original replay. Geometry estimates are held identical across those controls.

| Corrected quantities | One-channel FBG RMS | One-channel tip error N | Two-channel FBG RMS | Two-channel tip error N | Two-channel contact error N |
|---|---:|---:|---:|---:|---:|
| None | 99.892 | 3.2351 | 145.5 | 0.074007 | 0.165 |
| Arc only | 26.878 | 3.0878 | 118.74 | 0.04762 | 0.19774 |
| Normal only | 99.373 | 3.1319 | 140.18 | 0.073484 | 0.16191 |
| Arc + normal | 21.902 | 3.0014 | 113.02 | 0.047395 | 0.20046 |
| Offset only | 87.114 | 1.169 | 64.48 | 0.021681 | 0.2807 |
| Arc + offset | 2.3177 | 0.78675 | 8.824 | 0.0011137 | 0.030253 |
| Normal + offset | 86.99 | 0.89473 | 62.568 | 0.07278 | 0.35483 |
| Arc + normal + offset | 0.10082 | 0.071242 | 0.00088143 | 1.7122e-05 | 1.8714e-05 |

These are interacting factors, not an additive variance decomposition. Correcting normal alone has little effect. Offset is important for mechanical consistency; even with normal/offset exact, freezing the wrong arcs leaves substantial FBG mismatch. Correcting arc and offset leaves a much smaller orientation-driven error, but it is still measurable against the very tight clean-data likelihood. All three corrected plus the second bending channel converges in four SQP steps with about 1.9e-5 N contact error and 1.7e-5 N tip error.

On the original one-channel input, all geometry corrected still leaves 0.07124 N tip error, almost entirely in y. With the current estimated geometry the first-frame tip estimate is approximately [0.0454, -3.2082, -0.4928] N, versus true [0.1, 0, -0.08] N. The large y component therefore dominates the reported 3.235 N error. Removing this ambiguity does not make the wrong geometry consistent: the two-channel current-geometry control reaches its iteration cap with about 0.0043 mm contact/gap violation.

## Force conditioning and amplification

At the true planar equilibrium, after eliminating base moment through terminal equilibrium, the FBG sensitivity to [tip_x, tip_z, normal_1, normal_2] has singular values approximately [2.421e-3, 3.183e-4, 1.612e-4, 2.383e-5] /mm/N. Its condition number is **101.6**. It is sensitive, but full rank. These are local FBG-only force sensitivities, not a global identifiability guarantee or a covariance including hard geometric constraints.

Adding tip_y produces an additional singular value of about 2.6e-21; its FBG Jacobian column at the planar solution is exactly zero. Thus tip_y is **unobservable to first order** with this one measured bending channel. Higher-order effects and the prior can constrain its magnitude; this does not mean every y force is globally indistinguishable. It does explain the weakness of a generic 3-D solve with this packet and its sensitivity to geometry, regularization and stopping tolerances.

A local least-squares compensation calculation, perturbing each fixed arc and retaining terminal equilibrium, gives the following force response. These values do not additionally impose the fixed-wall gap constraints; they quantify how force can absorb arc error in FBG fitting.

| Force component | dF / ds1 N/mm | dF / ds2 N/mm |
|---|---:|---:|
| Tip x | +0.502 | +0.741 |
| Tip z | -0.583 | -0.885 |
| Normal 1 | -3.574 | +0.436 |
| Normal 2 | +1.216 | +0.957 |

A 0.1 mm arc error can therefore induce several tenths of a newton of contact-force bias and tip-force changes comparable to the true 0.1 / -0.08 N components. This supports the proposed coupling mechanism, but it is not the sole cause: the wrong wall offset and the missing bending channel are separate contributors.

## Conclusions and next controlled comparison

The normal estimates are already quite accurate. The dominant observed failures originate from sparse-shape bias being absorbed into plane offsets and arc locations, then frozen, with one-channel out-of-plane ambiguity amplifying force errors. Small normal errors matter for high precision but do not explain the large failures alone.

The previous comparison was not like-for-like. A fair clean-data comparison should hold the truly exact wall geometry fixed in both methods and use the same force/sensor subspace, then separately test uncertainty with explicitly injected noise. Jointly learned arc lengths must be distinguished from frozen Stage 1 arcs. Improving sparse reconstruction/model-error representation is necessary before attributing all difficulties to complementarity or force-tip singularity.

Iteration-limit/stagnation is not proof of mathematical infeasibility, nor was explosive divergence established. The SQP remains local and tolerance-sensitive. These observations identify data/model inconsistency and weak observability, not a global optimality theorem. No estimator changes were made in this investigation.

Sources: copied baseline `reference/rod/estimate_planar_multi_contact.m`, fixture generator `build_multi_contact_demo_truth.m`, scene `multi_contact_demo_scenes.m`, and new adapter/reconstruction/geometry/SQP under `matlab/+tsfs/`. Reproduce with `audit_s_channel` and `audit_s_channel_two_axis`. Raw results are in `results/s_channel_investigation/`.
