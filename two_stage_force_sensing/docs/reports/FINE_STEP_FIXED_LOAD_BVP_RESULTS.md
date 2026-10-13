# Five-times-finer single-pass rod plus fixed-load BVP

Completed report date: 2026-10-13 (run stamp preserves the start date). Branch codex/two-stage-force-sensing, base b376013. Eight complete trajectories / 1284 new solves. All outputs are separate; no commit or push.

Insertion distance step is 0.4 mm (previously 2 mm), producing 201 states over 80->40->80 mm. Rotation step is 0.75 degrees (previously 3.75), producing 121 states over 0->45->0 degrees; its first restored insertion state is excluded from scores. Both reference trajectories were regenerated with their actual finer-step friction histories using the existing collaborator algorithms read-only. No interpolation of ground truth was used. Finer-step friction history can change the physical reference itself, especially around reversal.

The inverse method is unchanged: one rod linearization; exhaustive full circular separation/sticking/sliding search; maximum 100 SQP iterations per branch; one fixed-load bvp4c call; independent nonlinear IVP validation; corrected history passed forward. No outer relinearization/force-correction loop was added. Constraint tolerance 1e-6; BVP scaled RelTol 1e-7, AbsTol 1e-9; no force priors or trust box. Geometry/collision spatial discretization remains unchanged. Timestamps remain 0.02 s apart as quasistatic history bookkeeping; this test changes actuation increment, not a validated dynamic model.

Gaussian Bishop curvature standard deviations remain 0, 9.26814e-5, 1.85363e-4, 3.70726e-4 /mm for 0/0.5/1/2 mm targets. Zero injected noise retains measurement covariance std 1e-4 /mm. At common actuation samples the original seed33001 draws are reused exactly; additional samples use independent seed33005 Gaussian draws. Restored rotation noise equals insertion noise at maximum insertion. No averaging or interpolation of noise was performed; environment and mu noise remain absent.

Checkpoint serialization occurs every ten frames and at completion, retaining all frame states, branches and diagnostic values. Redundant dense IVP interpolation objects are omitted after diagnostics, as the existing runner already did for final shapes. This affects storage, not numerical execution. Reported solve times exclude checkpoint serialization and forward truth generation.

## Accuracy and runtime

Errors are RMS vector norms; shape error pools centerline sample points and frames. Coarse values refer to the previous fixed-load BVP method. Fine values use all fine frames.

| Motion | Noise mm | Coarse tip N | Fine tip N | Coarse contact N | Fine contact N | Coarse shape mm | Fine shape mm | Coarse s/frame | Fine s/frame |
|---|---|---|---|---|---|---|---|---|---|
| insertion | 0 | 0.01044 | 0.01499 | 0.04788 | 0.02327 | 0.12 | 0.0127 | 0.7993 | 0.8657 |
| rotation | 0 | 0.4235 | 0.1087 | 1.97 | 0.3176 | 0.04378 | 0.01343 | 0.2469 | 0.2394 |
| insertion | 0.5 | 0.7502 | 0.8939 | 0.8243 | 0.9947 | 0.236 | 0.1248 | 0.7789 | 0.4583 |
| rotation | 0.5 | 0.9405 | 2.239 | 2.759 | 5.465 | 0.1379 | 1.082 | 0.2634 | 0.2763 |
| insertion | 1 | 1.529 | 1.816 | 1.723 | 2.058 | 0.4736 | 0.299 | 0.8394 | 0.4496 |
| rotation | 1 | 2.056 | 2.801 | 5.163 | 5.576 | 0.6438 | 0.728 | 0.2698 | 0.2905 |
| insertion | 2 | 3.391 | 4.26 | 3.84 | 6.051 | 1.039 | 7.249 | 0.759 | 0.4469 |
| rotation | 2 | 3.755 | 8.006 | 8.038 | 10.9 | 0.9475 | 8.588 | 0.3426 | 0.3545 |

## Matched actuation samples and noise realization

This table evaluates the fine trajectory only at every fifth sample, corresponding to the coarse actuation positions and identical noise draws. Histories and reference contact states may still differ. Realized geometric noise is endpoint displacement between noisy and clean geometric reconstructions on the fine trajectory.

| Motion | Noise mm | Common samples | Coarse tip N | Fine common tip N | Coarse contact N | Fine common contact N | Realized tip-noise RMS mm |
|---|---|---|---|---|---|---|---|
| insertion | 0 | 41 | 0.01044 | 0.004416 | 0.04788 | 0.003872 | 0 |
| rotation | 0 | 24 | 0.4235 | 0.08352 | 1.97 | 0.2263 | 0 |
| insertion | 0.5 | 41 | 0.7502 | 0.75 | 0.8243 | 0.8187 | 0.5185 |
| rotation | 0.5 | 24 | 0.9405 | 2.866 | 2.759 | 5.498 | 0.5286 |
| insertion | 1 | 41 | 1.529 | 1.521 | 1.723 | 1.716 | 1.034 |
| rotation | 1 | 24 | 2.056 | 1.501 | 5.163 | 4.089 | 1.055 |
| insertion | 2 | 41 | 3.391 | 3.91 | 3.84 | 7.628 | 2.076 |
| rotation | 2 | 24 | 3.755 | 6.81 | 8.038 | 9.391 | 2.111 |

## Peak errors

| Motion | Noise mm | Coarse peak tip N | Fine peak tip N | Coarse peak contact N | Fine peak contact N | Coarse peak shape mm | Fine peak shape mm |
|---|---|---|---|---|---|---|---|
| insertion | 0 | 0.05553 | 0.1806 | 0.2683 | 0.3104 | 0.7242 | 0.174 |
| rotation | 0 | 1.385 | 0.4792 | 5.731 | 1.321 | 0.09363 | 0.04627 |
| insertion | 0.5 | 2.125 | 4.209 | 2.209 | 4.223 | 0.7976 | 0.4515 |
| rotation | 0.5 | 2.825 | 12.67 | 7.563 | 14.49 | 0.3307 | 2.84 |
| insertion | 1 | 4.238 | 8.429 | 4.414 | 8.455 | 1.129 | 1.781 |
| rotation | 1 | 5.954 | 11.48 | 10.95 | 13.84 | 1.088 | 4.56 |
| insertion | 2 | 9.177 | 21.94 | 10.47 | 42.42 | 3.019 | 65.85 |
| rotation | 2 | 9.06 | 33.77 | 13.97 | 33.46 | 1.885 | 49.66 |

Peak errors are reported because a smaller step does not necessarily remove a friction-force jump at reversal; averaging over more samples can reduce trajectory RMS without eliminating the peak. Noise maxima also depend on sample count.

## Convergence and physical consistency

| Motion | Noise mm | Affine success | BVP success | Full mode success | Max terminal moment N mm | Max active gap mm | Max penetration mm | Positive-work frames |
|---|---|---|---|---|---|---|---|---|
| insertion | 0 | 201/201 | 201/201 | 28/201 | 1.127e-06 | 0.1286 | 0.001676 | 0 |
| rotation | 0 | 120/120 | 120/120 | 0/120 | 1.564e-07 | 0.002526 | 0.002531 | 70 |
| insertion | 0.5 | 199/201 | 201/201 | 33/201 | 9.219e-07 | 0.1614 | 0.1616 | 0 |
| rotation | 0.5 | 120/120 | 120/120 | 0/120 | 4.385e-07 | 0.9646 | 0.1411 | 49 |
| insertion | 1 | 200/201 | 201/201 | 44/201 | 7.932e-07 | 0.7478 | 0.7559 | 0 |
| rotation | 1 | 120/120 | 120/120 | 0/120 | 3.812e-07 | 2.997 | 3.135 | 53 |
| insertion | 2 | 200/201 | 200/201 | 63/201 | 641.1 | 70.15 | 73.99 | 2 |
| rotation | 2 | 119/120 | 118/120 | 1/120 | 686 | 10.11 | 18.16 | 31 |

The normalized FBG acceptance level remains 1.24. Exhaustive search still returns the lowest-objective affine-feasible candidate when no candidate meets all checks; diagnostics do not stop the step. FBG threshold exceedances are recorded in summary.csv. Full mode success requires native convergence and original nonlinear selected-mode constraints at tolerance 1e-6. BVP convergence alone enforces the prescribed-load equilibrium boundary problem. Gap/slip/friction remain diagnostic after correction. Positive-work counts use the existing displacement-based 1e-8 N mm diagnostic threshold. All exceptions and warnings are preserved in frames.csv.

BVP nonconvergence is counted separately from thrown exceptions: a bvp4c mesh-limit warning can return an approximate state without raising an exception. The current wrapper still audits and passes that state into history, while marking convergence false. Such frames are retained in every accuracy statistic; no fallback or retry was introduced in this experiment.

## Cost and iteration cap

| Motion | Noise mm | Fine total solve s | Coarse total solve s | IVPs/frame | SQP iterations/frame | Selected cap frames | Mean BVP s |
|---|---|---|---|---|---|---|---|
| insertion | 0 | 174 | 32.77 | 3 | 37.9 | 0 | 0.04017 |
| rotation | 0 | 28.73 | 5.926 | 3 | 56.32 | 0 | 0.04769 |
| insertion | 0.5 | 92.11 | 31.93 | 3 | 85.4 | 2 | 0.03964 |
| rotation | 0.5 | 33.16 | 6.322 | 3 | 76.22 | 0 | 0.04872 |
| insertion | 1 | 90.37 | 34.42 | 3 | 89.4 | 1 | 0.04101 |
| rotation | 1 | 34.86 | 6.476 | 3 | 86.7 | 0 | 0.04906 |
| insertion | 2 | 89.82 | 31.12 | 3 | 93.01 | 1 | 0.05913 |
| rotation | 2 | 42.54 | 8.223 | 2.992 | 98.16 | 1 | 0.06969 |

Total branch solves: 3710; all branches at iteration cap: 287. Exhaustive enumeration with capped or locally solved branches is not a global certificate. All same-frame force corrections preserve loads exactly. Independent IVP/BVP shape agreement and Stage 1 accuracy are included in summary.csv.

## Interpretation limits

Smaller actuation steps reduce deterministic state changes, but iid sensor noise retains its per-sample magnitude. Consequently noisy changes between consecutive observations do not shrink in proportion to the actuation step, and their effect on estimated slip may grow relative to actual motion. This experiment retains that noise model deliberately. One trajectory realization per noise level cannot establish a statistical ranking. Joint contact/force correction and temporal smoothing were not added.

At common actuation points, regenerated insertion truth forces match the coarse reference exactly. Rotation truth differs by 0.205 N vector RMS (0.373 N maximum), due to the finer friction history. Error metrics always use the corresponding physical reference. For iid channel noise, the covariance of successive noise differences remains twice the per-frame covariance; reducing actuation increments does not reduce this component.

A diagnostic example at 0.5 mm noise: fine rotation frame91 (22.5 degrees on return) selects the correct sticking mode but has 12.675 N tip error, versus 1.636 N at the shared coarse sample with the same noise. Its normalized FBG RMS is already 5.623 in the affine solve, 5.669 before BVP, and 5.476 after BVP. Thus this outlier is not explained solely by BVP nonlinearity or wrong mode classification; retained history constraints and the existing capped/local mode solves also matter.

## Failed corrections retained in the results

| Motion | Noise mm | Frame | Shape RMS mm | Terminal moment N mm | Failure |
|---|---|---|---|---|---|
| insertion | 2 | 173 | 65.85 | 641.1 | Unable to meet the tolerance without using more than 4000 mesh points.   The last mesh of 3324 points and the solution are available in the output argument.   The maximum residual is 2.03509, while requested accuracy is 1e-07. |
| rotation | 2 | 72 | 43.78 | 686 | Unable to meet the tolerance without using more than 4000 mesh points.   The last mesh of 3324 points and the solution are available in the output argument.   The maximum residual is 3.46508, while requested accuracy is 1e-07. |
| rotation | 2 | 78 | 2.009 | 231 | MATLAB:bvp4c:SingJac: Unable to solve the collocation equations -- a singular Jacobian encountered. |

## Findings

Noiseless rotation improves strongly with smaller steps; insertion contact/shape errors also improve. Noiseless insertion tip RMSE on all fine positions rises slightly because a new intermediate position has an isolated error spike, while matched-position tip RMSE improves. At 0.5/1 mm noise, matched-position insertion force errors remain almost unchanged; shape improves. Noisy rotation is mixed, and 2 mm runs exhibit serious failures. A smaller actuation increment is therefore useful for deterministic linearization accuracy but is not a sufficient noise-robustness measure for this recurrent estimator.

At 2 mm noise, insertion frame173 (68.8 mm wall distance on return) and rotation frame72 (36.75 degrees on return) reach the BVP mesh limit. Rotation frame78 (32.25 degrees) throws a singular-collocation-Jacobian exception and retains the pre-BVP estimate. Warning-returned states are passed onward by the unchanged wrapper, so error propagation is possible. These failures are included rather than removed. No statistical generalization is claimed from a single realization.

## Videos and plots

All eight cases: [media index](../../results/published/fine_bvp_comparison_20261012_step5/media/README.md). Each case includes an MP4 with XYZ force components and correction status, force plots, errors/timing/physics plots, Stage 1 plots, representative frames and a compact rendering source. Every saved frame is included; failures remain visible.

## Reproduction and outputs

Run setup_tsfs; addpath('experiments/inclined_wall'); run_fine_bvp_experiment('20261012_step5'). Then python experiments/inclined_wall/analyze_fine_bvp.py 20261012_step5. The runner skips completed outputs and rejects partial inverse trajectories. To rerun under a fresh stamp, copy noise_payload.mat/noise_protocol.json from the published comparison reproduction folder to the corresponding fresh input folder; numerical reference and inverse outputs must remain separate.

Combined output: results/published/fine_bvp_comparison_20261012_step5/. Raw trajectories: results/runs/fine_bvp_*_20261012_step5/comparison.mat. Truth folders: results/runs/fine_bvp_{insertion,rotation}_truth_20261012_step5/. Inputs: results/runs/fine_bvp_inputs_20261012_step5/. Main log: fine_bvp_20261012_step5.log.
