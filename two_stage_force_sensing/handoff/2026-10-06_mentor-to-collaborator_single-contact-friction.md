# Single-contact friction comparison handoff

Date: 2026-10-06. Author: Codex assisting the mentor. Direction: mentor to collaborator; prepared locally, not sent externally.

Branch: `codex/two-stage-force-sensing`. Parent commit: `9e8c3edafd0513097256cd709882de8e2f41253b`. This report ships with the experiment commit; identify it with `git log -1 -- two_stage_force_sensing/handoff/2026-10-06_mentor-to-collaborator_single-contact-friction.md`.

## Scope and approved decisions

The user requested the current estimator and current settings on noiseless single-contact sliding, comparing Tongyu's Aloi implementation, ours with mu=0.3, and ours with mu=0, including sensitivity, accuracy, timing and solver work. The same frictional truth is retained for the zero-friction inference ablation. No physics, objective, covariance, reconstruction, solver tolerance or prior was changed. No new planar restriction was imposed on the general estimator; the existing synthetic test is planar and assumes intrinsic twist for the unmeasured channel.

Added isolated MATLAB runners, saved-solution exports, local/finite-perturbation sensitivity analysis, a Python report generator, compact evidence and plots. `setup_tsfs` only adds the experiment folder to the path. Original rod/LCP algorithms are called directly from the collaborator checkout. Our core and all collaborator source files remain unchanged.

## Measured result

Twelve clean frames; true and correct estimator friction coefficient 0.3. Zero FBG and environment measurement error is verified explicitly, while existing nonzero likelihood/prior covariance scales are retained.

- Ours mu=0.3: contact/tip RMSE 0.00339/0.00175 N, shape RMS 0.00254 mm; all frames converge without flags.
- Ours mu=0: contact/tip RMSE 2.21148/2.08428 N, shape RMS 0.00714 mm; all frames converge but all flag FBG mismatch. Projection, complementarity, friction-work and geometric checks pass their current tolerances.
- Tongyu Aloi: total resultant RMSE 1.21819 N, shape RMS 0.05233 mm. It does not expose contact/tip separation. All final equilibrium checks pass; Gaussian width hits its lower bound in all frames.
- Mean full-step times: 0.152/0.164 s for ours at mu=0.3/0; mean SQP iterations 2.92/4; full rod IVP evaluations 4.83/7. Aloi takes 84.481 s for the full sequence (7.040 s/frame); a separate profiling replay counts 32,105 shape integrations. Counts represent different operations and timings use different warmup protocols, explicitly documented.

Fixed Stage 1 geometry is identical across mu. The local six-force measurement map has condition number 98-112; imposing the correct contact-law tangent gives a three-direction map with condition number 15.3-15.7. This is finite weak coupling rather than exact singularity. Removing friction and compensating with tip and normal forces predicts the nonlinear tip change within 0.00252 N over all frames. Good shape agreement alone cannot validate force separation.

## Reproduce and inspect

See [full report](../docs/reports/SINGLE_CONTACT_MU_COMPARISON.md) for commands, units, limitations and plots. Configure baseline dependencies as in the package README, set `TSFS_RUN_ID=single_contact_mu_20261006`, then run `run_single_contact_comparison('ours')`, `run_single_contact_comparison('aloi')`, `analyze_single_contact_mu`, `run_single_contact_comparison('profile')`, and `export_single_contact_evidence`. Run the Python summary script afterwards. All commands are after `setup_tsfs` from the package directory.

Large raw MAT files/logs stay locally under ignored results/runs. Selected CSV/JSON/plots are committed under results/published/single_contact_mu_20261006. Both our numerical repetitions agree exactly in the checked force/shape/counter outputs. Report generation asserts matching input conditions and expected statuses; the figures were visually inspected. No new core tests are needed for this experiment-only addition; existing core test results from migration remain separate evidence.

## Limitations and next owners

This does not test noisy data, estimate unknown friction, validate real FBGS calibration, or resolve S-channel failure. Later-frame finite mu perturbations vary with recurrent tracking and existing solver tolerances; they are reported rather than treated as an exact infinitesimal derivative. A clean single-contact result does not demonstrate general multi-contact observability.

Mentor: review whether to prioritize noise robustness, friction uncertainty or measurement-objective analysis. Collaborator: review force/shape metrics and repeat the provided scripts before adopting changes. Codex: preserve this baseline and seek approval before any new scientific assumption or core modification. Previous deferred topics (twist friction torque, tip contact, edge/corner geometry, and S-channel fixed-tip-Fy analysis) remain deferred.
