# Collaboration handoff

Current consolidated handoff: [method, equations, evidence and next steps](2026-10-13_mentor-to-collaborator_current-method-and-next-steps.md). Includes solver alternatives beyond fmincon SQP and proposed regularization work.

Use `YYYY-MM-DD_from-to_topic.md`. Identify the author, intended recipient, branch/commit, scope, changes and reasons, approved assumptions, reproduction steps, successful and failed checks, open issues, next actions and owner. Preserve historical reports and link follow-ups. Drafting a report is not authorization to send it externally.

Start with [folder migration](2026-10-06_mentor-to-collaborator_folder-migration.md). Governing collaboration instructions are in the repository root AGENTS.md.

Historical experiment: [noiseless sliding comparison and friction sensitivity](2026-10-06_mentor-to-collaborator_single-contact-friction.md).

Rotation follow-up: [base rotation and friction/twist comparison](2026-10-06_mentor-to-collaborator_base-rotation.md).

- [2026-10-07: Bishop rotation observation correction and remaining initialization sensitivity](2026-10-07_mentor-to-collaborator_bishop-rotation.md).

- [2026-10-07: Residual audit identifies premature stopping and stagnation](2026-10-07_mentor-to-collaborator_bishop-residual-audit.md).

- [2026-10-07: Direct 1e-6 relaxation, IPOPT versus fmincon, two rotation increments](2026-10-07_mentor-to-collaborator_relaxed-rotation.md).

- [2026-10-07: Objective, variable and constraint scaling audit](2026-10-07_mentor-to-collaborator_relaxed-scaling-audit.md).

- [2026-10-07: No force priors, revised FBG scale/tolerances, all four solver reruns](2026-10-07_mentor-to-collaborator_relaxed-v2.md).

- [2026-10-07: Rotation Y sensitivity versus prior planar sliding](2026-10-07_mentor-to-collaborator_rotation-direction-sensitivity.md).

- [2026-10-07: Nonlinear tip/contact load transfer and close-shape examples](2026-10-07_mentor-to-collaborator_load-transfer.md).

- [2026-10-08: Inclined-wall bidirectional experiment, video, and unchanged mu=0 crash](2026-10-08_mentor-to-collaborator_inclined-wall.md).

- [2026-10-08: Inclined-wall difficulty audit, controlled seeds, and observed mu=0 crash trial](2026-10-08_mentor-to-collaborator_inclined-difficulty.md).

- [2026-10-10: Explicit integer friction modes, full inclined-wall trajectories, and all-branch costs](2026-10-10_mentor-to-collaborator_integer-modes.md).

- [2026-10-10: Previous-mode-first conditional search with approved FBG threshold](2026-10-10_mentor-to-collaborator_conditional-modes.md).

- [2026-10-10: Loaded-state 0-45-0 degree rotation, sticking return, and XYZ force videos](2026-10-10_mentor-to-collaborator_inclined-rotation.md).

- [2026-10-11: Linearized rod with full circular friction, matched comparisons and rejected-mode slowdown](2026-10-11_mentor-to-collaborator_linearized-rod.md).

- [2026-10-11: Calibrated Gaussian FBG noise, convergence versus mode correctness, and full trajectory outputs](2026-10-11_mentor-to-collaborator_noisy-linearized.md).

- [2026-10-11 noisy exhaustive-mode audit](2026-10-11_mentor-to-collaborator_noisy-exhaustive.md): full noisy trajectories, force-error causes, cycle detection evidence and proposed budgets.

- [2026-10-13: Five-times-finer one-pass rod plus BVP](2026-10-13_mentor-to-collaborator_fine-step-bvp.md): all noise levels, matched samples, runtime and physical consistency.

- [2026-10-13: Videos, plots and branch publication](2026-10-13_mentor-to-collaborator_media-publication.md).

- [2026-10-25 (collaborator to mentor): exact QP branch solves, uncertain slip history and the existing force prior](2026-10-25_collaborator-to-mentor_qp-branches-history-prior.md): identical-input solver comparison, named-spike diagnoses, held-out multi-seed results, failures.
