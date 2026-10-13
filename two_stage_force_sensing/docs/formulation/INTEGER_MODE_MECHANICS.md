# Exact integer contact modes with nonlinear rod mechanics

Date: 2026-10-10. This is a separate experimental solver, not a replacement for the existing relaxed or projection solvers. The user requested explicit integer contact modes, unchanged nonlinear Cosserat mechanics, and initialization from the preceding mode.

## Mathematical problem

Stage 1 is unchanged. It fixes each candidate material arclength s_i, plane normal n_i, orthonormal tangent basis B_i (3 by 2), and estimated plane. Stage 2 still optimizes the initial moment, tip force, and contact loads. Contact position p(s_i) is determined by the nonlinear rod IVP and remains unknown during optimization. The material-frame mechanics, constitutive relation, force jumps, Bishop observation map, and full 3D force components are unchanged.

Let lambda_i >= 0 be normal load, tau_i in R^2 be tangential load, g_i be signed gap, and v_i = B_i' [p_k(s_i) - p_{k-1}(s_i)] be tangential displacement of the same material arclength. The preceding estimated mechanical shape provides history. Contact force is n_i lambda_i + B_i tau_i. The quasistatic discrete Coulomb law uses displacement direction; no dynamic velocity model is added.

Use three binary selectors b_sep, b_stick, b_slide with sum one. Their implications are:

| Mode | Exact constraints |
|---|---|
| Separation | lambda = 0, tau = 0, g >= 0; v unrestricted |
| Sticking | g = 0, v = 0, lambda >= 0, norm(tau) <= mu lambda |
| Sliding | g = 0, lambda >= 0, a >= 0, v = a d(theta), tau = -mu lambda d(theta) |

Here d(theta) = [cos(theta), sin(theta)]' spans the entire circular tangent disk boundary. Theta does not restrict the rod to a plane. At a = 0 the sliding branch overlaps the admissible sticking boundary, as required by the set-valued zero-slip law. Modes can also overlap at zero force; a unique mode label is not physically required there.

For mu = 0, eliminate tau identically and enumerate separation or active normal contact (g = 0, lambda >= 0, v unrestricted). This exact specialization avoids artificial distinctions between frictionless sticking/sliding and the degenerate zero-radius cone.

All branches retain the terminal moment equilibrium and sampled rod nonpenetration. Where an active contact gap equality duplicates a sampled gap inequality at precisely the same arclength, keep the equality only. All other sampled inequalities remain. Contact tangency is still a diagnostic, not an additional Stage-2 constraint.

The objective is the unchanged sensor-only Bishop MAP data term:

    1e-4 * 0.5 * norm(C_U^(-1/2) [w_pred(x) - w_obs])^2.

C_U = (1e-4 /mm)^2 I in this experiment. There are 48 residual entries (24 nodes, two Bishop components). No tip/contact force-prior penalties, complementarity relaxation, polygonal friction approximation, or squared cone/complementarity product constraints are used by this solver.

## Numerical realization

`tsfs.mode_solve` enumerates every admissible integer assignment, trying the matched preceding selected assignment first. For each assignment, `tsfs.mode_branch` solves the remaining nonlinear problem using MATLAB fmincon SQP and analytic derivatives propagated through the existing IVP. For a single frictional contact this is three continuous solves; for mu = 0 it is two. With no candidate it is one.

All branch seeds use the existing initializer, which reuses a preceding converged material solution when contact matching succeeds. Each mode projects the seed into its own coordinates. Sliding direction starts from the Stage-1 current contact position minus the preceding estimated position at the current arclength; no reference loads, contact locations, or reference shape enter initialization. Previous-mode-first ordering does not skip competing modes or give a global continuous certificate.

The full Cartesian product is supported for several candidates, but scales exponentially (3^C for C frictional candidates). This test only validates a single candidate and a planar physical fixture with a full 3D estimator.

Choose the lowest sensor objective among returned mode-feasible solutions. A feasible point whose native solver exit is nonpositive can be selected, but remains labeled nonconverged. If none is feasible, return the least-violating attempt and report the failure. Exceptions from a branch's fmincon callbacks return its last accepted iterate with a failure status; other integer assignments are still attempted. Diagnostics flag issues without repairing, stopping, or changing the state. As in the previous benchmark, a returned state supplies the next frame's history even if flagged.

This is exhaustive integer-assignment enumeration with LOCAL nonlinear solves. It solves the discrete disjunction explicitly, but it is not branch-and-bound with certified lower bounds and does not certify a global MINLP optimum. No big-M bounds are needed. Some losing branches may fail; all attempted computation is counted. No claim is made that a failed continuous branch is globally infeasible.

## Numerical settings

| Setting | Value |
|---|---:|
| fmincon algorithm | SQP |
| Iterations / objective evaluations | 300 / 5000 per assignment |
| Constraint tolerance | 1e-6 |
| Optimality tolerance | 1e-8 |
| Step tolerance | 1e-12 |
| Objective multiplier | 1e-4 |
| Base-moment / force scales | 200 N mm / 1 N |
| Gap / tangential displacement units | mm |
| Theta / sliding amplitude units | radians / mm |
| ODE relative / absolute tolerances | 2e-8 / 2e-10 |
| RHS safety budget | 100000 per IVP |
| Nonpenetration mesh / diagnostic mesh | 0.5 mm / 0.1 mm |
| Diagnostic gap tolerance | 1e-6 mm |
| Diagnostic terminal moment tolerance | 2e-4 N mm (matches normalized constraint) |
| Projection / cone diagnostic tolerances | 1e-6 N / 1e-6 N |
| Normal product diagnostic tolerance | 1e-5 N mm |
| Tangency / FBG warning thresholds | 1e-3 / 4 normalized RMS |

Time and iteration/IVP totals include every attempted integer assignment, not just the winner. Per-frame estimation time includes geometry, mechanics, and diagnosis, excluding artifact serialization. IVP counts include failed attempts; RHS counts include completed IVPs only because an interrupted callback does not return a partial RHS count. Tests and smoke validation are excluded from the full-trajectory performance totals.

## Reproduction

Run from the package, with existing baseline dependencies configured:

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
test_mode_formulation;
run_inclined_modes(Inf, 'inclined_wall_modes_NEW_RUN');
```

The runner reuses the preserved 2026-10-08 truth packet and does not rerun or modify the collaborator's forward algorithms. Default output tag is `inclined_wall_modes_20261010`; supply a fresh tag to preserve that benchmark. The published CSVs contain every returned frame and every branch attempt; the raw MAT file includes iteration traces, geometry, and saved diagnostics. `analyze_inclined_modes.py` analyzes the dated benchmark and verifies accounting, frame completeness, identical geometry, and previous-mode-first ordering.

## Conditional-search follow-up (2026-10-10)

The user subsequently approved trying the preceding mode first and stopping at the first sufficiently accurate converged branch. This is optional (`cfg.modeSearch='first-acceptable'`); omitting the option retains exhaustive search and its minimum-objective selection. The original exhaustive results remain preserved.

The user explicitly selected **raw Bishop FBG RMS <= 1e-6 /mm**. With the benchmark's covariance C_U = 1e-8 I, this is **whitened RMS <= 0.01**, stored as `cfg.fbgRmsAcceptance`. This is different from the nonlinear constraint tolerance 1e-6 and the existing diagnostic FBG warning threshold 4. For 48 residual components the acceptance test is equivalent to unscaled sensor objective <= 0.0024, or scaled objective <= 2.4e-7.

After each complete local fmincon solve, accept the branch if all three hold:

1. Native fmincon exit is positive (optimality or small-step termination).
2. Maximum implemented branch-constraint/bound violation is <= 1e-6.
3. RMS of whitened FBG residuals is <= 0.01.

Stop searching immediately at the first accepted mode. If it fails any test, try the next assignment in the original ordering. Individual fmincon stopping tolerances, mechanics, geometry, IVP, and initialization are unchanged. This policy does not interrupt a continuous solve before its normal termination.

If every mode fails the acceptance rule, retain the original fallback (lowest-objective feasible return, otherwise least violation), set `modeAccepted=false`, and add a reporting-only `no-acceptable-contact-mode` flag. Continue the trajectory with the existing history policy. A converged fallback is not reported as satisfying the requested FBG acceptance threshold.

The approved runner is `run_inclined_modes_conditional(Inf,'inclined_wall_modes_conditional_1e6_20261010',0.01)`. It benchmarks both mu=0.3 and mu=0. The preliminary broader-threshold run, started before the user's threshold reply, is preserved separately under `inclined_wall_modes_conditional_20261010` and is not the approved-threshold result.

This conditional policy sacrifices exhaustive mode comparison; it has no global objective certificate. Its 1e-6 /mm residual gate is specific to this noiseless test. It would need reconsideration for experiments with materially larger measurement noise.
