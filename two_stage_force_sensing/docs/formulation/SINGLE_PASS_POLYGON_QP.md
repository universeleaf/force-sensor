# Single rod linearization with circular or octagonal friction

This experiment is isolated in the mentor-side package. The user approved (2026-10-12): noise levels 0/0.5/1/2 mm; insertion/rotation; exhaustive modes; a single rod linearization, full circular friction with 100 SQP iterations; an inscribed eight-sided cone with quadprog; and removal of the single-pass trust box. The existing outer-loop reference retains its settings. No commit or push is authorized.

## Shared rod approximation

Stage 1 and initialization are unchanged. At each frame, one augmented nonlinear IVP supplies residual, terminal moment, contact/collision positions and their derivatives about z0. Stage 2 uses the affine approximations r(z)=r0+J(z-z0), p(z)=p0+P(z-z0), and terminal moment. Geometry and previous-frame history stay fixed. The cost is 1e-4/2 times the squared whitened FBG residual, without force priors. There is no trust-box restriction for the approved final single-pass comparison.

Full-cone branch solves retain the existing separation/sticking/sliding parameterization and fmincon SQP, but the inner iteration limit is 100. No outer rod refinement follows. Exhaustive selection uses the lowest affine-feasible FBG objective; if none is affine-feasible, the least affine violation is returned and flagged. This avoids silently refining/reselecting a one-pass result using nonlinear physics.

A final nonlinear IVP evaluates the selected force state for diagnostic shape, FBG mismatch, gap, penetration, equilibrium and friction consistency. It never corrects the estimated forces. Thus two augmented IVPs are requested per solved frame (linearization and validation), independent of branch count. This validation cost is included in timing. As in the current framework, the returned nonlinear state is the next-frame history; the next frame linearizes afresh. Native inner convergence and true nonlinear feasibility are reported separately. If validation throws, the affine shape is explicitly retained and flagged; it must not be interpreted as a nonlinear solution.

## Octagonal maximum-dissipation friction

In each fixed 2D tangent basis, let u_i=[cos(i*pi/4),sin(i*pi/4)]', h_i=[cos(i*pi/4+pi/8),sin(i*pi/4+pi/8)]', and c=cos(pi/8). The inscribed polygon is

    P(lambda) = {tau: h_i' tau <= mu lambda c for all i}, lambda >= 0.

For given slip v, friction minimizes v' tau over P(lambda). With rod positions affine and normals fixed, enumerate these convex branches:

- Separation: lambda=tau=0, nonpenetration.
- Sticking: contact gap=0, v=0, tau in P(lambda).
- Edge i sliding: gap=0, tau in P(lambda), h_i' tau=mu lambda c, v=-a h_i, a>=0.
- Vertex i sliding: gap=0, tau=mu lambda u_i, v=-a h_(i-1)-b h_i, a,b>=0.

This is 18 branches per contact (1 separation + 1 sticking + 8 edges + 8 vertices), not just eight sliding directions. Both edge and vertex cases are necessary for the full polygon normal-cone graph. Zero-slip/zero-force boundaries may overlap branches; that is allowed. Multiple contacts use the Cartesian product of assignments, with corresponding combinatorial cost.

Each branch is a convex QP:

    min_y 1/2 y' H y + f' y,  Aeq y=beq, A y<=b, lb<=y<=ub,

where y contains scaled Cartesian rod-load variables and nonnegative slip-normal coefficients, H=1e-4 J'J (with zero columns for auxiliary variables). MATLAB quadprog uses interior-point-convex, maximum 100 iterations, constraint tolerance 1e-6, optimality tolerance 1e-8. No explicit regularization/prior is added. Exiting positively certifies the solved convex branch to numerical tolerances; exhaustion is a global certificate for this affine polygon model only if all potentially better branches are solved or certified infeasible. It cannot certify the nonlinear rod or circular-friction problem.

For a known active inequality set, the optimum can be obtained from the KKT linear system [H C'; C 0][y;lambda]=[-f;d], subject to rank and multiplier-sign conditions. The unknown active set is why the constrained problem is solved with quadprog rather than one unconditional analytic inverse.

The polygon underestimates the circular radial friction bound by at most 1-cos(pi/8)=7.612%. Its sliding force direction can differ from exact circular Coulomb friction by up to 22.5 degrees. Direction error can therefore matter more than radial error. Circular-law residuals are diagnosed separately; polygon feasibility must not be labeled exact circular friction feasibility.

## Controls and interpretation

All methods share the same seed33001 Gaussian draws. Standard deviations are 0, 9.26814e-5, 1.85363e-4, 3.70726e-4 mm^-1. Zero injected noise retains the existing finite covariance scale 1e-4 mm^-1. New 0.5/1 mm amplitudes scale the prior 2 mm calibration; measured geometric realization RMS is verified separately. Environmental noise and mu uncertainty remain absent. Rotation starts from that method's insertion frame 21. Scoring excludes the restored rotation seed.

The final single-pass runs use no step bound. Preliminary bounded artifacts are preserved under stamp 20261012 and the smoke stamp. The inherited radius 4 blocked the insertion reversal (tangential force changes roughly -7 to +6 N), so those results cannot isolate rod-linearization accuracy. The final stamp is 20261012_unbounded.

The current comparison is exhaustive full-cone successive rod linearization: maximum 40 outer iterations, 300 inner SQP iterations, constraint tolerance 1e-6, outer step tolerance 1e-4, and normalized FBG gate 1.24. Existing 2 mm exhaustive results are reused only after bit-identical measurement verification. Other reference noise levels are replayed with unchanged reference solver settings.

Reproduction: `run_single_pass_comparison('FRESH_STAMP')` after setup and adding experiments/inclined_wall. Analyze with `python experiments/inclined_wall/analyze_single_pass_comparison.py FRESH_STAMP`. Preserve old stamps. Result tables separate native/affine convergence, nonlinear feasibility, physical error magnitudes, mode correctness, IVPs, and QP versus SQP iteration counts.
