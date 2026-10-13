# Fixed-load nonlinear BVP after exhaustive affine mode search

User-approved follow-up, 2026-10-12. This corrects rod equilibrium with prescribed loads; it does not optimize those loads or enforce wall contact again.

Let physical estimator coordinates be x=Dz=[m0; Ftip; lambda1; tau11; tau12; ...]. Stage 1 supplies fixed material contact arclengths sk and normal/tangent bases. Exhaustive single-pass circular SQP selects x once. All components of x after m0 stay exactly fixed during this correction.

On each material interval define the prescribed downstream resultant \(Q(s)=F_{tip}+\sum_{s_k>s}F_k\), with \(F_k=n_k\lambda_k+B_k\tau_k\). In the existing sign convention,

\[
\frac{dp}{ds}=Re_3,\qquad
\frac{dR}{ds}=R\widehat u,\qquad
u=u_{intrinsic}+\operatorname{diag}(K)^{-1}R^Tm,
\]
\[
\frac{dm}{ds}=Q\times(Re_3),\qquad
\frac{d\psi}{ds}=u_3.
\]

The Bishop channels are recomputed through the existing material-to-Bishop observation map using psi. Full 3D rotation and twist are retained.

Boundary conditions: p(0)=prescribed base position, R(0)=prescribed base rotation, psi(0)=0, m(L)=0. States are continuous across intrinsic-curvature, stiffness and point-force interfaces. With the centerline-force approximation used here, moment is continuous across a point force; the downstream resultant Q changes across it. No contact torque is introduced.

MATLAB bvp4c solves the multipoint collocation problem. Initial guess: the selected nonlinear IVP centerline, rotation, moment, and Bishop roll. The selected optimized m0 initializes the proximal moment. Position is scaled by rod length L, moment by the existing moment scale M, and arclength by L. RelTol=1e-7, AbsTol=1e-9, NMax=4000, initial mesh spacing <=5 mm with duplicated interface points. An analytic ODE Jacobian is supplied. Endpoint evaluations of the dense IVP seed are clamped within their own interval to remove floating-point roundoff only.

After the one BVP solver call, z(1:3) receives its corrected base moment. Forces and selected mode remain unchanged. An independent augmented nonlinear IVP reconstructs the state and checks agreement with the BVP shape. It supplies the standard observation and diagnostic derivatives. This third IVP is validation/representation, not another force correction.

The returned corrected state becomes the next frame's history, as requested. Consequently complete recurrent trajectories can have different estimated forces than the uncorrected benchmark, despite exact force invariance within every correction.

Important scope: the BVP enforces terminal equilibrium but does not impose the wall gap, no penetration, tangency, sticking/sliding or measurement MAP objective. Those are diagnosed on its returned shape. A converged fixed-load BVP can fail contact-mode consistency. BVP warnings/exceptions are recorded; exceptions retain the uncorrected estimate. A BVP call may contain multiple internal Newton and mesh iterations, so its cost is reported separately from the three IVPs and SQP iterations.

Implementation: matlab/+tsfs/fixed_load_bvp.m and single_pass_bvp_solve.m. Experiment: experiments/inclined_wall/run_single_pass_bvp.m. Source single_linearization_solve.m and all previous experiment outputs are unchanged.
