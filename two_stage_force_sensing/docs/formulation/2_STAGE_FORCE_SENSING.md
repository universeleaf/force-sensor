# 2 stage force sensing

Proposed formulation agreed on 2026-10-05. This note preserves the design before implementation. Each physical time step performs one geometric estimation, one mechanical estimation, and reporting-only diagnostics. There is no intermediate correction, repeated stage, rejection, or diagnostic feedback within a step.

The proposed MATLAB solvers, initial tolerances, diagnostics contract, and comparison protocol are specified in the [implementation plan](2_STAGE_FORCE_SENSING_IMPLEMENTATION_PLAN.md).

## Geometric estimation

Let a contain the curvature and twist coefficients reconstructed from FBG, with estimate a_bar and covariance Sigma_a. Base pose and required twist information must be calibrated or included in the uncertainty. Shape reconstruction uses only kinematics:

\[
p'=R e_3,\qquad R'=R[u(s;a)]_\times,\qquad t=Re_3.
\]

Plane j has unit inward normal n_j and offset d_j, defining admissible gap g_j(p)=n_j^T p-d_j >= 0. Use two local angular coordinates eta_j and one offset coordinate, theta_j=(eta_j,d_j), with environmental mean theta_bar and covariance Sigma_env. Contacts on a common plane share its parameters. The current repository uses a zero-radius centerline; for finite radius the plane offset must be adjusted consistently.

For candidate smooth interior contacts, solve

\[
\begin{aligned}
\min_{a,s,\theta}\;&\tfrac12\|a-\bar a\|_{\Sigma_a^{-1}}^2
+\tfrac12\|\theta-\bar\theta\|_{\Sigma_{env}^{-1}}^2
+\tfrac12\|s-s^-\|_{(P_s^-)^{-1}}^2\\
\text{s.t. }&n_{j(i)}^T p(s_i;a)-d_{j(i)}=0,\\
&n_{j(i)}^T t(s_i;a)=0,\quad 0\le s_i\le L.
\end{aligned}
\]

The arc-length tracking prior is optional. Candidates should be local distance minima with neighboring nonpenetration checks; tangency alone admits inappropriate extrema. Endpoints and corners require different contact geometry. A candidate is a hypothesis, and Stage 2 may return zero force for it. Near tangency, millimeter-scale surface uncertainty does not imply millimeter-scale arc-length uncertainty.

For a fast local approximation, stack closure and tangency residuals r_G evaluated on the reconstructed shape, and linearize r_G(a) = r_G(a_bar)+J_a delta_a. Eliminating delta_a from the constrained quadratic correction yields the profiled cost

\[
J_G=\tfrac12 r_G^T(J_a\Sigma_a J_a^T)^{-1}r_G
+\tfrac12\|\theta-\bar\theta\|_{\Sigma_{env}^{-1}}^2
+\tfrac12\|s-s^-\|_{(P_s^-)^{-1}}^2.
\]

Keep correlations between contacts induced by the same FBG data. Use the supported covariance subspace if singular, or justified model-error covariance. This is a local profile MAP approximation, not a marginalized Gaussian likelihood. Outputs are estimated arc lengths s_hat, normals n_hat, offsets d_hat, predicted geometric points p_G, and uncertainty diagnostics. Normals and arc lengths alone are insufficient to define a gap; offsets must also be carried forward.

## Mechanical estimation

Freeze s_hat, n_hat, and d_hat for this stage. Let B_i be an orthonormal 3 by 2 tangent basis with B_i^T n_hat_i=0. Contact force and position are

\[
f_i=f_{n,i}\hat n_i+B_i\tau_i,\qquad
p_{c,i}=p(\hat s_i;x),\qquad
g_i=\hat n_i^T p_{c,i}-\hat d_{j(i)}.
\]

The position p_c remains unknown and moves with the mechanical shape. It is not fixed to p_G or allowed to be a slack independent of the rod. It may be explicitly included with its equality constraint, or eliminated.

For an inextensible, unshearable quasistatic rod with point forces, no distributed loads, and zero tip moment, use the IVP

\[
\begin{aligned}
p'&=Re_3,& R'&=R[u]_\times,\\
u&=u_0+K^{-1}R^Tm,&
m'&=-p'\times\left(f_{tip}+\sum_{\hat s_i>s}f_i\right).
\end{aligned}
\]

The known base pose initializes p and R. The unknown base moment m_0 is an optimization variable and m(L)=0 is an outer equality constraint. Integration is segmented at contacts and intrinsic-curvature/stiffness discontinuities. A compact state is x=(m_0,f_tip,{f_n,i,tau_i}), containing 3C+6 unknowns.

For a stationary environment the tangential displacement is

\[
v_{i,k}=B_{i,k}^T[p_k(\hat s_{i,k})-p_{k-1}^+(\hat s_{i,k})].
\]

Both positions refer to the same material arc length and consecutive physical times, not optimizer iterations. For a moving surface, subtract its displacement. Missing predecessor information leaves the sliding direction unobserved and must be reported.

## Exact projection contact law

Choose positive numerical scales rho_n and rho_t in force per length. Define

\[
a_i=\max(0,f_{n,i}-\rho_n g_i),\qquad r_{N,i}=f_{n,i}-a_i=0.
\]

This is equivalent to g_i >= 0, f_n,i >= 0, and g_i f_n,i=0. For D(R)={z in R^2: ||z|| <= R}, impose

\[
r_{T,i}=\tau_i-\Pi_{D(\mu_i a_i)}(\tau_i-\rho_t v_i)=0.
\]

The exact disk projection is

\[
\Pi_{D(R)}(q)=\begin{cases}
0,&R=0,\\
q,&R>0,\ \|q\|\le R,\\
Rq/\|q\|,&R>0,\ \|q\|>R.
\end{cases}
\]

At a solution a_i=f_n,i. Separation gives zero force; sticking admits any force in the circular disk; sliding gives tau_i=-mu_i f_n,i v_i/||v_i|| and nonpositive frictional work. The zero-radius branch avoids division by zero and eliminates redundant polygon coefficients, but physical switching remains nonsmooth. Cone membership alone is not the sliding law. This normal/ball projection construction follows the standard contact formulation described in [GetFEM](https://getfem.readthedocs.io/en/latest/userdoc/model_contact_friction.html).

## Constrained MAP problem

With FBG observation y, prediction h_FBG(x), and predicted physical force state q^- with covariance P^-, solve

\[
\min_x\quad \tfrac12\|y-h_{FBG}(x)\|_{\Sigma_{FBG}^{-1}}^2
+\tfrac12\|q(x)-q^-\|_{(P^-)^{-1}}^2
\]

subject to the rod IVP, terminal moment, contact-position equalities, geometric gaps, normal and friction projection equalities, and sampled whole-rod nonpenetration. Define the force prior in world coordinates or transform its covariance when tangent bases change. No separate Stage 2 tangency equality is added in this proposed split. Continuous nonpenetration at a closed smooth interior contact would imply tangency; sampled constraints do not guarantee it.

Retain the MAP/IVP structure and use a semismooth SQP or related generalized-derivative method for the exact projections. A force projection after an unconstrained mechanical solve can break equilibrium. No smoothness or speed guarantee is claimed.

## Reporting after the two stages

Report geometric/mechanical point difference, active-contact tangency residual, gap and normal complementarity, friction-cone violation, projection-law residual, signed frictional work and maximum-dissipation residual, dense sampled penetration, terminal moment, weighted FBG residual, and solver termination status. Diagnostics never stop, change, or repeat a step. A tangential point difference is not automatically a physical violation; a force-free candidate need not remain tangent.

Both stages use the same FBG data. Do not add Stage 1 shape as an independent pseudo-measurement alongside raw FBG in Stage 2. This staged conditional estimate is an approximation to the joint posterior. Covariance computed with frozen geometry omits geometry-induced force uncertainty unless propagated separately.
