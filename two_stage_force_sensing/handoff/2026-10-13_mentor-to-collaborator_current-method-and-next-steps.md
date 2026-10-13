# Research handoff: two-stage force sensing with a linearized rod and nonlinear correction

Date: 2026-10-13. Author: mentor-side research collaboration, prepared with Codex. Recipient: Tongyu / incoming project collaborator.

Implementation reference: branch `codex/two-stage-force-sensing`, commit `a01a2b3c7480a48dad90479ba40f8542ed369508` ([GitHub](https://github.com/universeleaf/force-sensor/tree/codex/two-stage-force-sensing/two_stage_force_sensing)). This handoff was prepared after that implementation commit. The user subsequently authorized committing and pushing this document and its navigation links to the same branch. No implementation changes, simulations or external messages were made for this handoff.

## 1. Executive orientation and collaboration

The current framework is:

**Stage 1 geometric estimation -> Stage 2 one rod linearization and exhaustive full-cone contact-mode solves -> one fixed-load nonlinear BVP correction -> reporting-only diagnostics -> next physical frame.**

There is no internal rod relinearization loop or repeated Stage 1/Stage 2 correction within a frame. The BVP has internal numerical iterations: one correction means one BVP call, not one Newton iteration. This is a recurrent optimization estimator, not yet a Kalman filter with propagated posterior covariance.

The immediate task is to improve stability and speed inside this framework. Priorities are noisy force spikes, jerky reconstructed trajectories (particularly rotation), and expensive rejected contact branches. Investigate implementation issues, objective regularization, and solvers other than MATLAB fmincon SQP. The present results are promising without noise but are not a robust experimental sensing solution. BVP convergence alone does not imply correct contact physics, accurate forces, or acceptable FBG fit.

All mentor-side work belongs in `two_stage_force_sensing/`. Existing collaborator algorithms are called through interfaces and remain in their original folders. Root [AGENTS.md](../../AGENTS.md) governs isolation and approval. Preserve old results and use fresh experiment stamps. Each commit and push requires explicit authorization. The role is to provide meaningful diagnosis, evidence and guidance, with modular code the collaborator can take over. We may depart from the original formulation where agreed; important unplanned modeling/solver decisions require mentor review before implementation.

Current assumptions: quasistatic, inextensible, unshearable 3D Cosserat rod; known intrinsic strain and bending/torsional stiffness; known base pose; centerline point forces at smooth interior planar contacts; unknown applied tip force and zero tip moment. No distributed load term, direct contact friction torque, environmental tip contact, edge/corner contact law, or finite-radius contact mechanics is included. Applied tip load is distinct from environmental tip contact.

## 2. Design rationale

| Choice | Purpose | Limitation |
|---|---|---|
| Separate geometry and forces | Reduce joint coupling of arc length, normal, shape and loads | Frozen geometry is an estimate, not exact knowledge |
| Contact tangency in Stage 1 | Constrain smooth-contact location/orientation from shape | Small wall errors can still cause appreciable arc-length errors |
| Bishop observations | Consistent geometrical bending with full material twist in mechanics | Experimental sensor frame/gauge still needs calibration |
| Full circular friction cone | Preserve arbitrary 3D tangent-plane force direction | Sliding remains nonlinear and generally nonconvex |
| One rod linearization | Avoid repeated IVPs inside branch optimization | Noise or large changes can invalidate the local model |
| Exhaust sparse modes | Remove complementarity products/disjunction handling from continuous solves | Does not remove all nonconvexity or certify global optimality |
| Fixed-load nonlinear BVP | Restore nonlinear equilibrium after affine estimation | Can worsen gap, slip, tangency and measurement fit |

Fixing normal orientation reduces ambiguity between normal and tangential force decompositions. It does not remove tip/contact load compensation or other weak force directions.

## 3. Observation model

Units: mm, N, N mm, radians, curvature mm^-1. Material arc length s is in [0,L]. Positions and physical forces use the world frame. R(s) is the material orientation; K is the diagonal bending/torsional stiffness matrix in N mm^2. The cross-product matrix satisfies [u]_x v = u cross v.

The simulated sensor supplies two Bishop channels w=(w1,w2) at 24 FBG stations, rather than discarding u3 from material strain. With Bishop frame B=[b1,b2,t] initialized at the known base orientation,

\[
p'=t,\quad t'=w_1b_1+w_2b_2,\quad
b_1'=-w_1t,\quad b_2'=-w_2t.
\]

Material-to-Bishop prediction is

\[
R=B R_z(\psi),\quad \psi'=u_3,\quad \psi(0)=0,
\]
\[
w_1=\cos\psi\,u_2+\sin\psi\,u_1,\qquad
w_2=\sin\psi\,u_2-\cos\psi\,u_1.
\]

Bishop angular strain is [-w2,w1,0], not material strain. Zero Bishop tangent-axis spin does not mean zero physical material twist. Full material twist is retained in IVP/BVP mechanics and sensitivities.

Current geometric reconstruction uses segmented PCHIP interpolation, respecting known intrinsic/stiffness breaks, and SE(3) midpoint integration on a 1 mm grid. It is not simply piecewise constant curvature over FBG spacing. Refining actuation increments does not refine spatial discretization.

For experiments, calibrate illusense/ShapeSensing curvature, curvature angle, reported shape and base-frame conventions. The simulated Bishop contract does not establish that experimental curvature angle is already a Bishop angle. Shape and curvature derived from the same wavelengths must not be counted as independent observations without a covariance model.

## 4. Stage 1: fast geometric estimation

Reconstruct p_G(s),t_G(s) from observations and base pose, without mechanics. Candidate contacts are interior local minima of plane distance inside the candidate-distance threshold. Estimate arc lengths and small plane corrections. For each plane, use two local normal coordinates eta and one normal-offset correction delta:

\[
n_j(\eta_j)=\frac{\bar n_j+\bar B_j\eta_j}
{\|\bar n_j+\bar B_j\eta_j\|},\qquad
 g_j(p)=n_j^T(p-p_{0,j})-\delta_j.
\]

Normals point into admissible free space, so g>=0. Contacts on a common plane share its parameters. Stack closure and tangency residuals:

\[
r_i^G=\begin{bmatrix}
n_j^T[p_G(s_i)-p_{0,j}]-\delta_j\\n_j^Tt_G(s_i)
\end{bmatrix}.
\]

With J_Gw the residual sensitivity to Bishop measurements, the implemented fast profile-MAP approximation is

\[
\min_{s,\theta}\ \frac12(r^G)^T S_G^{-1}r^G+
\frac12\|\theta-\bar\theta\|_{\Sigma_E^{-1}}^2,
\qquad S_G=J_{Gw}\Sigma_wJ_{Gw}^T+\Sigma_{G,model}.
\]

This retains cross-contact correlations from shared FBG data. It is a local profiled least-squares approximation, not an exact marginalized posterior; no log-determinant term is added. The current implementation has no arc-length tracking prior. MATLAB lsqnonlin uses trust-region-reflective least squares. Its supplied Jacobian is centrally differenced through the geometric residual and covariance, without Cosserat solves.

Stage 2 freezes s_i, n_j, delta_j and tangent bases B_i for that frame, but the physical contact point remains unknown through the rod:

\[
p_{c,i}=p(s_i;x),\qquad F_i=n_i\lambda_i+B_i\tau_i,
\quad B_i^Tn_i=0.
\]

Plane offsets must be passed as well as normals and arc lengths. A candidate may carry zero force; Stage 1 does not establish active contact. Mechanical tangency mismatch is diagnosed, not repaired. Both stages use the same FBG data and uncertainty, so they are not independent information sources. Geometry posterior uncertainty is not propagated as a full joint mechanics posterior.

## 5. Stage 2: rod mechanics and likelihood

Physical variables are

\[
x=Dz=[m_0;F_{tip};\lambda_1;\tau_1;\ldots;\lambda_C;\tau_C],
\quad \tau_i\in\mathbb R^2.
\]

There are 6+3C Cartesian variables before branch reduction. D scales m0 by L times a 1 N force scale (200 N mm here), and loads by 1 N. Define downstream resultant Q(s)=F_tip+sum_{s_i>s} F_i. Between force/material interfaces,

\[
p'=Re_3,\quad R'=R[u]_\times,\quad
u=u_0+K^{-1}R^Tm,\quad m'=Q\times(Re_3).
\]

Known base position/orientation and unknown m0 initialize the IVP; terminal equilibrium requires m(L)=0. Integration is segmented at contacts and intrinsic/stiffness changes. Across a centerline point force Q changes, while p,R,m remain continuous. No contact-moment jump is introduced.

The material-to-Bishop map predicts measurements at actual sensor stations:

\[
r_w(z)=\Sigma_w^{-1/2}[w_{pred}(z)-w_{obs}],\qquad
J_{data}(z)=\frac{\alpha}{2}\|r_w(z)\|^2,\quad \alpha=10^{-4}.
\]

The current objective is sensor-only: no tip/contact force priors. Alpha is numerical objective scaling, separate from measurement covariance. Several stored objective/trace values are unmultiplied 0.5||r_w||^2; compare them to fmincon's callback using objectiveScale. Normalized FBG RMS is ||r_w||/sqrt(48).

## 6. Exact full-cone contact law and mode enumeration

For a stationary wall, tangential displacement compares the same material location across consecutive physical frames:

\[
v_{i,k}=B_{i,k}^T[p_k(s_{i,k})-p_{k-1}^{+}(s_{i,k})].
\]

The previous returned mechanical state supplies history. Do not compare two different arc lengths or advance history inside SQP. This is a quasistatic displacement-based law, not a calibrated velocity model.

Normal complementarity, the circular friction disk, and maximum dissipation are

\[
0\le\lambda_i\perp g_i\ge0,\qquad \|\tau_i\|_2\le\mu_i\lambda_i,
\qquad \tau_i\in\underset{\|q\|_2\le\mu_i\lambda_i}{\arg\min}\ q^Tv_i.
\]

For nonzero slip tau=-mu lambda v/||v||; at zero slip the entire disk is admissible. Equivalent projection laws are lambda=max(0,lambda-rho_n g) and tau=Pi_disk(tau-rho_t v), for positive dimensional scales. These express the physics; the current solver does not optimize projection equalities or Scholtes products.

Instead enumerate the following branches:

| Mode | Constraints |
|---|---|
| Separation | lambda=0, tau=0, g>=0; displacement unrestricted |
| Sticking | g=0, v=0, lambda>=0, ||tau||<=mu lambda |
| Sliding | g=0, lambda>=0, a>=0, v=a d(theta), tau=-mu lambda d(theta) |

Here d(theta)=[cos(theta),sin(theta)]^T covers the full circle. At a=0, sliding overlaps the sticking boundary; zero force also permits label overlap. The angle can remain unidentifiable at zero load/slip. Avoiding squared products does not remove every degeneracy.

Branches retain affine terminal equilibrium and sampled nonpenetration. Active gap equality replaces its identical sampled inequality. Stage 2 tangency is diagnostic. Other solver paths have a frictionless specialization; the current fine-step experiment uses mu=0.3 throughout.

For C frictional candidates enumerate 3^C assignments (three with one contact, one solve with none). Previous mode first is ordering only: all assignments are tested. Choose the lowest affine objective among returned feasible candidates; otherwise the least violating. A capped feasible return can be selected but remains nonconverged. The FBG threshold does not force abstention when every fit is poor.

This is enumeration with local continuous solves, not certified mixed-integer branch-and-bound. No big-M formulation or global certificate is used. Failure of a continuous solver does not prove a mode infeasible. Sparsity makes the current approach manageable; exponential scaling matters as candidate count grows.

## 7. One rod linearization; why fixed mode is not always a QP

Initialization uses a geometric inverse moment-balance seed or the previous converged material state when matching permits. Friction history can still come from a flagged returned state; history use and warm-start eligibility are separate policies.

One augmented nonlinear IVP at zbar provides outputs and analytic variational sensitivities:

\[
\hat w=w(\bar z)+J_w\Delta z,\quad
\hat p(s)=p(s;\bar z)+J_p(s)\Delta z,\quad
\hat m_L=m_L(\bar z)+J_m\Delta z,\quad \Delta z=z-\bar z.
\]

Gaps, sampled positions and tangential displacements are affine in z. All branches share this expansion and immutable previous-frame history. The data objective is quadratic in Cartesian z. No IVP is called inside the affine branch callbacks.

| Branch | Structure after rod linearization | Current / potential solver |
|---|---|---|
| Separation | Convex quadratic objective; linear constraints | fmincon currently; quadprog sufficient |
| Sticking | Convex quadratic objective; linear constraints and second-order cone | fmincon currently; convex quadratic/conic solver can exploit structure |
| Sliding | Exact direction/load/amplitude coupling; nonlinear mapping from (lambda,theta,a) to forces | Nonlinear optimizer for the current formulation |

Quadprog cannot directly impose the circular sticking cone. Sliding has products and sine/cosine; substitution of tau=-mu lambda d(theta) also makes the objective nonlinear in branch coordinates. Fixing theta or polygonizing friction creates a different approximation, requiring approval.

All branches currently use fmincon SQP for implementation uniformity, with at most 100 iterations, analytic callback derivatives and no finite trust box. Enumeration removes the complementarity disjunction from each subproblem but does not make sliding convex. No rod relinearization follows branch selection.

## 8. Nonlinear correction and diagnosis

After selection, a nonlinear IVP at z* supplies an actual mechanical BVP guess, including p,R,m,psi and optimized m0. The guess is not merely the affine plotted centerline. The correction keeps all estimated tip/contact forces and contact arc lengths fixed, solving the nonlinear rod with

\[
p(0)=p_b,\quad R(0)=R_b,\quad \psi(0)=0,\quad m(L)=0.
\]

MATLAB bvp4c uses multipoint collocation, an analytic ODE Jacobian, and continuous states across interfaces. Position, arc length and moment are scaled by L,L and the moment scale. Only z(1:3) receives the corrected base moment; force entries remain exactly unchanged. A final independent augmented IVP reconstructs the state and audits agreement with the BVP.

Typical accounting is three mechanics IVPs (linearization, selected-state/BVP seed, corrected-state audit), plus one BVP call. The selected force or mode is not reoptimized afterward. A BVP call can contain multiple internal Newton/mesh iterations.

The BVP imposes neither contact gap nor sticking/sliding, nonpenetration, tangency or FBG likelihood. It can restore terminal moment while worsening contact consistency. Although forces stay fixed within a correction, complete recurrent reruns can estimate different forces because corrected shapes enter later history.

Diagnostics include tangency mismatch, sampled penetration, active gap, normal complementarity, friction cone/projection, positive work, terminal moment, FBG fit and solver status. They do not stop or repair the trajectory.

Important existing behavior: a BVP mesh-limit warning returns an approximate state, marks convergence false, but still passes that audited state into history. A thrown BVP exception retains the pre-correction estimate. Audit these separate paths before stabilization; both may propagate errors. Changing fallback/history policy requires approval and a separate comparison.

## 9. Current settings

These are fine-step experiment settings, including overrides. Shared defaults and older successive-linearization configurations differ; the authoritative runner is run_fine_bvp_experiment.

| Setting | Value |
|---|---|
| Stage 1 solver | lsqnonlin trust-region-reflective |
| Stage 1 iterations / evaluations | 30 /300 |
| Stage 1 function / optimality / step tolerance | 1e-8 /1e-6 /1e-8 |
| Geometric model floors: position / tangency | 1e-3 mm /1e-5 |
| Candidate distance / endpoint exclusion | 2 mm /1 mm |
| Stage 2 solver | fmincon SQP |
| Branch iterations / evaluations | 100 /5000 |
| Constraint / optimality / step tolerance | 1e-6 /1e-8 /1e-12 |
| Objective multiplier | 1e-4 |
| Outer rod relinearizations / finite trust bound | none /none |
| IVP relative / scaled absolute tolerance | 2e-8 /2e-10 |
| BVP relative / absolute tolerance in scaled states | 1e-7 /1e-9 |
| BVP maximum mesh points / initial spacing | 4000 /at most 5 mm |
| Collision / diagnostic spacing | 0.5 mm /0.1 mm |
| Gap / terminal moment diagnostic tolerance | 1e-6 mm /2e-4 N mm |
| Cone / projection tolerance | 1e-6 N /1e-6 N |
| Normal force-gap product tolerance | 1e-5 N mm |
| Tangency warning / positive-work threshold | 1e-3 /1e-8 N mm |
| Normalized FBG acceptance / legacy warning threshold | 1.24 /4 |

Native positive exit, affine feasibility, post-BVP mode feasibility, FBG acceptance and diagnostic flags are distinct. solverSuccess means convergence plus nonlinear selected-mode feasibility, not universally clean diagnostics or accurate forces. Constraints mix physical units and scaled rows: retain/report those scales for solver comparisons. Sampled collision checks do not certify continuous nonpenetration.

## 10. Research scenarios, noise and S-channel decision

### Contact location and tip/contact coupling

Current inclined-wall rod: 200 mm total, 80 mm straight plus 120 mm precurved through 90 degrees (intrinsic curvature about 0.01309 mm^-1 in the curved section). Wall normal [1,0,-1]/sqrt(2); true tip load [1,0,0] N; mu=0.3. The base translates along world Z; perpendicular wall distance follows 80->40->80 mm. Contact migrates along the rod, supplying a continuous contact-location sweep. Location is correlated with configuration and load in this test; it is not an isolated causal intervention on arc length.

Fine insertion: 0.4 mm distance increments, 201 states; coarse: 2 mm, 41 states. Rotation starts from maximum insertion and rolls the base 0->45->0 degrees: fine increments 0.75 degrees, 121 states (120 new solves); coarse increments 3.75 degrees. Each rotation estimator restores its own insertion estimate, not truth. Rotation tests known base roll and force-induced elastic twist, not a modeled contact friction torque. The 0.02 s timestamp spacing is quasistatic bookkeeping, not a validated dynamic speed.

For sensitivity, eliminate nuisance base moment locally using E=dm(L)/dx and observation Jacobian J:

\[
G=J_f-J_mE_m^{-1}E_f,\qquad \delta w=G\delta f,
\quad\delta f=[\delta F_{tip};\delta F_{contact}].
\]

Use physical units and state the scaling. Let N span the tangent space of the current contact constraints; small singular values of G N identify weak admissible force combinations. Tip and contact loads affect different rod regions, but their bending contributions can nearly compensate. Axial-like force directions have weak bending signatures. Good shape accuracy does not guarantee accurate force decomposition.

### Definition of 0-2 mm noise

Inject independent zero-mean Gaussian noise into two Bishop channels at 24 stations:

\[
w^{obs}_{k,j}=w^{clean}_{k,j}+\epsilon_{k,j},\qquad
\epsilon_{k,j}\sim\mathcal N(0,\sigma_w^2I_2).
\]

Independence holds across stations, channels and frames, except the deliberately shared insertion/rotation seed. Noise level labels are calibrated by geometric integration:

\[
\delta_{tip}=\sqrt{\mathbb E\|p_G(L;w+\epsilon)-p_G(L;w)\|_2^2}.
\]

This is 3D endpoint RMS relative to noise-free geometric reconstruction. It is not axiswise standard deviation, a guaranteed displacement at each frame, independent position noise at every shape node, or force-estimator error. Calibration used 256 draws per coarse reference frame and 512 independent validation draws, with equal scenario weight. The 0.5/1 mm levels scale the calibrated 2 mm curvature standard deviation.

| Target endpoint RMS | Injected curvature std per channel, mm^-1 |
|---|---:|
| 0 mm | 0 |
| 0.5 mm | 9.268141721e-5 |
| 1 mm | 1.853628344e-4 |
| 2 mm | 3.707256688e-4 |

Noiseless runs still assume observation std 1e-4 mm^-1 in the objective; their covariance is not zero. Nonzero-noise runs use injected sigma in the likelihood. No environmental or mu noise is injected. Environmental covariance used as a prior does not mean actual environmental perturbations were simulated.

Fine trajectories reuse coarse noise draws at every fifth frame, adding independent samples between them. Realized endpoint noise RMS is approximately 0.519/1.034/2.076 mm for insertion and 0.529/1.055/2.111 mm for rotation. References were regenerated at finer increments. Common insertion truth forces match coarse values; rotation contact-force truth differs by 0.205 N RMS owing to friction history.

Why 2 mm is large: it is only 1% of length, but force observability is much weaker than that ratio suggests. An earlier conditional sensitivity audit predicted 3.60 N insertion /2.73 N rotation tip-force RMS for this sigma, against the 1 N true tip load. One admissible compensation direction changed 48-channel observation RMS by only 1.744e-5 mm^-1 per 1 N tip-X change. These are earlier fixed-geometry/mode/history local results, not new predictions fitted to the fine runs. Large force errors can occur with correct mode and good FBG fit.

### Why defer S-channel

The earlier S-channel's first contact carried about 65-119 N. Its first reaction changed by 47-57 N per millimeter of wall-normal displacement, versus 0.54-0.62 N/mm for the single plane. Scaled forward in-plane equilibrium/contact Jacobian condition numbers were about 9961 versus 184. Small geometric bias can therefore cause large reaction changes.

High load alone is not proof of ill-conditioning, and not every force change leaves shape unchanged. Geometry-to-force sensitivity, limited measurement response and tip/contact compensation are distinct diagnostics. The original S-channel packet also retained only one bending channel, making its first-order measured tip-Y response zero by planar symmetry. With both channels, that response was comparable to the single-contact case. No buckling/global-stability conclusion was established. Defer this confounded extreme fixture while preserving its sensitivity evidence.

## 11. Measured status and frames to investigate

Eight fine trajectories completed: 1284 new solves, 3710 branch solves, 3851 mechanics IVPs plus BVP work. Total inverse time: 585.6 s, excluding reference generation and serialization. Five selected branches and 287 branches overall reached the 100-iteration cap. Native affine successes: 1279/1284; BVP convergence: 1281/1284; strict nonlinear selected-mode success: 169/1284. Diagnostics remain nonblocking.

| Motion | Noise mm | Tip RMSE N | Contact RMSE N | Shape RMS mm | Mean s/frame |
|---|---:|---:|---:|---:|---:|
| Insertion | 0 | 0.01499 | 0.02327 | 0.01270 | 0.866 |
| Insertion | 0.5 | 0.89389 | 0.99473 | 0.12479 | 0.458 |
| Insertion | 1 | 1.81596 | 2.05821 | 0.29898 | 0.450 |
| Insertion | 2 | 4.25986 | 6.05116 | 7.24930 | 0.447 |
| Rotation | 0 | 0.10874 | 0.31764 | 0.01343 | 0.239 |
| Rotation | 0.5 | 2.23925 | 5.46476 | 1.08166 | 0.276 |
| Rotation | 1 | 2.80057 | 5.57598 | 0.72798 | 0.291 |
| Rotation | 2 | 8.00554 | 10.90361 | 8.58824 | 0.355 |

Errors include failures. Noiseless rotation improves substantially with smaller steps. Insertion shape/contact also improve, but an added intermediate position has a force spike. Matched-position insertion force errors at 0.5/1 mm are nearly unchanged. Noisy rotation is mixed; 2 mm can be seriously unstable. Cov(epsilon_k-epsilon_{k-1})=2 Sigma_w: independent noise differences do not shrink with actuation increments. Smaller steps do not automatically improve slip reliability. This mechanism is consistent with the evidence, not a causal variance partition or statistical ranking from one seed.

| Case | Frame / actuation | Evidence |
|---|---|---|
| Insertion, 0 mm | 99 /distance 40.8 mm | tip/contact error 0.181/0.310 N at an added position |
| Insertion, 0 mm | 102 /first withdrawal, 40.4 mm | shape RMS 0.174 mm; corrected gap 0.129 mm |
| Rotation, 0.5 mm | 91 /return 22.5 degrees | correct sticking; tip error 12.675 N; FBG RMS 5.623 affine, 5.669 pre-BVP, 5.476 after; coarse matched sample had 1.636 N tip error |
| Insertion, 2 mm | 173 /return 68.8 mm | BVP mesh limit; shape RMS 65.85 mm; terminal moment 641 N mm |
| Rotation, 2 mm | 72 /return 36.75 degrees | BVP mesh limit; shape RMS 43.78 mm; terminal moment 686 N mm |
| Rotation, 2 mm | 78 /return 32.25 degrees | singular collocation Jacobian; pre-BVP fallback |

The 0.5 mm spike is not solely a wrong-mode or BVP-nonlinearity failure. Distinguish observation weakness, inconsistent history, geometry mismatch, local optimization and correction failure before changing the objective.

## 12. Prioritized next work

### A. Audit spikes and recurrent history

Owner: incoming implementer; mentor reviews conclusions. Start with the listed failure frames and nearby good frames. Save initializer, distance from expansion point, conditioning, every branch's cost/feasibility/exit, affine and nonlinear FBG fit, gap/slip, base moment, BVP status and previous-state quality. Compare pre/post-BVP states and propagation into subsequent frames. Check derivative/scaling accuracy at difficult states rather than attributing every spike to noise. Separate numerical jumps from genuine friction-force changes at reversal.

Deliverable: a short root-cause table and reproducible saved-frame fixtures. Do not remove failures from metrics. The warning-returned BVP history path is a high-priority audit; changing fallback/history policy needs approval and its own comparison.

### B. Test regularization within the existing framework

Owner: implementer proposes prior/scaling; mentor approves before implementation. One candidate is weak temporal regularization of world-frame forces:

\[
J_{new}=\frac{\alpha}{2}\left[
\|r_w\|^2+\|f_k-\bar f_k^-\|_{Q_{f,k}^{-1}}^2
\right],\qquad f_k=[F_{tip,k};F_{1,k};\ldots].
\]

Match contact identities before imposing a prior. Use interpretable covariance and force units, with separate tip/contact uncertainty if justified. A history predictor may be preferable to simply the last force. Weak-direction regularization may stabilize load decomposition without dominating well-observed directions, but introduces prior information and bias; it cannot create sensor observability.

Permit real changes at reversal, contact onset/release and mode changes. Strong blanket smoothing can hide true transitions or preserve a bad previous estimate. The bookkeeping timestamps do not justify a physical process-noise law. Robust/mode-aware priors, uncertainty-aware history and failed-correction handling are proposals, not implemented decisions.

Use paired observations and multiple seeds for unregularized/regularized comparisons. Report force RMS and peaks, shape error, jitter, lag, physical consistency, failure rate and runtime. Video smoothness alone is insufficient; do not only smooth displayed trajectories. Keep prior/covariance definitions comparable across modes so objective-based mode selection remains meaningful. Record consequences for the 1.24 FBG acceptance statistic.

### C. Test solvers other than MATLAB fmincon SQP

This is explicitly requested future work. Preserve frozen geometry, one shared rod linearization, exhaustive full-cone assignments and one BVP correction. Separate optimizer replacement from physics/approximation changes.

Test a QP solver for separation, a convex quadratic/conic solver for sticking, and alternative nonlinear optimizers for exact sliding. IPOPT is a candidate already used in the historical relaxation experiments; those timings do not establish its performance on the current branch problems. Evaluate other available solvers after checking interfaces, derivative support and licensing. A different fmincon algorithm is a useful within-MATLAB comparison, but not the only alternative.

Compare identical saved expansion points, histories, initializations, scaling, noise and constraints. Measure achieved feasibility/accuracy, branch failures, median/tail runtime, evaluation counts and full recurrent behavior. Iteration counts are not directly comparable across algorithms. Supply explicit quadratic Hessians for QP/conic branches where appropriate; evaluate exact versus approximate second derivatives separately for sliding. Include setup/data-conversion overhead and BVP cost in timing.

Do not claim global optimality from exhaustive local solves. Freezing sliding direction or using a polygon is a separate approximation requiring approval; neither is necessary merely to benchmark another optimizer. Retain current fmincon as the reference.

### D. Scientific and experimental follow-up

Quantify tip/contact coupling over the location sweep, separating it from changes in load/configuration where possible. Repeat noise tests with multiple seeds; add correlated/bias noise only after agreeing on an experimental noise model. Examine Stage 1 uncertainty and history/slip uncertainty rather than treating the noisy previous shape as exact. FBG-derived shape residuals do not supply independent information beyond the same underlying data.

Validate experimental frame/gauge, curvature scale/angle, base pose, stiffness and friction coefficient. Rotation currently tests elastic torsion and 3D force coupling, not direct distributed torsional friction. Tip/edge/corner contact remains deferred. TDCR/multiple-contact demonstrations are later extensions after single-contact stability is understood; preserve general 3D and sparse multi-contact interfaces rather than hard-coding this fixture.

## 13. Code map and reproducibility

| Responsibility | Entry point |
|---|---|
| Dependency setup | [setup_tsfs.m](../setup_tsfs.m) |
| Geometric estimation | [geometry.m](../matlab/+tsfs/geometry.m) |
| Bishop reconstruction / observation | [reconstruct_bishop.m](../matlab/+tsfs/reconstruct_bishop.m), [bishop_observation.m](../matlab/+tsfs/bishop_observation.m) |
| Nonlinear variational IVP | [ivp.m](../matlab/+tsfs/ivp.m) |
| Affine model / evaluation | [linearized_model.m](../matlab/+tsfs/linearized_model.m), [linearized_evaluate.m](../matlab/+tsfs/linearized_evaluate.m) |
| Exhaustive one-pass selection | [single_linearization_solve.m](../matlab/+tsfs/single_linearization_solve.m) |
| Full-cone branch / nonlinear check | [mode_branch.m](../matlab/+tsfs/mode_branch.m), [mode_residual.m](../matlab/+tsfs/mode_residual.m) |
| Correction wrapper / BVP | [single_pass_bvp_solve.m](../matlab/+tsfs/single_pass_bvp_solve.m), [fixed_load_bvp.m](../matlab/+tsfs/fixed_load_bvp.m) |
| Fine-step runner | [run_fine_bvp_experiment.m](../experiments/inclined_wall/run_fine_bvp_experiment.m) |
| Analysis / media | [analyze_fine_bvp.py](../experiments/inclined_wall/analyze_fine_bvp.py), [publish_fine_bvp_media.py](../experiments/inclined_wall/publish_fine_bvp_media.py), [validate_fine_bvp_media.py](../experiments/inclined_wall/validate_fine_bvp_media.py) |

The runner explicitly selects single_pass_bvp_solve. Older mode_solve/successive-linearization defaults do not describe this method. Generic defaults still contain priors used by older solvers; fine-run overrides and sensor-only residual slicing govern the current experiment.

Follow the [package README](../README.md) for dependencies and provenance. MATLAB requires the configured read-only rod/ and LCP-Continuum dependencies and the optimization/BVP functions above. baselineRoot in ignored .local.json or TSFS_BASELINE_ROOT can select the baseline checkout. Do not copy collaborator algorithms into our package. Python analysis/rendering uses NumPy, SciPy, h5py, Matplotlib and imageio-ffmpeg.

Final stamp: `20261012_step5`. Published outputs:

- [Numerical report](../docs/reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md), [summary CSV](../results/published/fine_bvp_comparison_20261012_step5/summary.csv), [per-frame audit](../results/published/fine_bvp_comparison_20261012_step5/frames.csv).
- [All videos/plots](../results/published/fine_bvp_comparison_20261012_step5/media/README.md), [local HTML gallery](../results/published/fine_bvp_comparison_20261012_step5/media/index.html).
- [Exact noise inputs and reference audits](../results/published/fine_bvp_comparison_20261012_step5/reproduction/).

For a fresh numerical rerun, from the package directory:

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
stamp = 'handoff_replay_20261013'; % choose an unused stamp
inputFolder = fullfile('results','runs',['fine_bvp_inputs_' stamp]);
if ~isfolder(inputFolder), mkdir(inputFolder); end
published = fullfile('results','published', ...
    'fine_bvp_comparison_20261012_step5','reproduction');
copyfile(fullfile(published,'noise_payload.mat'), inputFolder);
copyfile(fullfile(published,'noise_protocol.json'), inputFolder);
run_fine_bvp_experiment(stamp);
```

This regenerates physical references and inverse trajectories. Completed outputs are skipped; partial inverse outputs are rejected rather than overwritten. Full analysis needs raw MATLAB checkpoints under local-only results/runs/. A fresh clone must regenerate those or obtain the existing local data. The analyzer also requires the preserved coarse fixed-load-BVP published summaries and estimate CSVs (included on the branch), plus the historical local results/runs/noisy_linearized_20261011/noise_payload.mat for its exact common-noise assertion. Obtain that historical payload from the existing local workspace before full analysis; the current analyzer does not automatically reconstruct it from every fifth fine-noise sample. Historical test scripts may require their corresponding old local fixtures.

```text
python experiments/inclined_wall/analyze_fine_bvp.py handoff_replay_20261013
python experiments/inclined_wall/publish_fine_bvp_media.py --stamp handoff_replay_20261013
python experiments/inclined_wall/validate_fine_bvp_media.py handoff_replay_20261013
```

For the published stamp, compact media/shapes.npz sources plus published CSVs permit rerendering without large checkpoints. Each video includes all 201 insertion or 121 rotation states, including the restored rotation seed. Validation checks shape/force errors against CSVs and fully decodes videos for frame count/duration checks. No failed frame is removed.

Further reading: [noise/force sensitivity](../docs/reports/NOISY_FORCE_SENSITIVITY.md), [S-channel/plane sensitivity](../docs/reports/PLANE_CONTACT_SENSITIVITY.md), [load compensation](../docs/reports/LOAD_TRANSFER_SHAPE_COUPLING.md), [Bishop observation contract](../docs/formulation/BISHOP_OBSERVATION_CONTRACT.md), [BVP equations](../docs/formulation/FIXED_LOAD_BVP_CORRECTION.md), [deferred physics](../docs/decisions/DEFERRED_WORK.md).

## 14. Expected next handoff

Provide reproducible spike diagnoses separating measurement limits from numerical/history failures. For each approved change, compare paired trajectories and multiple seeds, without hiding failures. Preserve generic 3D/sparse-contact interfaces. Report speed at equivalent physical accuracy, not native solver exits alone. Include a comparison beyond fmincon SQP, regularization tradeoffs (accuracy, jitter and lag), and an explicit proposal for untrusted corrected states. No assumption changes should be implicit in a speedup claim.
