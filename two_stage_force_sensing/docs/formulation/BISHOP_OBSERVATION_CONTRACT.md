# Bishop curvature observation contract

Approved 2026-10-07 for simulation; experimental angle calibration is deferred.

Let R be the material frame, B=[b1,b2,t] the Bishop frame, and R=B*Rz(psi). Set B(0)=R(0), psi(0)=0. The observation is defined by t'=w1*b1+w2*b2; hence w=R2(psi)[u2,-u1] and psi'=u3. The angular strain integrating B is [-w2,w1,0]. Do not confuse these tangent-curvature components with the first two angular-strain components.

The simulator adapter converts independently generated full material mechanics to sampled w before packaging measurements. The estimator packet identifies observationType='bishop-curvature' and carries bishopCurvaturePerMm and previousBishopCurvaturePerMm (2-by-station-by-frame), with sensor arc lengths and known base poses. No true twist, contact location, forces or dense true shape is passed to the estimator. The outer saved benchmark file keeps truth separate for scoring.

Stage 1 reconstructs only from the sampled w and model calibration, then estimates surface geometry/contact arc as before. PCHIP interpolation is confined to the existing piecewise calibrated material segments. It requires at least two stations per segment, extrapolates within each segment where necessary, and integrates by SE(3) midpoint steps. All intrinsic/stiffness breaks must occur on the rod integration grid. Current automatic break detection is for piecewise-constant calibration; continuously varying calibration needs an explicit discontinuity interface before use.

Stage 2's material IVP adds psi and its variational sensitivity. Predictions are evaluated at the actual sensor arc lengths. With C=[0 1;-1 0], A=[0 -1;1 0], delta_w=R2(psi)*C*delta_u_perp+A*w*delta_psi. Full material constitutive laws retain intrinsic strain and all three material curvature components.

The MAP observation residual is chol(CU,'lower')\(predicted_w(:)-measured_w(:)); existing force priors and mechanical/contact constraints remain. The likelihood appears once in Stage 2. Stage 1 geometry is plugged in rather than marginalized, so this remains the approved approximate two-stage inference.

A Bishop frame is not a measured material frame. The linear moment initializer uses its zero-roll interpretation only as an approximate seed. Option bishopWarmStart reuses a previous converged material-state solution when contacts match. Neither seed is uniformly reliable; the rotation report preserves both outcomes and a same-history comparison. Do not silently use reference material twist to improve either initializer.

The default untagged material-curvature path remains available for previous experiments. Unsupported or insufficient segment sampling must fail explicitly, not be filled using true state. The new ideal geometric observation assumption does not prove that experimental FBGS curvature angles are Bishop angles or that the device compensates material twist.
