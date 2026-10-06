# Curvature matching versus reconstructed-shape matching

Status: discussed alternative; the implemented Stage 2 objective is unchanged.

For measured bending stack y, predicted observations h(x), and covariance Sigma_u, the current measurement cost is 0.5*(h(x)-y)'*inv(Sigma_u)*(h(x)-y). It includes only measured channels, plus separate force priors in the full objective.

A shape objective could compare mechanically predicted positions P(x) with G(y), the reconstructed positions. Linearizing the reconstruction gives deltaP=A*deltay and Sigma_p=A*Sigma_u*A'. If model and data use the same reconstruction map and A preserves all measured information, the correctly whitened local shape cost with a pseudoinverse is equivalent to the curvature cost. Otherwise information can be lost or additional reconstruction discrepancy introduced.

Unweighted shape RMSE instead weights curvature residuals by A'*A. For small planar bending with fixed base pose, delta p_y(s)=integral_0^s (s-xi)*delta kappa(xi) dxi. Upstream errors propagate to multiple points, neighboring errors are correlated, and oscillating curvature errors can cancel. Adding more sampled positions does not add independent FBG measurements.

A reconstructed planar shape from the one-channel S-channel packet is not a measurement that out-of-plane displacement is zero. Treating it as such adds a planar assumption. Likewise unmeasured twist needs uncertainty or an explicit model assumption, not an invented zero measurement.

Recommendation discussed: retain observed-curvature matching in mechanics, use reconstruction in geometry, and compare positions/tangents in diagnostics. Any future shape-fitting objective must account for propagated covariance, reconstruction error, and frame/twist assumptions. Do not add shape and curvature costs as independent evidence from the same FBG data.
