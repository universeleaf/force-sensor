# FBG curvature, material frames, and reconstruction

Status: mathematical clarification discussed 2026-10-05; no estimator change adopted during folder migration.

With arc length s, material orientation R=[d1 d2 d3], t=p'=d3 and R'=R[u]_x, the Cosserat strain u=[u1,u2,u3] contains two bending components and material twist. Thus t'=u2*d1-u1*d2 and kappa=sqrt(u1^2+u2^2).

If the device's angle phi measures the direction of t' from d1, the conversion is u1=-kappa*sin(phi), u2=kappa*cos(phi). The actual FBG device convention, signs, reference frame and any intrinsic-curvature subtraction must be confirmed before applying this mapping to real measurements.

The same centerline can have different material frames: Rtilde=R*Q3(psi) leaves t unchanged while utilde=Q3(psi)'*u+psi'*e3. This is frame freedom for a geometric curve; physical FBG cores fix a material frame, so model and data cannot be independently rotated in a measurement residual. Where kappa>0, centerline torsion is phi'+u3 for the convention above. Two bending signals alone do not identify arbitrary material twist.

Our objective selects observed axes only, rather than filling unmeasured twist with a zero observation. The original S-channel fixture has one measured bending channel. Stage 1 uses calibrated intrinsic values for unobserved components; this is a reconstruction assumption, not an extra measurement. Excluding contact friction torque does not imply zero elastic twist.

## Direct constant-curvature reconstruction

A sensor-centered piecewise-constant alternative assigns each sensor value to the interval between adjacent sensor midpoints. Exact constant-strain SE(3) integration gives continuous position and tangent. Preserve known intrinsic-curvature discontinuities. Evaluate position and tangent from the same integrated segment if this alternative is adopted.

The separate `audit_fbg_integration` experiment compared PCHIP, left hold, adjacent average, and sensor-centered hold on both 12-frame datasets. Mean frame shape RMS (mm):

| Method | Sliding | S-channel |
|---|---:|---:|
| Existing PCHIP | 0.0008124 | 0.0299888 |
| Left hold | 0.506218 | 1.774583 |
| Adjacent average | 0.0155184 | 0.0414171 |
| Sensor-centered | 0.0024702 | 0.0324662 |

This was reconstruction-only. Sensor-centered integration was discussed as the preferred interpolation-free option, not approved as an estimator replacement. Existing PCHIP remains active. Sparse reconstruction bias and twist uncertainty remain open issues.
