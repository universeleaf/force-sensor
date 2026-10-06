# Deferred scope, agreed 2026-10-05

1. **Unmodeled friction torque / twist:** no distributed torsional friction or contact-couple jump is added. The rod retains its calibrated torsional stiffness and elastic torsion from point-force loading. Missing twist sensing in geometric reconstruction is separately reported; these are different limitations.
2. **Tip contact:** no environmental contact hypothesis or endpoint complementarity at the tip. The unknown applied tip force remains an estimated load. Tip moment is zero in this version.
3. **Edges and corners:** only smooth interior contact with planes is supported. No corner normal cone, surface switching at an edge, or endpoint tangency model is claimed.

Other measured follow-up needs: sparse-FBG reconstruction model-error calibration; propagation of unobserved twist uncertainty; force observability with one measured bending channel; frozen-geometry mismatch; globally reliable semismooth SQP on incompatible or degenerate hypotheses; geometry-induced force covariance. No within-step geometry correction has been added. Diagnostics remain reporting-only.
