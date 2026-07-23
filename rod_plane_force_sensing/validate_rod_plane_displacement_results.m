function report = validate_rod_plane_displacement_results(results)
%VALIDATE_ROD_PLANE_DISPLACEMENT_RESULTS Check physics, not target accuracy.

estimatedForce = results.ours.contactForceResultant;
mu = results.forward.frictionMu(:)';
estimatedNormalDirection = results.ours.planeNormal;
estimatedNormalDirection = estimatedNormalDirection ./ ...
    max(vecnorm(estimatedNormalDirection, 2, 1), eps);

estimatedNormal = sum(estimatedNormalDirection .* estimatedForce, 1);
estimatedTangential = estimatedForce - estimatedNormalDirection .* estimatedNormal;
estimatedTangentialNorm = vecnorm(estimatedTangential, 2, 1);
coneViolation = estimatedTangentialNorm - mu .* max(estimatedNormal, 0);
frictionless = mu <= 1e-12;

assert(all(isfinite(results.forward.p(:))), 'Forward shape contains nonfinite values.');
assert(all(isfinite(results.ours.p(:))), 'Estimated shape contains nonfinite values.');
assert(all(isfinite(estimatedForce(:))), 'Estimated contact force contains nonfinite values.');
assert(all(isfinite(estimatedNormalDirection(:))), ...
    'Estimated plane normal contains nonfinite values.');
assert(max(coneViolation) < 1e-4, 'Estimated force violates the Coulomb cone.');
if any(frictionless)
    assert(max(estimatedTangentialNorm(frictionless)) < 1e-4, ...
        'The inverse estimated friction while the known coefficient was zero.');
end
assert(max(results.ours.normalComplementarity) < 1e-3, ...
    'Normal complementarity residual is too large.');
assert(max(results.ours.frictionComplementarity) < 1e-3, ...
    'Friction complementarity residual is too large.');
assert(max(results.ours.coneComplementarity) < 1e-3, ...
    'Friction-cone complementarity residual is too large.');

truthContact = vecnorm(results.forward.contactForceResultant, 2, 1) > 1e-8;
if any(truthContact)
    assert(max(results.truthConsistency.maxInequalityViolation(truthContact)) < 5e-2, ...
        'Forward truth violates the nonlinear formulation inequalities.');
    assert(max(results.truthConsistency.maxEqualityResidual(truthContact)) < 5e-1, ...
        'Forward truth violates the nonlinear formulation equalities.');
end

forceError = estimatedForce + results.ours.tipForce - ...
    results.forward.totalForceResultant;
report = struct;
report.totalForceRmseN = sqrt(mean(vecnorm(forceError, 2, 1) .^ 2));
report.finalRelativeErrorPct = results.metrics.ours.finalRelativeErrorPct;
report.maxShapeRmseMm = max(results.ours.shapeRmseMm);
report.maxFrictionlessTangentialForceN = max([0, estimatedTangentialNorm(frictionless)]);
report.maxConeViolationN = max(coneViolation);
report.maxComplementarityResidual = max([results.ours.normalComplementarity, ...
    results.ours.frictionComplementarity, results.ours.coneComplementarity]);
report.maxTruthInequalityViolation = max([0, ...
    results.truthConsistency.maxInequalityViolation(truthContact)]);
report.maxTruthEqualityResidual = max([0, ...
    results.truthConsistency.maxEqualityResidual(truthContact)]);

fprintf('\n=== Displacement force-sensing checks passed ===\n');
fprintf('Total-force trajectory RMSE: %.6g N\n', report.totalForceRmseN);
fprintf('Final relative force error: %.6g %%\n', report.finalRelativeErrorPct);
fprintf('Maximum shape RMSE: %.6g mm\n', report.maxShapeRmseMm);
fprintf('Maximum zero-mu tangential estimate: %.6g N\n', ...
    report.maxFrictionlessTangentialForceN);
fprintf('Maximum Coulomb-cone violation: %.6g N\n', report.maxConeViolationN);
fprintf('Maximum complementarity residual: %.6g\n', ...
    report.maxComplementarityResidual);
fprintf('Maximum forward-truth inequality violation: %.6g\n', ...
    report.maxTruthInequalityViolation);
fprintf('Maximum forward-truth equality residual: %.6g\n', ...
    report.maxTruthEqualityResidual);
end
