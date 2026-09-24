function report = validate_rod_plane_displacement_results(results)
%VALIDATE_ROD_PLANE_DISPLACEMENT_RESULTS Check physics, not target accuracy.
% This remains the strict full-friction gate. The uncertainty-aware branch
% intentionally cannot pass it while friction kinematics are unresolved.
if isfield(results.ours,'frictionKinematicsEnforced') && ...
        any(~results.ours.frictionKinematicsEnforced)
    error('rod:UnresolvedFrictionKinematics', ...
        'Friction kinematics are unresolved in frames %s; cone feasibility does not certify full complementarity. See force(''noise'') for the scoped noise regression.', ...
        mat2str(find(~results.ours.frictionKinematicsEnforced)));
end

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
assert(all(isfinite(results.ours.tipForce(:))), 'Estimated tip force contains nonfinite values.');
assert(all(isfinite(results.forward.totalForceResultant(:))), 'True total force contains nonfinite values.');
shapeDifference = results.ours.p - results.measurements.p;
shapeRmseMm = reshape(sqrt(mean(sum(shapeDifference.^2,1),2)),1,[]);
assert(all(isfinite(estimatedNormalDirection(:))), ...
    'Estimated plane normal contains nonfinite values.');
assert(all(isfinite(shapeRmseMm)) && max(shapeRmseMm) < 0.5, ...
    'Estimated shape differs by more than 0.5 mm; a feasible force alone is not a successful reconstruction.');
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
oracleHistory=~isfield(results.measurements,'historySource') || ...
    strcmpi(results.measurements.historySource,'oracle');
truthInequality=results.truthConsistency.maxInequalityViolation(truthContact);
truthEquality=results.truthConsistency.maxEqualityResidual(truthContact);
truthConsistencyEvaluated=all(isfinite(truthInequality)) && all(isfinite(truthEquality));
% Only an oracle predecessor promises consistency with the incremental
% forward model. Noisy measured history is a model-input perturbation; its
% truth residual remains reported, not treated as a forward physics failure.
if any(truthContact) && oracleHistory
    assert(truthConsistencyEvaluated, ...
        'Oracle truth-consistency diagnostics are missing or nonfinite.');
    assert(max(results.truthConsistency.maxInequalityViolation(truthContact)) < 5e-2, ...
        'Forward truth violates the nonlinear formulation inequalities.');
    assert(max(results.truthConsistency.maxEqualityResidual(truthContact)) < 5e-1, ...
        'Forward truth violates the nonlinear formulation equalities.');
end

forceError = estimatedForce + results.ours.tipForce - ...
    results.forward.totalForceResultant;
report = struct;
report.scope = 'Shape consistency and contact/friction checks; not a force-accuracy or optimizer-convergence certificate.';
report.oracleHistory=oracleHistory;
report.friction = validate_friction(results);
report.totalForceRmseN = sqrt(mean(vecnorm(forceError, 2, 1) .^ 2));
report.finalRelativeErrorPct = 100*norm(forceError(:,end))/max(norm(results.forward.totalForceResultant(:,end)),eps);
report.maxShapeRmseMm = max(shapeRmseMm);
report.contactForceRmseN = sqrt(mean(sum((estimatedForce-results.forward.contactForceResultant).^2,1)));
report.tipForceRmseN = sqrt(mean(sum((results.ours.tipForce-results.forward.tipLoad).^2,1)));
trueShapeDifference = results.ours.p-results.forward.p;
report.maxTrueShapeRmseMm = max(reshape(sqrt(mean(sum(trueShapeDifference.^2,1),2)),1,[]));
report.cachedShapeMetricMaxDifferenceMm = max(abs(shapeRmseMm-results.ours.shapeRmseMm));
report.maxFrictionlessTangentialForceN = max([0, estimatedTangentialNorm(frictionless)]);
report.maxConeViolationN = max(coneViolation);
report.maxComplementarityResidual = max([results.ours.normalComplementarity, ...
    results.ours.frictionComplementarity, results.ours.coneComplementarity]);
report.truthConsistencyEvaluated = truthConsistencyEvaluated;
if truthConsistencyEvaluated
    report.maxTruthInequalityViolation = max([0, truthInequality]);
    report.maxTruthEqualityResidual = max([0, truthEquality]);
else
    % MATLAB max can omit NaNs: do not turn an unevaluated diagnostic into
    % an apparent zero-residual success. jsonencode exports NaN as null.
    report.maxTruthInequalityViolation = NaN;
    report.maxTruthEqualityResidual = NaN;
end

fprintf('\n=== Shape consistency and contact/friction checks passed ===\n');
fprintf('This does not certify force accuracy or optimizer convergence.\n');
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
