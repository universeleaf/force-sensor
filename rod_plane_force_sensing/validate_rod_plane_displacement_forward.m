function report = validate_rod_plane_displacement_forward()
%VALIDATE_ROD_PLANE_DISPLACEMENT_FORWARD Physics checks for the copied solver.

packageDir = fileparts(mfilename('fullpath'));
rootDir = packageDir;
if ~exist(fullfile(rootDir, 'LCP-Continuum'), 'dir')
    rootDir = fileparts(packageDir);
end
overrides = struct;
overrides.outputDir = fullfile(rootDir, 'force_outputs', ...
    'rod_plane_displacement_forward_validation_tmp');
overrides.exposedLengthMm = 150;
overrides.scalePrecurvature = false;
overrides.basePositionMm = [-120; 0; 0];
overrides.planePointMm = [20; 0; 0];
overrides.planeNormal = [-1; 0; 0];
overrides.frictionMu = 0.5;
overrides.initialLowFrictionMu = 0.0;
overrides.tipLoadN = zeros(3, 1);
overrides.numTimeSteps = 10;
overrides.forward.motionMode = 'push-slide';
overrides.forward.pushDistanceMm = 30;
overrides.forward.slideDistanceMm = 0.1;
overrides.forward.pushFraction = 4 / 9;
overrides.forward.slideDirection = [0; 0; 1];
overrides.video.enabled = false;
overrides.diagnostics.stopAfterTruthConsistency = true;
overrides.diagnostics.runMapCandidateCosts = false;

results = run_rod_plane_force_sensing_experiment(true, overrides);
forward = results.forward;
n = results.config.planeNormal(:);

normalLoad = n' * forward.contactForceResultant;
tangentialForce = forward.contactForceResultant - n * normalLoad;
tangentialLoad = vecnorm(tangentialForce, 2, 1);
coneViolation = tangentialLoad - forward.frictionMu .* max(normalLoad, 0);

isPush = strcmp(forward.phase, 'push');
isSlide = strcmp(forward.phase, 'slide');
isSlideContact = isSlide & normalLoad > 1e-8;

fprintf('Truth inequality residuals: %s\n', ...
    mat2str(results.truthConsistency.maxInequalityViolation, 4));
fprintf('Truth equality residuals:   %s\n', ...
    mat2str(results.truthConsistency.maxEqualityResidual, 4));
fprintf('Truth gaps (mm):            %s\n', ...
    mat2str(results.truthConsistency.gap, 4));
fprintf('Truth normal forces (N):    %s\n', ...
    mat2str(results.truthConsistency.normalForce, 4));
fprintf('Truth normal comp:          %s\n', ...
    mat2str(results.truthConsistency.normalComplementarity, 4));
fprintf('Truth force reconstruction: %s\n', ...
    mat2str(results.truthConsistency.forceReconstructionResidual, 4));
fprintf('Truth measurement residual: %s\n', ...
    mat2str(results.truthConsistency.measurementResidualNorm, 4));
for frame = find(normalLoad > 1e-8)
    contact = forward.contacts{frame}(1);
    appliedIndex = contact.applied_tube_point_id;
    trackingIndex = contact.tube_point_id;
    appliedPoint = forward.p(:, appliedIndex, frame);
    fprintf(['  contact frame %d: applied id=%s, tracking id=%s, ', ...
        'applied p=[%.4f %.4f %.4f], projected p=[%.4f %.4f %.4f]\n'], ...
        frame, mat2str(appliedIndex), mat2str(trackingIndex), appliedPoint, contact.point);
end

assert(all(isfinite(forward.p(:))), 'Forward shape contains nonfinite values.');
assert(all(isfinite(forward.contactForceResultant(:))), ...
    'Forward contact force contains nonfinite values.');
assert(any(isSlideContact), 'The validation trajectory never reaches sliding contact.');
assert(forward.contactArcLength(end) < forward.s(end) - 5, ...
    'The final contact remains at the tip and cannot be separated from a tip load.');
assert(max(tangentialLoad(isPush)) < 1e-6, ...
    'The frictionless push phase produced a nonzero tangential load.');
pushContact = isPush & normalLoad > 1e-8;
assert(max(results.truthConsistency.maxInequalityViolation(pushContact)) < 5e-2, ...
    'The frictionless forward contact violates the nonlinear formulation inequalities.');
assert(max(results.truthConsistency.maxEqualityResidual(pushContact)) < 5e-1, ...
    'The frictionless forward contact violates the nonlinear normal complementarity equation.');
assert(max(tangentialLoad(isSlideContact)) > 1e-3, ...
    'The sliding phase produced no measurable tangential friction force.');
assert(max(coneViolation(isSlideContact)) < 1e-6, ...
    'The forward force violates the polyhedral Coulomb cone.');
assert(max(vecnorm(forward.totalForceResultant - ...
    forward.contactForceResultant, 2, 1)) < 1e-12, ...
    'Zero tip load did not produce contact-only total force.');
assert(max(results.truthConsistency.maxInequalityViolation(isSlideContact)) < 5e-2, ...
    ['The forward trajectory does not satisfy the nonlinear formulation ', ...
     'inequalities at sampled sliding frames. Reduce the internal step size.']);
assert(max(results.truthConsistency.maxEqualityResidual(isSlideContact)) < 5e-1, ...
    ['The forward trajectory does not satisfy the nonlinear formulation ', ...
     'complementarity equations at sampled sliding frames.']);

report = struct;
report.maxPushTangentialForceN = max(tangentialLoad(isPush));
report.maxSlideTangentialForceN = max(tangentialLoad(isSlideContact));
report.maxSlideNormalForceN = max(normalLoad(isSlideContact));
report.maxConeViolationN = max(coneViolation(isSlideContact));
report.maxTruthInequalityViolation = ...
    max(results.truthConsistency.maxInequalityViolation(isSlideContact));
report.maxTruthEqualityResidual = ...
    max(results.truthConsistency.maxEqualityResidual(isSlideContact));
report.slideContactFrames = find(isSlideContact);

fprintf('\n=== Forward displacement validation passed ===\n');
fprintf('Slide-contact frames: %s\n', mat2str(report.slideContactFrames));
fprintf('Max push tangential force: %.6g N\n', report.maxPushTangentialForceN);
fprintf('Max slide tangential force: %.6g N\n', report.maxSlideTangentialForceN);
fprintf('Max slide normal force: %.6g N\n', report.maxSlideNormalForceN);
fprintf('Max Coulomb-cone violation: %.6g N\n', report.maxConeViolationN);
fprintf('Max nonlinear truth inequality violation: %.6g\n', ...
    report.maxTruthInequalityViolation);
fprintf('Max nonlinear truth complementarity residual: %.6g\n', ...
    report.maxTruthEqualityResidual);
end
