function report = validate_aloi_gaussian_baseline()
%VALIDATE_ALOI_GAUSSIAN_BASELINE Check the baseline inside its load model.
%
% This is a noiseless same-model consistency test. It generates a shape
% from one known local-transverse Gaussian load, then estimates that load
% from 24 sparse centerline positions. It does not test contact physics.

packageDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(packageDir);
validate_lcp_dependency(rootDir);

tube = CreatTube(150);
thetaTrue = [4.0; -8.0; 110.0; 12.0];
truth = solveKnownGaussianTruth(tube, thetaTrue, 100, 1e-8, 0.5);

measurementIdx = unique(round(linspace(1, numel(tube.s), 24)), 'stable');
measurements = struct;
measurements.betaMm = 0;
measurements.baseTraj = reshape(tube.T_base, 4, 4, 1);
measurements.fbgIdx = measurementIdx;
measurements.pSparse = reshape(truth.p(:, measurementIdx), 3, [], 1);

cfg = struct;
cfg.aloi.sigmaCandidatesMm = [3, 6, 12, 20, 35];
cfg.aloi.numCenterCandidates = 21;
cfg.aloi.amplitudeBoundN = 100;
cfg.aloi.positionStdMm = 0.2;
cfg.aloi.maxIterations = 80;
cfg.aloi.maxFunctionEvaluations = 800;
cfg.aloi.numOptimizationStarts = 5;
cfg.aloi.maxEquilibriumIterations = 60;
cfg.aloi.equilibriumToleranceMm = 1e-7;
cfg.aloi.equilibriumRelaxation = 0.5;
cfg.aloi.showProgress = true;

estimate = estimate_aloi_gaussian_baseline(tube, measurements, cfg);
forceErrorN = norm(estimate.totalForceResultant(:, 1) - truth.forceResultant);
relativeForceErrorPct = 100 * forceErrorN / norm(truth.forceResultant);
parameterError = estimate.parameters(:, 1) - thetaTrue;

report = struct;
report.thetaTrue = thetaTrue;
report.thetaEstimated = estimate.parameters(:, 1);
report.parameterError = parameterError;
report.trueForceResultant = truth.forceResultant;
report.estimatedForceResultant = estimate.totalForceResultant(:, 1);
report.forceErrorN = forceErrorN;
report.relativeForceErrorPct = relativeForceErrorPct;
report.shapeRmseMm = estimate.shapeRmseMm(1);
report.equilibriumResidualMm = estimate.equilibriumResidualMm(1);
report.solverExitFlag = estimate.solverExitFlag(1);
report.truthEquilibriumResidualMm = truth.equilibriumResidualMm;

fprintf('\nAloi in-domain consistency check\n');
fprintf('  true theta [a1 a2 center sigma]: [%.6g %.6g %.6g %.6g]\n', ...
    thetaTrue);
fprintf('  estimated theta:                 [%.6g %.6g %.6g %.6g]\n', ...
    estimate.parameters(:, 1));
fprintf('  true resultant:      [%.6g %.6g %.6g] N\n', truth.forceResultant);
fprintf('  estimated resultant: [%.6g %.6g %.6g] N\n', ...
    estimate.totalForceResultant(:, 1));
fprintf('  force mismatch: %.6g %%\n', relativeForceErrorPct);
fprintf('  sparse-position RMSE: %.6g mm\n', estimate.shapeRmseMm(1));

assert(truth.equilibriumResidualMm < 1e-7, ...
    'Known-load truth did not reach nonlinear equilibrium.');
assert(estimate.equilibriumConverged(1), ...
    'Estimated load did not reach nonlinear equilibrium.');
assert(estimate.shapeRmseMm(1) < 1e-4, ...
    'Aloi baseline did not reproduce its in-domain target shape.');
assert(relativeForceErrorPct < 0.1, ...
    'Aloi baseline did not recover its in-domain known load.');
end


function truth = solveKnownGaussianTruth(tube, theta, maxIterations, toleranceMm, relaxation)
s = tube.s(:);
density = normalizedGaussian(s, theta(3), theta(4));
localLoad = [theta(1) * density'; theta(2) * density'; zeros(1, numel(s))];
weights = trapezoidalWeights(s);
[~, R, p] = solveShape(tube.T_base, tube.uhat, tube.s);
invK = 1 ./ getTubeK(tube);
u = tube.uhat;

for iteration = 1:maxIterations
    nodalForces = transformAndIntegrate(localLoad, R, weights);
    J = computeJacobian(R, p);
    equilibriumU = reshape(invK .* (J' * nodalForces(:)), 3, []) + tube.uhat;
    u = (1 - relaxation) * u + relaxation * equilibriumU;
    [~, nextR, nextP] = solveShape(tube.T_base, u, tube.s);
    updateResidualMm = sqrt(mean(sum((nextP - p) .^ 2, 1)));
    R = nextR;
    p = nextP;
    if updateResidualMm <= relaxation * toleranceMm
        break;
    end
end

nodalForces = transformAndIntegrate(localLoad, R, weights);
J = computeJacobian(R, p);
equilibriumU = reshape(invK .* (J' * nodalForces(:)), 3, []) + tube.uhat;
[~, ~, equilibriumP] = solveShape(tube.T_base, equilibriumU, tube.s);

truth = struct;
truth.p = p;
truth.R = R;
truth.forceResultant = sum(nodalForces, 2);
truth.equilibriumResidualMm = ...
    sqrt(mean(sum((equilibriumP - p) .^ 2, 1)));
truth.iterations = iteration;
end


function nodalForces = transformAndIntegrate(localLoad, R, weights)
distributedLoad = zeros(size(localLoad));
for i = 1:size(localLoad, 2)
    distributedLoad(:, i) = R(:, :, i) * localLoad(:, i);
end
nodalForces = distributedLoad .* reshape(weights, 1, []);
end


function density = normalizedGaussian(s, center, sigma)
phi = exp(-0.5 * ((s - center) ./ sigma) .^ 2);
density = phi / trapz(s, phi);
end


function weights = trapezoidalWeights(s)
weights = zeros(size(s));
weights(1) = 0.5 * (s(2) - s(1));
weights(end) = 0.5 * (s(end) - s(end - 1));
weights(2:end - 1) = 0.5 * (s(3:end) - s(1:end - 2));
end
