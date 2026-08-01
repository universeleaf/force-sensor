function aloi = estimate_aloi_gaussian_baseline(tube0, measurements, cfg)
%ESTIMATE_ALOI_GAUSSIAN_BASELINE Fit one shape-only Gaussian load per frame.
%
% The estimator uses sparse centerline positions only. Plane geometry,
% friction, contact state, and forward-simulation forces are intentionally
% withheld so that it remains an independent Aloi-style comparison.

fprintf('\nEstimating Aloi Gaussian position-fit baseline...\n');

nt = numel(measurements.betaMm);
ns = numel(tube0.s);
forceResultant = zeros(3, nt);
centerMm = nan(1, nt);
sigmaMm = nan(1, nt);
cost = nan(1, nt);
shapeRmseMm = nan(1, nt);
equilibriumResidualMm = nan(1, nt);
equilibriumIterations = nan(1, nt);
equilibriumConverged = false(1, nt);
solverExitFlag = nan(1, nt);
solverIterations = nan(1, nt);
optimizationStarts = nan(1, nt);
parameters = nan(4, nt);
predictedShape = nan(3, ns, nt);
predictedRotation = nan(3, 3, ns, nt);
loadPoint = nan(3, nt);
nodalForces = nan(3, ns, nt);
previousTheta = [];

for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    fit = fitGaussianToSparsePositions(tube, ...
        measurements.pSparse(:, :, it), measurements.fbgIdx, cfg, previousTheta);
    forceResultant(:, it) = fit.forceResultant;
    centerMm(it) = fit.centerMm;
    sigmaMm(it) = fit.sigmaMm;
    shapeRmseMm(it) = fit.shapeRmseMm;
    equilibriumResidualMm(it) = fit.equilibriumResidualMm;
    equilibriumIterations(it) = fit.equilibriumIterations;
    equilibriumConverged(it) = fit.equilibriumConverged;
    solverExitFlag(it) = fit.solverExitFlag;
    solverIterations(it) = fit.solverIterations;
    optimizationStarts(it) = fit.optimizationStarts;
    parameters(:, it) = fit.theta;
    predictedShape(:, :, it) = fit.predictedShape;
    predictedRotation(:, :, :, it) = fit.predictedRotation;
    loadPoint(:, it) = interp1(tube.s(:), fit.predictedShape', fit.centerMm, ...
        'linear', 'extrap')';
    nodalForces(:, :, it) = fit.nodalForces;
    cost(it) = fit.cost;
    previousTheta = fit.theta;
    if cfg.aloi.showProgress
        fprintf(['  Aloi frame %3d/%3d: s=%.2f mm, sigma=%.2f mm, ', ...
            'resultant=[%.3f %.3f %.3f] N, position RMSE=%.4f mm, ', ...
            'equilibrium residual=%.3g mm (%d iter)\n'], ...
            it, nt, fit.centerMm, fit.sigmaMm, fit.forceResultant, ...
            fit.shapeRmseMm, fit.equilibriumResidualMm, fit.equilibriumIterations);
    end
end

aloi = struct;
aloi.forceResultant = forceResultant;
aloi.totalForceResultant = forceResultant;
aloi.centerMm = centerMm;
aloi.sigmaMm = sigmaMm;
aloi.tipSigmaMm = nan(1, nt);
aloi.componentResultants = reshape(forceResultant, 3, 1, nt);
aloi.cost = cost;
aloi.shapeRmseMm = shapeRmseMm;
aloi.equilibriumResidualMm = equilibriumResidualMm;
aloi.equilibriumIterations = equilibriumIterations;
aloi.equilibriumConverged = equilibriumConverged;
aloi.solverExitFlag = solverExitFlag;
aloi.solverIterations = solverIterations;
aloi.optimizationStarts = optimizationStarts;
aloi.parameters = parameters;
aloi.p = predictedShape;
aloi.R = predictedRotation;
aloi.loadPoint = loadPoint;
aloi.nodalForces = nodalForces;
aloi.finalNodalForces = nodalForces(:, :, end);
aloi.methodDescription = ['Single-Gaussian local-transverse load fitted to sparse positions ', ...
    'with the weighted nonlinear least-squares objective used by Aloi et al. ', ...
    'The loaded material frame and geometric Jacobian are updated to nonlinear ', ...
    'equilibrium for every prediction; no plane or friction measurements are used.'];
end


function fit = fitGaussianToSparsePositions(tube, targetPositions, measurementIdx, cfg, previousTheta)
s = tube.s(:);
[~, referenceR, referenceP] = solveShape(tube.T_base, tube.uhat, tube.s);
referenceJacobian = computeJacobian(referenceR, referenceP);
model = struct;
model.s = s;
model.tube = tube;
model.referenceP = referenceP;
model.referenceR = referenceR;
model.referenceJacobian = referenceJacobian;
model.invK = 1 ./ getTubeK(tube);
model.integrationWeightsMm = trapezoidalIntegrationWeights(s);
model.measurementIdx = measurementIdx(:)';
model.targetPositions = targetPositions;
model.positionStdMm = cfg.aloi.positionStdMm;
model.maxEquilibriumIterations = cfg.aloi.maxEquilibriumIterations;
model.equilibriumToleranceMm = cfg.aloi.equilibriumToleranceMm;
model.equilibriumRelaxation = cfg.aloi.equilibriumRelaxation;

amplitudeBound = cfg.aloi.amplitudeBoundN;
sigmaMin = min(cfg.aloi.sigmaCandidatesMm);
sigmaMax = max(cfg.aloi.sigmaCandidatesMm);
lb = [-amplitudeBound; -amplitudeBound; s(1); sigmaMin];
ub = [ amplitudeBound;  amplitudeBound; s(end); sigmaMax];
centerCandidates = linspace(s(1), s(end), cfg.aloi.numCenterCandidates);
targetDelta = targetPositions - referenceP(:, model.measurementIdx);
targetDelta = targetDelta(:);
theta0 = [0; 0; centerCandidates(1); cfg.aloi.sigmaCandidatesMm(1)];
initialTheta = zeros(4, numel(centerCandidates) * numel(cfg.aloi.sigmaCandidatesMm) + 1);
initialCost = inf(1, size(initialTheta, 2));
candidateIndex = 0;

for sigma = cfg.aloi.sigmaCandidatesMm
    for center = centerCandidates
        thetaBasis1 = [1; 0; center; sigma];
        thetaBasis2 = [0; 1; center; sigma];
        pBasis1 = predictGaussianShapeLinearized(thetaBasis1, model);
        pBasis2 = predictGaussianShapeLinearized(thetaBasis2, model);
        response1 = pBasis1(:, model.measurementIdx) - referenceP(:, model.measurementIdx);
        response2 = pBasis2(:, model.measurementIdx) - referenceP(:, model.measurementIdx);
        A = [response1(:), response2(:)];
        amplitude = pinv(A) * targetDelta;
        amplitude = min(max(amplitude, -amplitudeBound), amplitudeBound);
        candidate = [amplitude; center; sigma];
        linearizedResidual = (A * amplitude - targetDelta) / model.positionStdMm;
        currentCost = linearizedResidual' * linearizedResidual;
        candidateIndex = candidateIndex + 1;
        initialTheta(:, candidateIndex) = candidate;
        initialCost(candidateIndex) = currentCost;
    end
end

if nargin >= 5 && ~isempty(previousTheta)
    previousTheta = min(max(previousTheta(:), lb), ub);
    previousCost = sum(positionResidual(previousTheta, model) .^ 2);
    candidateIndex = candidateIndex + 1;
    initialTheta(:, candidateIndex) = previousTheta;
    initialCost(candidateIndex) = previousCost;
end

residualFunction = @(theta) positionResidual(theta, model);
initialTheta = initialTheta(:, 1:candidateIndex);
initialCost = initialCost(1:candidateIndex);
[~, startOrder] = sort(initialCost, 'ascend');
numStarts = min(cfg.aloi.numOptimizationStarts, numel(startOrder));
bestOptimizationCost = inf;
theta = theta0;
solverExitFlag = nan;
solverIterations = nan;

if exist('lsqnonlin', 'file') == 2
    options = optimoptions('lsqnonlin', 'Display', 'off', ...
        'MaxIterations', cfg.aloi.maxIterations, ...
        'MaxFunctionEvaluations', cfg.aloi.maxFunctionEvaluations, ...
        'FunctionTolerance', 1e-9, 'StepTolerance', 1e-8);
    for iStart = 1:numStarts
        thetaStart = initialTheta(:, startOrder(iStart));
        [candidateTheta, candidateCost, ~, candidateExitFlag, candidateOutput] = ...
            lsqnonlin(residualFunction, thetaStart, lb, ub, options);
        if candidateCost < bestOptimizationCost
            bestOptimizationCost = candidateCost;
            theta = candidateTheta;
            solverExitFlag = candidateExitFlag;
            solverIterations = candidateOutput.iterations;
        end
    end
else
    objective = @(theta) sum(residualFunction(theta) .^ 2);
    options = optimoptions('fmincon', 'Display', 'off', ...
        'MaxIterations', cfg.aloi.maxIterations, ...
        'MaxFunctionEvaluations', cfg.aloi.maxFunctionEvaluations);
    for iStart = 1:numStarts
        thetaStart = initialTheta(:, startOrder(iStart));
        [candidateTheta, candidateCost, candidateExitFlag, candidateOutput] = ...
            fmincon(objective, thetaStart, [], [], [], [], lb, ub, [], options);
        if candidateCost < bestOptimizationCost
            bestOptimizationCost = candidateCost;
            theta = candidateTheta;
            solverExitFlag = candidateExitFlag;
            solverIterations = candidateOutput.iterations;
        end
    end
end

[predictedP, fittedNodalForces, predictedR, equilibrium] = ...
    predictGaussianShape(theta, model);
positionError = predictedP(:, model.measurementIdx) - targetPositions;
fit = struct;
fit.theta = theta(:);
fit.centerMm = theta(3);
fit.sigmaMm = theta(4);
fit.forceResultant = sum(fittedNodalForces, 2);
fit.nodalForces = fittedNodalForces;
fit.predictedShape = predictedP;
fit.predictedRotation = predictedR;
fit.shapeRmseMm = sqrt(mean(sum(positionError .^ 2, 1)));
fit.cost = sum((positionError(:) / model.positionStdMm) .^ 2);
fit.equilibriumResidualMm = equilibrium.residualMm;
fit.equilibriumIterations = equilibrium.iterations;
fit.equilibriumConverged = equilibrium.converged;
fit.solverExitFlag = solverExitFlag;
fit.solverIterations = solverIterations;
fit.optimizationStarts = numStarts;
end


function residual = positionResidual(theta, model)
p = predictGaussianShape(theta, model);
residual = p(:, model.measurementIdx) - model.targetPositions;
residual = residual(:) / model.positionStdMm;
end


function [p, fittedNodalForces, R, equilibrium] = predictGaussianShape(theta, model)
density = gaussianDensityOnArc(model.s, theta(3), theta(4));
localLoad = [theta(1) * density'; theta(2) * density'; ...
    zeros(1, numel(model.s))];
R = model.referenceR;
p = model.referenceP;
u = model.tube.uhat;

for iteration = 1:model.maxEquilibriumIterations
    fittedNodalForces = localLoadToNodalForces( ...
        localLoad, R, model.integrationWeightsMm);
    J = computeJacobian(R, p);
    moment = J' * fittedNodalForces(:);
    equilibriumU = reshape(model.invK .* moment, 3, []) + model.tube.uhat;
    u = (1 - model.equilibriumRelaxation) * u + ...
        model.equilibriumRelaxation * equilibriumU;
    [~, nextR, nextP] = solveShape(model.tube.T_base, u, model.tube.s);
    residualMm = shapeRmsDifference(nextP, p);
    R = nextR;
    p = nextP;
    if residualMm <= ...
            model.equilibriumRelaxation * model.equilibriumToleranceMm
        break;
    end
end

% The final fixed-point defect checks nonlinear equilibrium independently
% of the relaxed update size used inside the iteration.
fittedNodalForces = localLoadToNodalForces( ...
    localLoad, R, model.integrationWeightsMm);
J = computeJacobian(R, p);
moment = J' * fittedNodalForces(:);
equilibriumU = reshape(model.invK .* moment, 3, []) + model.tube.uhat;
[~, ~, equilibriumP] = solveShape(model.tube.T_base, equilibriumU, model.tube.s);
fixedPointResidualMm = shapeRmsDifference(equilibriumP, p);

equilibrium = struct;
equilibrium.residualMm = fixedPointResidualMm;
equilibrium.iterations = iteration;
equilibrium.converged = fixedPointResidualMm <= model.equilibriumToleranceMm;
end


function p = predictGaussianShapeLinearized(theta, model)
density = gaussianDensityOnArc(model.s, theta(3), theta(4));
localLoad = [theta(1) * density'; theta(2) * density'; ...
    zeros(1, numel(model.s))];
nodalForces = localLoadToNodalForces( ...
    localLoad, model.referenceR, model.integrationWeightsMm);
moment = model.referenceJacobian' * nodalForces(:);
u = reshape(model.invK .* moment, 3, []) + model.tube.uhat;
[~, ~, p] = solveShape(model.tube.T_base, u, model.tube.s);
end


function nodalForces = localLoadToNodalForces(localLoad, R, weightsMm)
ns = size(localLoad, 2);
distributedLoad = zeros(3, ns);
for i = 1:ns
    distributedLoad(:, i) = R(:, :, i) * localLoad(:, i);
end
nodalForces = distributedLoad .* reshape(weightsMm, 1, []);
end


function value = shapeRmsDifference(p1, p2)
value = sqrt(mean(sum((p1 - p2) .^ 2, 1)));
end


function density = gaussianDensityOnArc(s, center, sigma)
phi = exp(-0.5 * ((s - center) ./ sigma).^2);
area = trapz(s, phi);
if area <= eps
    density = zeros(size(s));
else
    density = phi / area;
end
end


function weights = trapezoidalIntegrationWeights(s)
weights = zeros(size(s));
weights(1) = 0.5 * (s(2) - s(1));
weights(end) = 0.5 * (s(end) - s(end - 1));
weights(2:end - 1) = 0.5 * (s(3:end) - s(1:end - 2));
end
