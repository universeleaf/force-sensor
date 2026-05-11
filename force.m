function force()
clearvars;
clc;
close all;

cfg = defaultForceConfig();
results = runForceSensingBenchmark(cfg);
printForceSummary(results, cfg);
plotForceSensingReport(results, cfg);
end


function cfg = defaultForceConfig()
cfg.dim = 2;
cfg.seed = 7;
cfg.outputDir = fullfile(pwd, 'force_outputs');
cfg.makeAnimation = true;
cfg.animationFile = 'continuum_robot_motion.gif';
cfg.animationKeyframeFile = 'continuum_robot_motion_keyframes.png';
cfg.animationDelay = 0.12;

cfg.L = 0.30;
cfg.EI = 0.03;
cfg.nGrid = 101;
cfg.nMeas = 21;
cfg.baseCurvature = 3.2;
cfg.tipCase.forceTrue = 0.18;
cfg.tipCase.nSteps = 40;
cfg.tipCase.forceRampStart = 12;
cfg.tipCase.forceRampStop = 28;
cfg.tipCase.poseNoiseStd = [2.0e-4; 2.0e-4; 5.0e-3];
cfg.tipCase.actuationNoiseStd = 0.03;
cfg.tipCase.forceProcessStd = 0.006;
cfg.tipCase.forceInitStd = 0.10;
cfg.tipCase.actuationScaleTrue = 1.0;

cfg.shapeNoiseStd = 6.0e-4;
cfg.curvatureNoiseStd = 0.05;
cfg.measurementSigmaScale = 1.0;

cfg.aloi.nGauss = 1;
cfg.aloi.theta0 = [0.10; 0.18; 0.045];
cfg.aloi.lb = [0.01; 0.04; 0.015];
cfg.aloi.ub = [0.40; 0.28; 0.10];
cfg.aloi.maxIter = 50;
cfg.aloi.lambda0 = 1e-3;
cfg.aloi.fdStep = [1e-3; 2e-3; 1e-3];
cfg.aloi.priorWeight = diag([1e-5, 1e-5, 1e-5]);
cfg.aloi.forceSeeds = [0.06, 0.10, 0.14, 0.20];
cfg.aloi.muSeeds = [0.08, 0.14, 0.18, 0.22, 0.26];
cfg.aloi.sigmaSeeds = [0.025, 0.04, 0.065];

cfg.ferguson.lambdaDynamics = 0.2;
cfg.ferguson.lambdaBoundary = 8.0;
cfg.ferguson.lambdaLoadSmooth = 3e-4;
cfg.ferguson.maxIter = 10;
cfg.ferguson.stepDamping = 0.7;
cfg.ferguson.priorLoadStd = 1.2;
cfg.ferguson.posteriorBandSigma = 2.0;

cfg.acceptance.ruckerRelErrMax = 0.20;
cfg.acceptance.aloiCentroidErrMax = 0.08 * cfg.L;
cfg.acceptance.fergusonCentroidErrMax = 0.10 * cfg.L;
end


function results = runForceSensingBenchmark(cfg)
if cfg.dim ~= 2
    error('This prototype currently supports cfg.dim = 2 only. 3D hooks are reserved for a later extension.');
end

rng(cfg.seed, 'twister');
ensureOutputDir(cfg.outputDir);

model = beamInfluenceMatrices(cfg);
tipCase = simulateTipCase(cfg, model);
distCase = simulateDistributedCase(cfg, model);

results.config = cfg;
results.model = model;
results.tipCase = tipCase;
results.distCase = distCase;
results.rucker = estimateRuckerEKF(cfg, model, tipCase);
results.aloi = estimateAloiGaussianLoad(cfg, model, distCase);
results.ferguson = estimateFergusonBatchLoad(cfg, model, distCase);
results.summary = buildSummary(results, cfg, model);
end


function model = beamInfluenceMatrices(cfg)
s = linspace(0, cfg.L, cfg.nGrid).';
ds = s(2) - s(1);
sMeas = linspace(0, cfg.L, cfg.nMeas).';
measIdx = round(linspace(1, cfg.nGrid, cfg.nMeas));
measIdx = unique(min(max(measIdx, 1), cfg.nGrid));
if numel(measIdx) ~= cfg.nMeas
    measIdx = round(interp1(1:numel(measIdx), measIdx, linspace(1, numel(measIdx), cfg.nMeas), 'nearest', 'extrap'));
end
sMeas = s(measIdx);

Phi = zeros(cfg.nGrid, cfg.nGrid);
for i = 1:cfg.nGrid
    si = s(i);
    for j = 1:cfg.nGrid
        xi = s(j);
        if si <= xi
            kernel = si^2 * (3 * xi - si) / (6 * cfg.EI);
        else
            kernel = xi^2 * (3 * si - xi) / (6 * cfg.EI);
        end
        Phi(i, j) = kernel * ds;
    end
end

D1 = zeros(cfg.nGrid, cfg.nGrid);
for i = 2:(cfg.nGrid - 1)
    D1(i, i-1) = -0.5 / ds;
    D1(i, i+1) = 0.5 / ds;
end
D1(1, 1:2) = [-1, 1] / ds;
D1(end, end-1:end) = [-1, 1] / ds;

D2 = zeros(cfg.nGrid, cfg.nGrid);
for i = 2:(cfg.nGrid - 1)
    D2(i, i-1:i+1) = [1, -2, 1] / ds^2;
end
D2(1, 1:3) = [1, -2, 1] / ds^2;
D2(end, end-2:end) = [1, -2, 1] / ds^2;

Mpos = zeros(cfg.nMeas, cfg.nGrid);
for k = 1:cfg.nMeas
    Mpos(k, measIdx(k)) = 1;
end

actShape = 0.5 * cfg.baseCurvature * s.^2;
actTheta = cfg.baseCurvature * s;
actKappa = cfg.baseCurvature * ones(cfg.nGrid, 1);
tipSelector = zeros(3, cfg.nGrid);
tipSelector(1, :) = 0;
tipSelector(1, end) = 1;
tipSelector(2, :) = 0;
tipSelector(2, end) = 1;
tipSelector(3, :) = 0;

model.s = s;
model.ds = ds;
model.sMeas = sMeas;
model.measIdx = measIdx;
model.Phi = Phi;
model.D1 = D1;
model.D2 = D2;
model.Mpos = Mpos;
model.actShape = actShape;
model.actTheta = actTheta;
model.actKappa = actKappa;
model.tipSelector = tipSelector;
end


function tipCase = simulateTipCase(cfg, model)
n = cfg.tipCase.nSteps;
forceTrue = zeros(n, 1);
rampVals = linspace(0, cfg.tipCase.forceTrue, cfg.tipCase.forceRampStop - cfg.tipCase.forceRampStart + 1);
forceTrue(cfg.tipCase.forceRampStart:cfg.tipCase.forceRampStop) = rampVals;
forceTrue((cfg.tipCase.forceRampStop + 1):end) = cfg.tipCase.forceTrue;

actScaleTrue = cfg.tipCase.actuationScaleTrue * ones(n, 1);
poseTrue = zeros(3, n);
poseMeas = zeros(3, n);
actMeas = zeros(n, 1);
shapeTrue = zeros(cfg.nGrid, n);
thetaTrue = zeros(cfg.nGrid, n);

for k = 1:n
    [yTipLoad, thetaTipLoad] = tipForceInfluence(model, cfg, forceTrue(k));
    y = actScaleTrue(k) * model.actShape + yTipLoad;
    theta = actScaleTrue(k) * model.actTheta + thetaTipLoad;
    shapeTrue(:, k) = y;
    thetaTrue(:, k) = theta;
    poseTrue(:, k) = [model.s(end); y(end); theta(end)];
    poseMeas(:, k) = poseTrue(:, k) + [0; cfg.tipCase.poseNoiseStd(2) * randn; cfg.tipCase.poseNoiseStd(3) * randn];
    actMeas(k) = actScaleTrue(k) + cfg.tipCase.actuationNoiseStd * randn;
end

tipCase.forceTrue = forceTrue;
tipCase.actuationScaleTrue = actScaleTrue;
tipCase.poseTrue = poseTrue;
tipCase.poseMeas = poseMeas;
tipCase.actuationMeas = actMeas;
tipCase.shapeTrue = shapeTrue;
tipCase.thetaTrue = thetaTrue;
end


function distCase = simulateDistributedCase(cfg, model)
paramsTrue = [0.12; 0.17; 0.04];
qTrue = gaussianLoad(model.s, paramsTrue(1), paramsTrue(2), paramsTrue(3));
yLoad = model.Phi * qTrue;
yTrue = model.actShape + yLoad;
thetaTrue = model.actTheta + model.D1 * yLoad;
kappaTrue = model.actKappa + model.D2 * yLoad;

yMeas = yTrue(model.measIdx) + cfg.shapeNoiseStd * randn(cfg.nMeas, 1);
kappaMeas = kappaTrue(model.measIdx) + cfg.curvatureNoiseStd * randn(cfg.nMeas, 1);
yNoisyFull = yTrue + cfg.shapeNoiseStd * randn(cfg.nGrid, 1);
kappaNoisyFull = kappaTrue + cfg.curvatureNoiseStd * randn(cfg.nGrid, 1);

distCase.paramsTrue = paramsTrue;
distCase.qTrue = qTrue;
distCase.yTrue = yTrue;
distCase.thetaTrue = thetaTrue;
distCase.kappaTrue = kappaTrue;
distCase.yMeas = yMeas;
distCase.kappaMeas = kappaMeas;
distCase.yNoisyFull = yNoisyFull;
distCase.kappaNoisyFull = kappaNoisyFull;
distCase.rawMeasurementRmse = sqrt(mean((yMeas - yTrue(model.measIdx)).^2));
end


function rucker = estimateRuckerEKF(cfg, model, tipCase)
n = cfg.tipCase.nSteps;
x = [0; model.actShape(end); model.actTheta(end); 1.0; 0.0];
P = diag([1e-8, 1e-5, 1e-4, 0.04, cfg.tipCase.forceInitStd^2]);
Q = diag([1e-10, 1e-8, 1e-6, 2e-4, cfg.tipCase.forceProcessStd^2]);
R = diag([1e-10, cfg.tipCase.poseNoiseStd(2)^2, cfg.tipCase.poseNoiseStd(3)^2, cfg.tipCase.actuationNoiseStd^2]);

xHist = zeros(numel(x), n);
PHist = zeros(numel(x), numel(x), n);
innovHist = zeros(4, n);

for k = 1:n
    xPred = x;
    PPred = P + Q;

    zPred = measurementFromRuckerState(xPred, cfg, model);
    H = finiteDifferenceJacobian(@(xx) measurementFromRuckerState(xx, cfg, model), xPred, [1e-7; 1e-6; 1e-6; 1e-4; 1e-4]);
    z = [tipCase.poseMeas(:, k); tipCase.actuationMeas(k)];
    innov = z - zPred;
    S = H * PPred * H.' + R;
    K = (PPred * H.') / S;

    x = xPred + K * innov;
    P = (eye(size(P)) - K * H) * PPred;
    x(1) = model.s(end);
    x(4) = max(0.7, min(1.3, x(4)));

    xHist(:, k) = x;
    PHist(:, :, k) = P;
    innovHist(:, k) = innov;
end

forceEst = xHist(5, :).';
sigmaForce = squeeze(sqrt(PHist(5, 5, :)));
relErr = abs(forceEst(end) - tipCase.forceTrue(end)) / max(abs(tipCase.forceTrue(end)), 1e-8);

rucker.xHist = xHist;
rucker.PHist = PHist;
rucker.innovHist = innovHist;
rucker.forceEst = forceEst;
rucker.forceSigma = sigmaForce;
rucker.forceTrue = tipCase.forceTrue;
rucker.posePredFinal = xHist(1:3, end);
rucker.finalAbsErr = abs(forceEst(end) - tipCase.forceTrue(end));
rucker.finalRelErr = relErr;
rucker.shapeEst = reconstructRuckerShapes(xHist, cfg, model);
end


function z = measurementFromRuckerState(x, cfg, model)
scale = x(4);
force = x(5);
[yLoad, thetaLoad] = tipForceInfluence(model, cfg, force);
yTip = scale * model.actShape(end) + yLoad(end);
thetaTip = scale * model.actTheta(end) + thetaLoad(end);
z = [model.s(end); yTip; thetaTip; scale];
end


function aloi = estimateAloiGaussianLoad(cfg, model, distCase)
Wpos = eye(cfg.nMeas) / (cfg.shapeNoiseStd^2);
best.theta = cfg.aloi.theta0;
best.cost = inf;
best.iter = 0;

for f0 = cfg.aloi.forceSeeds
    for mu0 = cfg.aloi.muSeeds
        for sig0 = cfg.aloi.sigmaSeeds
            theta0 = [f0; mu0; sig0];
            [thetaTry, costTry, iterTry] = runAloiLocalFit(theta0, cfg, model, distCase, Wpos);
            if costTry < best.cost
                best.theta = thetaTry;
                best.cost = costTry;
                best.iter = iterTry;
            end
        end
    end
end

theta = best.theta;
qEst = gaussianLoad(model.s, theta(1), theta(2), theta(3));
yEst = model.actShape + model.Phi * qEst;

shapeRmse = sqrt(mean((yEst(model.measIdx) - distCase.yTrue(model.measIdx)).^2));
centroidTrue = loadCentroid(model.s, distCase.qTrue);
centroidEst = loadCentroid(model.s, qEst);

aloi.paramsEst = theta;
aloi.qEst = qEst;
aloi.yEst = yEst;
aloi.shapeRmse = shapeRmse;
aloi.centroidTrue = centroidTrue;
aloi.centroidEst = centroidEst;
aloi.centroidErr = abs(centroidEst - centroidTrue);
aloi.netForceTrue = trapz(model.s, distCase.qTrue);
aloi.netForceEst = trapz(model.s, qEst);
aloi.netForceAbsErr = abs(aloi.netForceEst - aloi.netForceTrue);
aloi.cost = best.cost;
aloi.iterations = best.iter;
end


function [theta, bestCost, iter] = runAloiLocalFit(theta0, cfg, model, distCase, Wpos)
theta = projectBounds(theta0, cfg.aloi.lb, cfg.aloi.ub);
lambda = cfg.aloi.lambda0;
bestCost = inf;

for iter = 1:cfg.aloi.maxIter
    yPred = predictShapeFromGaussian(theta, model);
    r = distCase.yMeas - yPred(model.measIdx);
    J = finiteDifferenceJacobian(@(tt) predictShapeFromGaussian(tt, model, model.measIdx), theta, cfg.aloi.fdStep);
    prior = cfg.aloi.priorWeight * (theta - cfg.aloi.theta0);
    costNow = r.' * Wpos * r + prior.' * prior;
    if costNow < bestCost
        bestCost = costNow;
    end

    A = J.' * Wpos * J + lambda * eye(numel(theta)) + cfg.aloi.priorWeight;
    b = J.' * Wpos * r - cfg.aloi.priorWeight * (theta - cfg.aloi.theta0);
    step = A \ b;

    accepted = false;
    for alpha = [1.0, 0.5, 0.25, 0.1, 0.05]
        thetaCand = projectBounds(theta + alpha * step, cfg.aloi.lb, cfg.aloi.ub);
        yCand = predictShapeFromGaussian(thetaCand, model);
        rCand = distCase.yMeas - yCand(model.measIdx);
        priorCand = cfg.aloi.priorWeight * (thetaCand - cfg.aloi.theta0);
        costCand = rCand.' * Wpos * rCand + priorCand.' * priorCand;
        if costCand < costNow
            theta = thetaCand;
            bestCost = costCand;
            lambda = max(lambda * 0.6, 1e-7);
            accepted = true;
            break;
        end
    end

    if ~accepted
        lambda = min(lambda * 8.0, 1e6);
    end

    if norm(step) < 1e-7
        break;
    end
end
end


function y = predictShapeFromGaussian(theta, model, idx)
if nargin < 3
    idx = [];
end
q = gaussianLoad(model.s, theta(1), theta(2), theta(3));
y = model.actShape + model.Phi * q;
if ~isempty(idx)
    y = y(idx);
end
end


function ferguson = estimateFergusonBatchLoad(cfg, model, distCase)
n = cfg.nGrid;
N = 5 * n;

y0 = smoothFromMeasurements(model.s, model.sMeas, distCase.yMeas, model.actShape);
theta0 = gradient(y0, model.ds);
kappa0 = smoothFromMeasurements(model.s, model.sMeas, distCase.kappaMeas, model.actKappa);
kappaDot0 = gradient(kappa0, model.ds);
q0 = cfg.EI * gradient(gradient(kappa0 - model.actKappa, model.ds), model.ds);

x = packBatchState(y0, theta0, kappa0, kappaDot0, q0);

WposSqrt = eye(cfg.nMeas) / cfg.shapeNoiseStd;
WcurvSqrt = eye(cfg.nMeas) / cfg.curvatureNoiseStd;

for iter = 1:cfg.ferguson.maxIter
    [res, J] = batchResidualAndJacobian(x, cfg, model, distCase, WposSqrt, WcurvSqrt);
    H = J.' * J + 1e-8 * speye(N);
    g = J.' * res;
    dx = -H \ g;
    x = x + cfg.ferguson.stepDamping * dx;
    if norm(dx) / sqrt(numel(dx)) < 1e-6
        break;
    end
end

[res, J] = batchResidualAndJacobian(x, cfg, model, distCase, WposSqrt, WcurvSqrt);
covX = inv(full(J.' * J + 1e-8 * speye(N)));
[yEst, thetaEst, kappaEst, kappaDotEst, qEst] = unpackBatchState(x, n);

idxLoad = (4 * n + 1):(5 * n);
covQ = covX(idxLoad, idxLoad);
qStd = sqrt(max(diag(covQ), 0));

shapeRmse = sqrt(mean((yEst(model.measIdx) - distCase.yTrue(model.measIdx)).^2));
centroidTrue = loadCentroid(model.s, distCase.qTrue);
centroidEst = loadCentroid(model.s, qEst);

ferguson.yEst = yEst;
ferguson.thetaEst = thetaEst;
ferguson.kappaEst = kappaEst;
ferguson.kappaDotEst = kappaDotEst;
ferguson.qEst = qEst;
ferguson.qStd = qStd;
ferguson.posteriorBand = cfg.ferguson.posteriorBandSigma * qStd;
ferguson.shapeRmse = shapeRmse;
ferguson.centroidTrue = centroidTrue;
ferguson.centroidEst = centroidEst;
ferguson.centroidErr = abs(centroidEst - centroidTrue);
ferguson.rawMeasurementRmse = distCase.rawMeasurementRmse;
ferguson.covX = covX;
ferguson.finalResidualNorm = norm(res) / sqrt(numel(res));
end


function [res, J] = batchResidualAndJacobian(x, cfg, model, distCase, WposSqrt, WcurvSqrt)
res = batchResidualOnly(x, cfg, model, distCase, WposSqrt, WcurvSqrt);
n = numel(x);
m = numel(res);
J = spalloc(m, n, 15 * n);
fd = 1e-6;
for i = 1:n
    xp = x;
    xp(i) = xp(i) + fd;
    rp = batchResidualOnly(xp, cfg, model, distCase, WposSqrt, WcurvSqrt);
    J(:, i) = sparse((rp - res) / fd);
end
end


function res = batchResidualOnly(x, cfg, model, distCase, WposSqrt, WcurvSqrt)
n = cfg.nGrid;
[y, theta, kappa, kappaDot, q] = unpackBatchState(x, n);
ds = model.ds;

rDyn1 = sqrt(cfg.ferguson.lambdaDynamics) * (diff(y) / ds - theta(1:end-1));
rDyn2 = sqrt(cfg.ferguson.lambdaDynamics) * (diff(theta) / ds - kappa(1:end-1));
rDyn3 = sqrt(cfg.ferguson.lambdaDynamics) * (diff(kappa) / ds - kappaDot(1:end-1));
rDyn4 = sqrt(cfg.ferguson.lambdaDynamics) * (cfg.EI * diff(kappaDot) / ds - q(1:end-1));

rPos = WposSqrt * (y(model.measIdx) - distCase.yMeas);
rCurv = WcurvSqrt * (kappa(model.measIdx) - distCase.kappaMeas);

rBc = sqrt(cfg.ferguson.lambdaBoundary) * [y(1); theta(1); kappa(1) - cfg.baseCurvature];
rLoad = sqrt(cfg.ferguson.lambdaLoadSmooth) * diff(q);

res = [rDyn1; rDyn2; rDyn3; rDyn4; rPos; rCurv; rBc; rLoad];
end


function summary = buildSummary(results, cfg, model)
ruckerPass = results.rucker.finalRelErr < cfg.acceptance.ruckerRelErrMax;
aloiPass = results.aloi.centroidErr < cfg.acceptance.aloiCentroidErrMax;
fergPass = results.ferguson.centroidErr < cfg.acceptance.fergusonCentroidErrMax && ...
    results.ferguson.shapeRmse < results.ferguson.rawMeasurementRmse;

names = {'Rucker tip EKF'; 'Aloi Gaussian load'; 'Ferguson batch'};
metric1 = [results.rucker.finalRelErr; results.aloi.centroidErr / cfg.L; results.ferguson.centroidErr / cfg.L];
metric2 = [results.rucker.finalAbsErr; results.aloi.shapeRmse; results.ferguson.shapeRmse];
passVec = [ruckerPass; aloiPass; fergPass];

summary.names = names;
summary.metric1 = metric1;
summary.metric2 = metric2;
summary.passVec = passVec;
summary.modelLength = model.s(end);
end


function plotForceSensingReport(results, cfg)
ensureOutputDir(cfg.outputDir);
model = results.model;
tipCase = results.tipCase;
distCase = results.distCase;
rucker = results.rucker;
aloi = results.aloi;
ferguson = results.ferguson;

fig1 = figure('Name', 'Force Sensing Benchmark', 'Color', 'w', 'Position', [100 80 1400 900]);

subplot(2, 3, 1);
plot(model.s, distCase.yTrue * 1e3, 'k-', 'LineWidth', 2); hold on;
plot(model.sMeas, distCase.yMeas * 1e3, 'bo', 'MarkerFaceColor', [0.3 0.6 1.0]);
plot(model.s, distCase.yNoisyFull * 1e3, 'Color', [0.7 0.7 0.9], 'LineWidth', 1);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Deflection y [mm]');
title('Truth and noisy shape sensing');
legend({'True shape', 'Measured points', 'Noisy full-field sample'}, 'Location', 'best');

subplot(2, 3, 2);
stairs(1:numel(rucker.forceTrue), rucker.forceTrue, 'k-', 'LineWidth', 1.8); hold on;
plot(1:numel(rucker.forceEst), rucker.forceEst, 'r-', 'LineWidth', 2);
plot(1:numel(rucker.forceEst), rucker.forceEst + 2 * rucker.forceSigma, 'r--', 'LineWidth', 1.0);
plot(1:numel(rucker.forceEst), rucker.forceEst - 2 * rucker.forceSigma, 'r--', 'LineWidth', 1.0);
grid on; box on;
xlabel('Time step');
ylabel('Lateral tip force [N]');
title(sprintf('Rucker 2011 EKF, final rel err = %.1f%%', 100 * rucker.finalRelErr));
legend({'True force', 'Estimated force', '+2\sigma', '-2\sigma'}, 'Location', 'best');

subplot(2, 3, 3);
plot(model.s, distCase.qTrue, 'k-', 'LineWidth', 2); hold on;
plot(model.s, aloi.qEst, 'r-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Distributed load q [N/m]');
title(sprintf('Aloi 2022 Gaussian load, centroid err = %.1f mm', 1e3 * aloi.centroidErr));
legend({'True load', 'Estimated load'}, 'Location', 'best');

subplot(2, 3, 4);
plot(model.s, distCase.yTrue * 1e3, 'k-', 'LineWidth', 2); hold on;
plot(model.s, ferguson.yEst * 1e3, 'm-', 'LineWidth', 2);
plot(model.sMeas, distCase.yMeas * 1e3, 'bo', 'MarkerFaceColor', [0.3 0.6 1.0]);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Deflection y [mm]');
title(sprintf('Ferguson 2024 posterior shape, RMSE = %.3f mm', 1e3 * ferguson.shapeRmse));
legend({'True shape', 'Posterior mean', 'Measurements'}, 'Location', 'best');

subplot(2, 3, 5);
fillX = [model.s; flipud(model.s)];
fillY = [ferguson.qEst + ferguson.posteriorBand; flipud(ferguson.qEst - ferguson.posteriorBand)];
patch(fillX, fillY, [0.92 0.75 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.6); hold on;
plot(model.s, distCase.qTrue, 'k-', 'LineWidth', 2);
plot(model.s, ferguson.qEst, 'm-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Distributed load q [N/m]');
title(sprintf('Ferguson load posterior, centroid err = %.1f mm', 1e3 * ferguson.centroidErr));
legend({'Posterior 2\sigma band', 'True load', 'Posterior mean'}, 'Location', 'best');

subplot(2, 3, 6);
barData = [100 * rucker.finalRelErr, 1e3 * aloi.centroidErr, 1e3 * ferguson.centroidErr; ...
           1e3 * rucker.finalAbsErr, 1e3 * aloi.shapeRmse, 1e3 * ferguson.shapeRmse];
b = bar(barData.');
b(1).FaceColor = [0.85 0.35 0.35];
b(2).FaceColor = [0.35 0.55 0.85];
grid on; box on;
set(gca, 'XTickLabel', {'Rucker', 'Aloi', 'Ferguson'});
ylabel('Mixed benchmark metrics');
title('Error summary');
legend({'Primary metric', 'Secondary metric'}, 'Location', 'best');

sgtitle('2D comparable prototype for continuum-robot force sensing');
saveFigure(fig1, fullfile(cfg.outputDir, 'force_benchmark_overview.png'));

fig2 = figure('Name', 'Force Sensing Details', 'Color', 'w', 'Position', [120 100 1200 700]);
subplot(2, 2, 1);
plot(model.sMeas, distCase.kappaMeas, 'bo', 'MarkerFaceColor', [0.3 0.6 1.0]); hold on;
plot(model.s, distCase.kappaTrue, 'k-', 'LineWidth', 2);
plot(model.s, ferguson.kappaEst, 'm-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Curvature \kappa [1/m]');
title('FBG curvature sensing');
legend({'Measured curvature', 'True curvature', 'Posterior curvature'}, 'Location', 'best');

subplot(2, 2, 2);
plot(1:numel(rucker.forceEst), tipCase.poseMeas(2, :) * 1e3, 'b.-', 'LineWidth', 1.5); hold on;
plot(1:numel(rucker.forceEst), tipCase.poseTrue(2, :) * 1e3, 'k-', 'LineWidth', 1.5);
grid on; box on;
xlabel('Time step');
ylabel('Tip y [mm]');
title('Tip pose measurements used by EKF');
legend({'Measured tip y', 'True tip y'}, 'Location', 'best');

subplot(2, 2, 3);
plot(model.s, distCase.qTrue - aloi.qEst, 'r-', 'LineWidth', 1.8); hold on;
plot(model.s, distCase.qTrue - ferguson.qEst, 'm-', 'LineWidth', 1.8);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Load estimation error [N/m]');
title('Distributed-load estimation error');
legend({'Aloi error', 'Ferguson error'}, 'Location', 'best');

subplot(2, 2, 4);
plot(model.sMeas, distCase.yTrue(model.measIdx) * 1e3 - distCase.yMeas * 1e3, 'bo-', 'LineWidth', 1.2); hold on;
plot(model.s, distCase.yTrue * 1e3 - ferguson.yEst * 1e3, 'm-', 'LineWidth', 1.6);
plot(model.s, distCase.yTrue * 1e3 - aloi.yEst * 1e3, 'r-', 'LineWidth', 1.6);
grid on; box on;
xlabel('Arc length s [m]');
ylabel('Shape error [mm]');
title('Shape error comparison');
legend({'Measurement error at sensed points', 'Ferguson shape error', 'Aloi shape error'}, 'Location', 'best');

sgtitle('Estimator details');
saveFigure(fig2, fullfile(cfg.outputDir, 'force_benchmark_details.png'));

if cfg.makeAnimation
    animateContinuumRobotMotion(results, cfg);
end
end


function animateContinuumRobotMotion(results, cfg)
model = results.model;
tipCase = results.tipCase;
rucker = results.rucker;

gifPath = fullfile(cfg.outputDir, cfg.animationFile);
keyframePath = fullfile(cfg.outputDir, cfg.animationKeyframeFile);
if exist(gifPath, 'file')
    delete(gifPath);
end

nSteps = numel(rucker.forceTrue);
robotScale = 1e3;
forceScale = 90;
yAll = [tipCase.shapeTrue(:); rucker.shapeEst(:)];
yLim = robotScale * [min(yAll) - 0.015, max(yAll) + 0.02];
xLim = robotScale * [-0.01, cfg.L + 0.08];

fig = figure('Name', 'Continuum Robot Motion Animation', 'Color', 'w', ...
    'Position', [80 80 1150 650], 'Visible', 'on');

for k = 1:nSteps
    clf(fig);

    ax1 = subplot(2, 2, [1 3], 'Parent', fig);
    plot(ax1, robotScale * model.s, robotScale * tipCase.shapeTrue(:, k), ...
        'k-', 'LineWidth', 3); hold(ax1, 'on');
    plot(ax1, robotScale * model.s, robotScale * rucker.shapeEst(:, k), ...
        'r--', 'LineWidth', 2);
    plot(ax1, robotScale * model.s(end), robotScale * tipCase.poseMeas(2, k), ...
        'bo', 'MarkerFaceColor', [0.3 0.6 1.0], 'MarkerSize', 7);

    xTip = robotScale * model.s(end);
    yTipTrue = robotScale * tipCase.shapeTrue(end, k);
    trueDy = forceScale * rucker.forceTrue(k);
    estDy = forceScale * rucker.forceEst(k);
    quiver(ax1, xTip, yTipTrue, 0, trueDy, 0, ...
        'Color', [0 0 0], 'LineWidth', 2.2, 'MaxHeadSize', 0.8);
    quiver(ax1, xTip + 14, robotScale * rucker.shapeEst(end, k), 0, estDy, 0, ...
        'Color', [0.85 0.1 0.1], 'LineWidth', 2.2, 'MaxHeadSize', 0.8);

    plot(ax1, [0 0], yLim, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    grid(ax1, 'on'); box(ax1, 'on');
    axis(ax1, 'equal');
    xlim(ax1, xLim);
    ylim(ax1, yLim);
    xlabel(ax1, 'Backbone x [mm]');
    ylabel(ax1, 'Backbone y [mm]');
    title(ax1, sprintf('2D continuum robot motion, step %02d/%02d', k, nSteps));
    legend(ax1, {'True backbone', 'EKF-estimated backbone', 'Noisy tip measurement', ...
        'True tip force', 'Estimated tip force'}, 'Location', 'northwest');

    ax2 = subplot(2, 2, 2, 'Parent', fig);
    idx = 1:k;
    stairs(ax2, 1:nSteps, rucker.forceTrue, 'k-', 'LineWidth', 1.5); hold(ax2, 'on');
    plot(ax2, idx, rucker.forceEst(idx), 'r-', 'LineWidth', 2);
    plot(ax2, idx, rucker.forceEst(idx) + 2 * rucker.forceSigma(idx), 'r--', 'LineWidth', 1);
    plot(ax2, idx, rucker.forceEst(idx) - 2 * rucker.forceSigma(idx), 'r--', 'LineWidth', 1);
    xline(ax2, k, 'Color', [0.2 0.2 0.2], 'LineStyle', ':');
    grid(ax2, 'on'); box(ax2, 'on');
    xlim(ax2, [1 nSteps]);
    ylim(ax2, [-0.05, max(rucker.forceTrue) * 1.45]);
    xlabel(ax2, 'Time step');
    ylabel(ax2, 'Tip force [N]');
    title(ax2, 'Rucker EKF force convergence');

    ax3 = subplot(2, 2, 4, 'Parent', fig);
    tipErr = abs(tipCase.poseTrue(2, idx) - rucker.xHist(2, idx));
    plot(ax3, idx, robotScale * tipErr, 'Color', [0.45 0.1 0.65], 'LineWidth', 2);
    grid(ax3, 'on'); box(ax3, 'on');
    xlim(ax3, [1 nSteps]);
    ylim(ax3, [0, max(1.0, robotScale * max(tipErr(:)) * 1.2)]);
    xlabel(ax3, 'Time step');
    ylabel(ax3, '|tip y error| [mm]');
    title(ax3, sprintf('Current force estimate %.3f N, true %.3f N', ...
        rucker.forceEst(k), rucker.forceTrue(k)));

    drawnow;
    framePath = fullfile(cfg.outputDir, sprintf('_tmp_motion_frame_%03d.png', k));
    print(fig, framePath, '-dpng', '-r150');
    rgb = imread(framePath);
    [A, map] = rgb2ind(rgb, 256);
    if k == 1
        imwrite(A, map, gifPath, 'gif', 'LoopCount', inf, 'DelayTime', cfg.animationDelay);
    else
        imwrite(A, map, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', cfg.animationDelay);
    end
    if exist(framePath, 'file')
        delete(framePath);
    end
end

saveAnimationKeyframes(results, cfg, keyframePath, robotScale, forceScale);
if isvalid(fig)
    close(fig);
end
end


function saveAnimationKeyframes(results, cfg, filePath, robotScale, forceScale)
model = results.model;
tipCase = results.tipCase;
rucker = results.rucker;
frames = unique(round(linspace(1, numel(rucker.forceTrue), 6)));

fig = figure('Name', 'Continuum Robot Keyframes', 'Color', 'w', ...
    'Position', [120 120 1200 500], 'Visible', 'on');
cmap = turbo(numel(frames));
for i = 1:numel(frames)
    k = frames(i);
    plot(robotScale * model.s, robotScale * tipCase.shapeTrue(:, k), ...
        '-', 'LineWidth', 2.3, 'Color', cmap(i, :)); hold on;
    xTip = robotScale * model.s(end);
    yTip = robotScale * tipCase.shapeTrue(end, k);
    quiver(xTip, yTip, 0, forceScale * rucker.forceTrue(k), 0, ...
        'Color', cmap(i, :), 'LineWidth', 1.4, 'MaxHeadSize', 0.7);
end
grid on; box on; axis equal;
xlabel('Backbone x [mm]');
ylabel('Backbone y [mm]');
title('Continuum robot motion keyframes with true tip-force arrows');
legend(arrayfun(@(k) sprintf('step %d, F=%.2f N', k, rucker.forceTrue(k)), frames, ...
    'UniformOutput', false), 'Location', 'northwest');
saveFigure(fig, filePath);
if isvalid(fig)
    close(fig);
end
end


function shapeEst = reconstructRuckerShapes(xHist, cfg, model)
nSteps = size(xHist, 2);
shapeEst = zeros(cfg.nGrid, nSteps);
for k = 1:nSteps
    scale = xHist(4, k);
    force = xHist(5, k);
    [yLoad, ~] = tipForceInfluence(model, cfg, force);
    shapeEst(:, k) = scale * model.actShape + yLoad;
end
end


function printForceSummary(results, cfg)
summary = results.summary;
names = string(summary.names(:));
primaryMetric = summary.metric1;
secondaryMetric = summary.metric2;
passVec = summary.passVec;

T = table(names, primaryMetric, secondaryMetric, passVec, ...
    'VariableNames', {'Method', 'PrimaryMetric', 'SecondaryMetric', 'Pass'});

fprintf('\nForce sensing benchmark summary\n');
fprintf('Output directory: %s\n\n', cfg.outputDir);
disp(T);

fprintf('Detailed numerical outputs\n');
fprintf('  Rucker final force true/est [N]   : %.4f / %.4f\n', ...
    results.rucker.forceTrue(end), results.rucker.forceEst(end));
fprintf('  Rucker abs/rel err                : %.4f N / %.2f %%\n', ...
    results.rucker.finalAbsErr, 100 * results.rucker.finalRelErr);
fprintf('  Aloi net force true/est [N]       : %.4f / %.4f\n', ...
    results.aloi.netForceTrue, results.aloi.netForceEst);
fprintf('  Aloi centroid true/est [m]        : %.4f / %.4f\n', ...
    results.aloi.centroidTrue, results.aloi.centroidEst);
fprintf('  Aloi centroid err / shape RMSE    : %.4f m / %.4f mm\n', ...
    results.aloi.centroidErr, 1e3 * results.aloi.shapeRmse);
fprintf('  Ferguson centroid true/est [m]    : %.4f / %.4f\n', ...
    results.ferguson.centroidTrue, results.ferguson.centroidEst);
fprintf('  Ferguson centroid err             : %.4f m\n', results.ferguson.centroidErr);
fprintf('  Ferguson shape RMSE               : %.4f mm\n', 1e3 * results.ferguson.shapeRmse);
fprintf('  Raw measurement RMSE              : %.4f mm\n', 1e3 * results.ferguson.rawMeasurementRmse);
fprintf('  Ferguson residual norm            : %.4e\n', results.ferguson.finalResidualNorm);
end


function q = gaussianLoad(s, Fnet, mu, sigma)
sigma = max(sigma, 1e-4);
q = Fnet * exp(-0.5 * ((s - mu) ./ sigma).^2) ./ (sqrt(2 * pi) * sigma);
end


function [yLoad, thetaLoad] = tipForceInfluence(model, cfg, force)
s = model.s;
L = cfg.L;
yLoad = force .* s.^2 .* (3 * L - s) ./ (6 * cfg.EI);
thetaLoad = force .* s .* (2 * L - s) ./ (2 * cfg.EI);
end


function c = loadCentroid(s, q)
mass = trapz(s, abs(q));
if mass < 1e-12
    c = 0;
else
    c = trapz(s, s .* abs(q)) / mass;
end
end


function J = finiteDifferenceJacobian(fun, x, step)
f0 = fun(x);
m = numel(f0);
n = numel(x);
J = zeros(m, n);
for i = 1:n
    xp = x;
    xp(i) = xp(i) + step(i);
    fp = fun(xp);
    J(:, i) = (fp(:) - f0(:)) / step(i);
end
end


function y = smoothFromMeasurements(sGrid, sMeas, values, fallback)
y = interp1(sMeas, values, sGrid, 'pchip', 'extrap');
alpha = linspace(0.85, 0.15, numel(sGrid)).';
y = alpha .* y + (1 - alpha) .* fallback;
end


function x = packBatchState(y, theta, kappa, kappaDot, q)
x = [y; theta; kappa; kappaDot; q];
end


function [y, theta, kappa, kappaDot, q] = unpackBatchState(x, n)
y = x(1:n);
theta = x((n + 1):(2 * n));
kappa = x((2 * n + 1):(3 * n));
kappaDot = x((3 * n + 1):(4 * n));
q = x((4 * n + 1):(5 * n));
end


function x = projectBounds(x, lb, ub)
x = min(max(x, lb), ub);
end


function ensureOutputDir(pathStr)
if ~exist(pathStr, 'dir')
    mkdir(pathStr);
end
end


function saveFigure(figHandle, filePath)
drawnow;
try
    print(figHandle, filePath, '-dpng', '-r200');
catch
    exportgraphics(figHandle, filePath, 'Resolution', 200);
end
end
