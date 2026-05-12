function force_ferguson()
% Ferguson 2024 Batch method for distributed load estimation
clearvars;
clc;
close all;

cfg = defaultFergusonConfig();
results = runFergusonBenchmark(cfg);
printFergusonSummary(results, cfg);
plotFergusonReport(results, cfg);
end


function cfg = defaultFergusonConfig()
cfg.dim = 2;
cfg.seed = 7;
cfg.outputDir = fullfile(pwd, 'force_outputs');

cfg.L = 0.30;
cfg.EI = 0.03;
cfg.nGrid = 101;
cfg.nMeas = 21;
cfg.baseCurvature = 3.2;
cfg.shapeNoiseStd = 6.0e-4;
cfg.curvatureNoiseStd = 0.05;

cfg.ferguson.lambdaDynamics = 0.2;
cfg.ferguson.lambdaBoundary = 8.0;
cfg.ferguson.lambdaLoadSmooth = 3e-4;
cfg.ferguson.maxIter = 10;
cfg.ferguson.stepDamping = 0.7;
cfg.ferguson.posteriorBandSigma = 2.0;
end


function results = runFergusonBenchmark(cfg)
rng(cfg.seed, 'twister');
ensureOutputDir(cfg.outputDir);

model = beamInfluenceMatrices(cfg);
distCase = simulateDistributedCase(cfg, model);

results.config = cfg;
results.model = model;
results.distCase = distCase;
results.ferguson = estimateFergusonBatchLoad(cfg, model, distCase);
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

distCase.paramsTrue = paramsTrue;
distCase.qTrue = qTrue;
distCase.yTrue = yTrue;
distCase.thetaTrue = thetaTrue;
distCase.kappaTrue = kappaTrue;
distCase.yMeas = yMeas;
distCase.kappaMeas = kappaMeas;
distCase.rawMeasurementRmse = sqrt(mean((yMeas - yTrue(model.measIdx)).^2));
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
ferguson.qEst = qEst;
ferguson.qStd = qStd;
ferguson.posteriorBand = cfg.ferguson.posteriorBandSigma * qStd;
ferguson.shapeRmse = shapeRmse;
ferguson.centroidTrue = centroidTrue;
ferguson.centroidEst = centroidEst;
ferguson.centroidErr = abs(centroidEst - centroidTrue);
ferguson.rawMeasurementRmse = distCase.rawMeasurementRmse;
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


function q = gaussianLoad(s, Fnet, mu, sigma)
sigma = max(sigma, 1e-4);
q = Fnet * exp(-0.5 * ((s - mu) ./ sigma).^2) ./ (sqrt(2 * pi) * sigma);
end


function c = loadCentroid(s, q)
mass = trapz(s, abs(q));
if mass < 1e-12
    c = 0;
else
    c = trapz(s, s .* abs(q)) / mass;
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


function printFergusonSummary(results, cfg)
ferguson = results.ferguson;
fprintf('\n=== Ferguson 2024 Batch Method ===\n');
fprintf('Output directory: %s\n\n', cfg.outputDir);
fprintf('Centroid true/est [m]: %.4f / %.4f\n', ferguson.centroidTrue, ferguson.centroidEst);
fprintf('Centroid error: %.4f m (%.1f mm)\n', ferguson.centroidErr, 1e3 * ferguson.centroidErr);
fprintf('Shape RMSE: %.4f mm\n', 1e3 * ferguson.shapeRmse);
fprintf('Raw measurement RMSE: %.4f mm\n', 1e3 * ferguson.rawMeasurementRmse);
end


function plotFergusonReport(results, cfg)
ensureOutputDir(cfg.outputDir);
model = results.model;
distCase = results.distCase;
ferguson = results.ferguson;

fig = figure('Name', 'Ferguson Batch', 'Color', 'w', 'Position', [100 100 1200 800]);

subplot(2, 2, 1);
fillX = [model.s; flipud(model.s)];
fillY = [ferguson.qEst + ferguson.posteriorBand; flipud(ferguson.qEst - ferguson.posteriorBand)];
patch(fillX, fillY, [0.92 0.75 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.6); hold on;
plot(model.s, distCase.qTrue, 'k-', 'LineWidth', 2);
plot(model.s, ferguson.qEst, 'm-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Load q [N/m]');
title(sprintf('Load Posterior (Centroid Err: %.1f mm)', 1e3 * ferguson.centroidErr));
legend({'2\sigma band', 'True', 'Estimated'}, 'Location', 'best');

subplot(2, 2, 2);
plot(model.s, distCase.yTrue * 1e3, 'k-', 'LineWidth', 2); hold on;
plot(model.s, ferguson.yEst * 1e3, 'm-', 'LineWidth', 2);
plot(model.sMeas, distCase.yMeas * 1e3, 'bo', 'MarkerFaceColor', [0.3 0.6 1.0]);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Deflection y [mm]');
title(sprintf('Shape (RMSE: %.3f mm)', 1e3 * ferguson.shapeRmse));
legend({'True', 'Estimated', 'Measurements'}, 'Location', 'best');

subplot(2, 2, 3);
plot(model.sMeas, distCase.kappaMeas, 'bo', 'MarkerFaceColor', [0.3 0.6 1.0]); hold on;
plot(model.s, distCase.kappaTrue, 'k-', 'LineWidth', 2);
plot(model.s, ferguson.kappaEst, 'm-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Curvature \kappa [1/m]');
title('Curvature (FBG measurements)');
legend({'Measured', 'True', 'Estimated'}, 'Location', 'best');

subplot(2, 2, 4);
plot(model.s, distCase.qTrue - ferguson.qEst, 'm-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Load error [N/m]');
title('Load Estimation Error');

sgtitle('Ferguson 2024 Batch Load Estimation Method');
saveFigure(fig, fullfile(cfg.outputDir, 'ferguson_method.png'));
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
