function results = run_rod_plane_force_sensing_experiment(quickMode, overrides)
%RUN_ROD_PLANE_FORCE_SENSING_EXPERIMENT
% Rod-plane validation for the full constrained EKF/MAP force formulation.
%
% The forward trajectory is generated with Jia Shen's LCP-Continuum
% rod_plane contact model. The inverse part is intentionally separated from
% that solver: it receives simulated sparse FBG curvature data and a plane
% measurement, then solves the iterated constrained EKF/MAP problem
% from Formulation.pdf with a nonlinear Cosserat forward model, unilateral
% contact, and Coulomb friction complementarity constraints.
%
% Usage:
%   results = run_rod_plane_force_sensing_experiment();
%   results = run_rod_plane_force_sensing_experiment(true);
%   results = run_rod_plane_force_sensing_experiment(true, overrides);

if nargin < 1
    quickMode = false;
end
if nargin < 2
    overrides = struct;
end

clc;
close all;

rootDir = fileparts(mfilename('fullpath'));
lcpDir = fullfile(rootDir, 'LCP-Continuum');
addpath(genpath(lcpDir));

cfg = defaultExperimentConfig(rootDir, quickMode);
cfg = mergeStructRecursive(cfg, overrides);
ensureDir(cfg.outputDir);

rng(cfg.randomSeed, 'twister');

fprintf('\n=== Rod-plane force sensing validation ===\n');
fprintf('Output directory: %s\n', cfg.outputDir);

[tube0, obstaclesTruth, setupInfo] = makeRodPlaneScenario(cfg);
fprintf('Rod length %.1f mm, target bend %.1f deg, actual bend %.1f deg\n', ...
    tube0.s(end), cfg.targetBendDeg, setupInfo.actualBendDeg);
fprintf('Applied tip load: [%.3f %.3f %.3f] N\n', cfg.tipLoadN);

forward = runForwardRodPlaneSimulation(tube0, obstaclesTruth, cfg);
measurements = simulateSensingMeasurements(tube0, forward, cfg);
truthConsistency = diagnoseForwardTruthConsistency(tube0, forward, measurements, cfg);
mapCandidateDiagnostics = diagnoseMapCandidateCosts(tube0, forward, measurements, truthConsistency, cfg);

if cfg.diagnostics.stopAfterTruthConsistency
    results = struct;
    results.config = cfg;
    results.setup = setupInfo;
    results.forward = forward;
    results.measurements = measurements;
    results.truthConsistency = truthConsistency;
    results.mapCandidateDiagnostics = mapCandidateDiagnostics;
    fprintf('\nTruth consistency diagnostic complete.\n');
    fprintf('Final true-state residual %.3g, max equality residual %.3g, max inequality violation %.3g\n', ...
        truthConsistency.measurementResidualNorm(end), truthConsistency.maxEqualityResidual(end), ...
        truthConsistency.maxInequalityViolation(end));
    printFinalMapCandidateDiagnostics(mapCandidateDiagnostics);
    return;
end

ours = estimateForcesWithShapeAndEnvironment(tube0, measurements, cfg);
aloi = estimateForcesWithAloiBaseline(tube0, measurements, cfg);

results = struct;
results.config = cfg;
results.setup = setupInfo;
results.forward = forward;
results.measurements = measurements;
results.truthConsistency = truthConsistency;
results.mapCandidateDiagnostics = mapCandidateDiagnostics;
results.ours = ours;
results.aloi = aloi;
results.metrics = summarizeExperiment(results);

save(fullfile(cfg.outputDir, 'rod_plane_force_sensing_results.mat'), 'results', '-v7.3');
writeSummary(results);
writeTrajectoryCsv(results);
plotExperimentResults(results);

fprintf('\n=== Summary ===\n');
fprintf('Full constrained EKF-MAP final total-load error: %.2f%%\n', ...
    results.metrics.ours.finalRelativeErrorPct);
fprintf('Full constrained EKF-MAP total-load trajectory RMSE: %.3f N\n', ...
    results.metrics.ours.resultantRmse);
fprintf('Aloi baseline final total-load error: %.2f%%\n', ...
    results.metrics.aloi.finalRelativeErrorPct);
if results.metrics.ours.finalRelativeErrorPct > cfg.forceSensor.warningRelativeErrorPct || ...
        results.metrics.ours.finalMeasurementResidualNorm > cfg.forceSensor.warningMeasurementResidualNorm
    fprintf('WARNING: inverse estimate failed numerical sanity checks; inspect CSV/summary before using this run.\n');
end
fprintf('Saved results to %s\n', cfg.outputDir);
end


function cfg = defaultExperimentConfig(rootDir, quickMode)
cfg = struct;
cfg.rootDir = rootDir;
cfg.outputDir = fullfile(rootDir, 'force_outputs', 'rod_plane_force_sensing');
cfg.randomSeed = 7;

% Forward rod-plane setup. LCP-Continuum uses millimeters and Newtons.
% The default case is the separated-contact validation used in the README.
cfg.exposedLengthMm = 180;
cfg.targetBendDeg = 270;
cfg.wallDistanceMm = 10;
cfg.planeNormal = [0; 0; -1];
cfg.frictionMu = 0.8;
cfg.initialLowFrictionMu = 1e-3;
cfg.cornerRange = 0.5;
cfg.frictionConeEdges = 16;
cfg.betaMaxMm = 35;
cfg.numTimeSteps = 16;
cfg.tipLoadN = [0; 0; -3.5];

% Simulated sensing setup. The main validation assumes the shape and plane
% are known from the forward simulation; nonzero MAP measurement covariance
% below is used as an estimator weight, not as injected noise.
cfg.sensing.numFbgPoints = 24;
cfg.sensing.curvatureNoiseStd = 0.0;  % rad/mm
cfg.sensing.positionNoiseStdMm = 0.0;
cfg.sensing.planeOffsetBiasMm = 0.0;
cfg.sensing.planeOffsetNoiseStdMm = 0.0;
cfg.sensing.shapeSmoothing = 0.0;

% Full constrained EKF/MAP parameters. State:
% x = [p1(3); eta1(2); s1; f1n; beta1(m); lambda1; fe(3)].
cfg.forceSensor.numFrictionDirs = cfg.frictionConeEdges;
cfg.forceSensor.contactSearchBandMm = 4.0;
cfg.forceSensor.contactHistoryResetMm = 6.0;
cfg.forceSensor.minContactDepthMm = -1.0;
cfg.forceSensor.priorStd.planePointMm = [8; 8; 0.25];
cfg.forceSensor.priorStd.normalParam = [0.04; 0.04];
cfg.forceSensor.priorStd.sMm = 8.0;
cfg.forceSensor.priorStd.normalForceN = 35.0;
cfg.forceSensor.priorStd.betaN = 35.0;
cfg.forceSensor.priorStd.lambda = 10.0;
cfg.forceSensor.priorStd.tipForceN = 4.0;
cfg.forceSensor.useForceBounds = false;
cfg.forceSensor.forceBounds.normalForceN = 200.0;
cfg.forceSensor.forceBounds.betaN = 200.0;
cfg.forceSensor.forceBounds.lambda = 200.0;
cfg.forceSensor.forceBounds.tipForceN = 25.0;
cfg.forceSensor.processStd.planePointMm = [0.25; 0.25; 0.10];
cfg.forceSensor.processStd.normalParam = [0.01; 0.01];
cfg.forceSensor.processStd.sMm = 2.0;
cfg.forceSensor.processStd.normalForceN = 14.0;
cfg.forceSensor.processStd.betaN = 18.0;
cfg.forceSensor.processStd.lambda = 2.0;
cfg.forceSensor.processStd.tipForceN = 0.75;
cfg.forceSensor.measurementStd.curvature = 1.5e-4;
cfg.forceSensor.measurementStd.planePointMm = [8; 8; 0.13];
cfg.forceSensor.measurementStd.normalVector = [0.02; 0.02; 0.02];
cfg.forceSensor.maxEkfIterations = 2;
cfg.forceSensor.linearizedSolveMaxIter = 25;
cfg.forceSensor.convergenceTol = 3e-3;
cfg.forceSensor.finiteDifferenceStep = [];
cfg.forceSensor.penalty.inequality = 1e5;
cfg.forceSensor.penalty.complementarity = 1e5;
cfg.forceSensor.penalty.lineSearch = [1.0, 0.5, 0.25, 0.1, 0.03, 0.01];
cfg.forceSensor.solver = 'fmincon';
cfg.forceSensor.allowApproximateFallback = false;
cfg.forceSensor.showProgress = true;
cfg.forceSensor.fminconProgressInterval = 10;
cfg.forceSensor.dampingScales = [1.0, 0.5, 0.25, 0.1, 0.03, 0.0];
cfg.forceSensor.useMultiStart = false;
cfg.forceSensor.maxStartCandidates = 1;
cfg.forceSensor.warningRelativeErrorPct = 100.0;
cfg.forceSensor.warningMeasurementResidualNorm = 1.0e4;
cfg.diagnostics.stopAfterTruthConsistency = false;
cfg.diagnostics.runMapCandidateCosts = true;

% Aloi-style comparison: shape-only Gaussian load fitting. It does not use
% the plane contact/friction information.
cfg.aloi.numCenterCandidates = 31;
cfg.aloi.sigmaCandidatesMm = [6, 12, 22, 35];
cfg.aloi.tipSigmaCandidatesMm = [4, 8, 14];
cfg.aloi.ridge = 1e-7;

if quickMode
    cfg.outputDir = fullfile(rootDir, 'force_outputs', 'rod_plane_force_sensing_smoke_tmp');
    cfg.numTimeSteps = 6;
    cfg.betaMaxMm = 5;
    cfg.sensing.numFbgPoints = 12;
    cfg.forceSensor.maxEkfIterations = 1;
    cfg.forceSensor.linearizedSolveMaxIter = 25;
    cfg.forceSensor.solver = 'projected';
    cfg.forceSensor.allowApproximateFallback = true;
    cfg.aloi.numCenterCandidates = 17;
    cfg.aloi.sigmaCandidatesMm = [10, 25];
    cfg.aloi.tipSigmaCandidatesMm = [6, 12];
end
end


function [tube, obstacles, info] = makeRodPlaneScenario(cfg)
tube = CreatTube(cfg.exposedLengthMm);

baseBendRad = trapz(tube.s, sqrt(sum(tube.uhat(1:2, :).^2, 1)));
targetBendRad = deg2rad(cfg.targetBendDeg);
curvatureScale = targetBendRad / max(baseBendRad, eps);
tube.uhat = tube.uhat * curvatureScale;

Rbase = [0 0 1; 0 -1 0; 1 0 0];
tube.T_base = [Rbase, zeros(3, 1); zeros(1, 3), 1];

planePoint = [0; 0; cfg.wallDistanceMm];
obstacles = {createPlane(planePoint, cfg.planeNormal, cfg.frictionMu)};

[~, ~, pFree] = solveShape(tube.T_base, tube.uhat, tube.s);

info = struct;
info.baseBendDeg = rad2deg(baseBendRad);
info.curvatureScale = curvatureScale;
info.actualBendDeg = rad2deg(trapz(tube.s, sqrt(sum(tube.uhat(1:2, :).^2, 1))));
info.freeTip = pFree(:, end);
info.freeRange = [min(pFree, [], 2), max(pFree, [], 2)];
end


function forward = runForwardRodPlaneSimulation(tube0, obstacles, cfg)
param = struct('d', cfg.frictionConeEdges);
tube = tube0;

fprintf('\nForward LCP simulation (%d frames)...\n', cfg.numTimeSteps);

[u, contacts] = getInitialShape3(tube, obstacles);
contacts = [];
contacts = detectAdditionalContacts(tube, u, obstacles, contacts, cfg.cornerRange);

obstacles{1}.mu = cfg.initialLowFrictionMu;
[u, contacts, ~, R, p] = getFrictionalContactShape3WithTipForce(tube, obstacles, u, contacts, cfg.cornerRange, param, cfg.tipLoadN);
obstacles{1}.mu = cfg.frictionMu;

ns = length(tube.s);
nt = cfg.numTimeSteps;
beta = linspace(0, cfg.betaMaxMm, nt);

pTraj = zeros(3, ns, nt);
uTraj = zeros(3, ns, nt);
RTraj = zeros(3, 3, ns, nt);
forceResultant = zeros(3, nt);
contactCount = zeros(1, nt);
contactArcLength = nan(1, nt);
contactTraj = cell(1, nt);
baseTraj = zeros(4, 4, nt);

[forceResultant(:, 1), contactArcLength(1)] = summarizeContacts(contacts, tube.s);
pTraj(:, :, 1) = p;
uTraj(:, :, 1) = u;
RTraj(:, :, :, 1) = R;
contactCount(1) = length(contacts);
contactTraj{1} = contacts;
baseTraj(:, :, 1) = tube.T_base;

for it = 2:nt
    contacts = [];
    contacts = detectAdditionalContacts(tube, u, obstacles, contacts, cfg.cornerRange);

    tube.T_base(1, 4) = beta(it);
    [u, contacts, ~, R, p] = getFrictionalContactShape3WithTipForce(tube, obstacles, u, contacts, cfg.cornerRange, param, cfg.tipLoadN);

    pTraj(:, :, it) = p;
    uTraj(:, :, it) = u;
    RTraj(:, :, :, it) = R;
    [forceResultant(:, it), contactArcLength(it)] = summarizeContacts(contacts, tube.s);
    contactCount(it) = length(contacts);
    contactTraj{it} = contacts;
    baseTraj(:, :, it) = tube.T_base;

    if mod(it, max(1, floor(nt / 6))) == 0 || it == nt
        fprintf('  frame %3d/%3d, contacts=%d, resultant=[%8.3f %8.3f %8.3f] N\n', ...
            it, nt, contactCount(it), forceResultant(:, it));
    end
end

forward = struct;
forward.s = tube.s;
forward.betaMm = beta;
forward.p = pTraj;
forward.u = uTraj;
forward.R = RTraj;
forward.forceResultant = forceResultant;
forward.contactForceResultant = forceResultant;
forward.tipLoad = repmat(cfg.tipLoadN(:), 1, nt);
forward.totalForceResultant = forceResultant + forward.tipLoad;
forward.contactCount = contactCount;
forward.contactArcLength = contactArcLength;
forward.contacts = contactTraj;
forward.baseTraj = baseTraj;
forward.finalR = R;
end


function [u, contacts, T, R, p] = getFrictionalContactShape3WithTipForce(tube, obstacles, uPrev, contactsPrev, cornerRange, param, tipForce)
% Local copy of the LCP-Continuum frictional contact solve with an added
% known tip force. The upstream getFrictionalContactShape3.m is unchanged.

if nargin < 6 || isempty(param)
    d = 150;
else
    d = param.d;
end
if nargin < 7
    tipForce = zeros(3, 1);
end
tipForce = tipForce(:);

[contactsPrev, T, R, p] = detectAdditionalContacts(tube, uPrev, obstacles, contactsPrev, cornerRange);
[~, Rl, pl] = solveShape(tube.T_base, uPrev, tube.s);
J = computeJacobian(Rl, pl);
K = getTubeK(tube);
invK = 1 ./ K;
ds = tube.s(end) - tube.s(end - 1);

tipRows = 3 * length(tube.s) - (2:-1:0);
Jtip = J(tipRows, :);
mTip = Jtip' * tipForce;

if isempty(contactsPrev)
    u = reshape(invK .* mTip, 3, []) + tube.uhat;
    contacts = contactsPrev;
    [T, R, p] = solveShape(tube.T_base, u, tube.s);
    return;
end

nc = length(contactsPrev);
pcVec = zeros(3, nc);
Jc = zeros(3 * nc, size(J, 2));
bVec = zeros(3, nc);

for ic = 1:nc
    itc = contactsPrev(ic).tube_point_id;
    pt = contactsPrev(ic).tube_point;
    obs = obstacles{contactsPrev(ic).obstacle_id};

    if strcmp(contactsPrev(ic).type, 'cornerContact')
        itc3Prev = [3 * itc(1) - 2; 3 * itc(1) - 1; 3 * itc(1)];
        itc3 = [3 * itc(2) - 2; 3 * itc(2) - 1; 3 * itc(2)];
        piPrev = pt(:, 1);
        piCur = pt(:, 2);
        pPrev = contactsPrev(ic).point;
        diPrev = norm(piPrev - pPrev);
        diCur = norm(piCur - pPrev);
        dsum = diPrev + diCur;
        Jc(3 * ic - 2:3 * ic, :) = (diCur * J(itc3Prev, :) + diPrev * J(itc3, :)) / dsum;
        pProj = obs.project2corner(pPrev, 0);
        plc = (diCur * pl(:, itc(1)) + diPrev * pl(:, itc(2))) / dsum;
    else
        itc3 = [3 * itc - 2; 3 * itc - 1; 3 * itc];
        pProj = obs.project(pt);
        plc = pl(:, itc);
        Jc(3 * ic - 2:3 * ic, :) = J(itc3, :);
    end

    if size(obs.T_history, 3) >= 2
        Tcurr = obs.T_history(:, :, end);
        Tprev = obs.T_history(:, :, end - 1);
        pc = Tcurr * inv(Tprev) * [pProj; 1];
        pcVec(:, ic) = pc(1:3);
    else
        pcVec(:, ic) = pProj;
    end

    bVec(:, ic) = plc - pcVec(:, ic);
end

b = bVec(:);
mPrev = K .* (uPrev(:) - tube.uhat(:));
JcInvKds = Jc .* invK' * ds;
A = JcInvKds * Jc';
q = JcInvKds * (mTip - mPrev) + b;

n = zeros(3, nc, nc);
mu = zeros(nc, nc);
B = zeros(3, 2, nc);
for ic = 1:nc
    io = contactsPrev(ic).obstacle_id;
    nCur = contactsPrev(ic).normal;
    n(:, ic, ic) = nCur;
    mu(ic, ic) = obstacles{io}.mu;
    B(:, :, ic) = null(nCur');
end
n = reshape(n, [], nc);

theta = linspace(0, pi, round(d / 2) + 1);
theta = theta(1:end - 1);
cs = [cos(theta); sin(theta)];

D = zeros(3 * nc, d * nc);
e = zeros(d * nc, nc);
for ic = 1:nc
    Dcur = B(:, :, ic) * cs;
    Dcur = [Dcur, -Dcur];
    D(3 * ic - 2:3 * ic, d * (ic - 1) + 1:d * ic) = Dcur;
    e(d * (ic - 1) + 1:d * ic, ic) = 1;
end

icF = find(diag(mu) > 0);
icF = reshape(icF, 1, []);
ncF = length(icF);
if ncF ~= nc
    mu = mu(icF, :);
    col = d * repmat(icF, d, 1) - (d - 1:-1:0)';
    D = D(:, col);
    e = e(col, icF);
end

M = [n' * A * n, n' * A * D, zeros(nc, ncF);
     D' * A * n, D' * A * D, e;
     mu, -e', zeros(ncF, ncF)];
g = [n' * q; D' * q; zeros(ncF, 1)];

dimM = size(M, 1);
scale = 1;
Dw = diag([ones(1, dimM - ncF), ones(1, ncF) * scale]);
Dx = diag([ones(1, dimM - ncF) / scale, ones(1, ncF)]);

[~, xScaled] = LCPSolve(Dw * M * Dx, Dw * g, 1e-8, dimM^2);
x = Dx * xScaled;

fn = x(1:nc);
beta = x(nc + 1:nc + ncF * d);
lambda = x(nc + ncF * d + 1:end);

fContact = n * fn;
if ~isempty(beta)
    fContact = fContact + D * beta;
end

m = Jc' * fContact + mTip;
u = reshape(invK .* m, 3, []) + tube.uhat;

[T, R, p] = solveShape(tube.T_base, u, tube.s);

contacts = contactsPrev;
dp = A * fContact + q;
pContactsNew = pcVec + reshape(dp, 3, []);

icToRemove = [];
for ic = 1:nc
    if fn(ic) > 0
        contacts(ic).force = fContact(3 * ic - 2:3 * ic);
        contacts(ic).point = pContactsNew(:, ic);
        contacts(ic).normal_force = fn(ic);
        contacts(ic).friction_beta = zeros(d, 1);
        contacts(ic).friction_lambda = 0;
        contacts(ic).friction_directions = zeros(3, d);
        icFLocal = find(icF == ic, 1);
        if ~isempty(icFLocal)
            betaCols = d * (icFLocal - 1) + (1:d);
            contactRows = 3 * ic - (2:-1:0);
            contacts(ic).friction_beta = beta(betaCols);
            contacts(ic).friction_lambda = lambda(icFLocal);
            contacts(ic).friction_directions = D(contactRows, betaCols);
        end
    else
        icToRemove = [icToRemove, ic];
    end
end
contacts(icToRemove) = [];
end


function measurements = simulateSensingMeasurements(tube0, forward, cfg)
fprintf('\nSimulating sparse FBG/environment measurements...\n');

s = tube0.s(:);
nt = numel(forward.betaMm);
ns = numel(s);
fbgIdx = unique(round(linspace(1, ns, cfg.sensing.numFbgPoints)));
if fbgIdx(1) ~= 1
    fbgIdx = [1, fbgIdx];
end
if fbgIdx(end) ~= ns
    fbgIdx = [fbgIdx, ns];
end
fbgIdx = unique(fbgIdx, 'stable');
numFbg = numel(fbgIdx);

uMeasured = zeros(3, ns, nt);
pMeasured = zeros(3, ns, nt);
uSparse = zeros(3, numFbg, nt);
pSparse = zeros(3, numFbg, nt);
planeZMeasured = zeros(1, nt);

for it = 1:nt
    trueU = forward.u(:, :, it);
    sparseU = trueU(:, fbgIdx);
    sparseU = sparseU + cfg.sensing.curvatureNoiseStd * randn(size(sparseU));
    sparseU(3, :) = 0;

    sparseP = forward.p(:, fbgIdx, it);
    sparseP = sparseP + cfg.sensing.positionNoiseStdMm * randn(size(sparseP));

    uInterp = interpolateCurvature(s, s(fbgIdx), sparseU, cfg.sensing.shapeSmoothing);
    tube = tube0;
    tube.T_base = forward.baseTraj(:, :, it);
    [~, ~, pInterp] = solveShape(tube.T_base, uInterp, tube.s);

    uMeasured(:, :, it) = uInterp;
    pMeasured(:, :, it) = pInterp;
    uSparse(:, :, it) = sparseU;
    pSparse(:, :, it) = sparseP;
    planeZMeasured(it) = cfg.wallDistanceMm + cfg.sensing.planeOffsetBiasMm + ...
        cfg.sensing.planeOffsetNoiseStdMm * randn();
end

measurements = struct;
measurements.fbgIdx = fbgIdx;
measurements.sFbg = s(fbgIdx);
measurements.uSparse = uSparse;
measurements.pSparse = pSparse;
measurements.u = uMeasured;
measurements.p = pMeasured;
measurements.planeZMeasured = planeZMeasured;
measurements.planeNormalMeasured = cfg.planeNormal(:) / norm(cfg.planeNormal);
measurements.baseTraj = forward.baseTraj;
measurements.betaMm = forward.betaMm;
measurements.description = 'Sparse curvature/position samples plus a measured plane offset.';
end


function truth = diagnoseForwardTruthConsistency(tube0, forward, measurements, cfg)
nt = numel(measurements.betaMm);
nx = formulationStateSize(cfg);
state = nan(nx, nt);
measurementResidualNorm = nan(1, nt);
maxEqualityResidual = nan(1, nt);
maxInequalityViolation = nan(1, nt);
forceReconstructionResidual = nan(1, nt);
normalComplementarity = nan(1, nt);
frictionComplementarity = nan(1, nt);
coneComplementarity = nan(1, nt);
gap = nan(1, nt);
normalForce = nan(1, nt);
lambda = nan(1, nt);
coneSlack = nan(1, nt);
minFrictionW = nan(1, nt);
tangentialDisplacementNorm = nan(1, nt);
prevShape = [];

for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    contacts = forward.contacts{it};
    planePoint = [0; 0; measurements.planeZMeasured(it)];
    n = measurements.planeNormalMeasured(:) / norm(measurements.planeNormalMeasured);
    eta = normalToEta(n);

    if isempty(contacts)
        s1 = tube.s(end);
        contactForce = zeros(3, 1);
        ic = [];
    else
        contactForces = reshape([contacts.force], 3, []);
        [~, ic] = max(vecnorm(contactForces, 2, 1));
        contactForce = contacts(ic).force(:);
        s1 = tube.s(contacts(ic).tube_point_id);
    end

    [fn, beta, lambda, reconResidual] = contactVariablesFromForwardContact(contacts, ic, contactForce, n, cfg);
    x = [planePoint; eta; s1; fn; beta; lambda; forward.tipLoad(:, it)];
    x = projectMapState(x, tube, cfg);
    state(:, it) = x;

    z = measurementVectorForFrame(measurements, it);
    Rdiag = measurementStdVector(measurements, cfg) .^ 2;
    h = mapMeasurementModel(tube, x, cfg, prevShape);
    [c, ceq] = fullMapConstraints(tube, x, cfg, prevShape);
    decoded = decodeMapState(x, tube, cfg, prevShape);

    measurementResidualNorm(it) = norm((z - h) ./ sqrt(Rdiag));
    maxEqualityResidual(it) = max(abs(ceq));
    maxInequalityViolation(it) = max([0; c(:)]);
    forceReconstructionResidual(it) = reconResidual;
    normalComplementarity(it) = abs(decoded.gap * decoded.fn);
    frictionComplementarity(it) = norm(decoded.frictionW(:) .* decoded.beta(:));
    coneComplementarity(it) = abs(decoded.frictionConeSlack * decoded.lambda);
    gap(it) = decoded.gap;
    normalForce(it) = decoded.fn;
    lambda(it) = decoded.lambda;
    coneSlack(it) = decoded.frictionConeSlack;
    minFrictionW(it) = min(decoded.frictionW(:));
    tangentialDisplacementNorm(it) = norm(decoded.vTangential);

    prevShape = struct('p', forward.p(:, :, it), 'R', forward.R(:, :, :, it), ...
        'u', forward.u(:, :, it), 's1', s1);
end

truth = struct;
truth.state = state;
truth.measurementResidualNorm = measurementResidualNorm;
truth.maxEqualityResidual = maxEqualityResidual;
truth.maxInequalityViolation = maxInequalityViolation;
truth.forceReconstructionResidual = forceReconstructionResidual;
truth.normalComplementarity = normalComplementarity;
truth.frictionComplementarity = frictionComplementarity;
truth.coneComplementarity = coneComplementarity;
truth.gap = gap;
truth.normalForce = normalForce;
truth.lambda = lambda;
truth.coneSlack = coneSlack;
truth.minFrictionW = minFrictionW;
truth.tangentialDisplacementNorm = tangentialDisplacementNorm;
truth.description = 'Diagnostic only: forward contact/tip truth evaluated in the inverse formulation model.';
end


function diagnostics = diagnoseMapCandidateCosts(tube0, forward, measurements, truthConsistency, cfg)
candidateNames = {'truth', 'measurement_init_tip_only', 'reduced_seed', ...
    'reduced_seed_projected', 'zero_force'};
nc = numel(candidateNames);
nt = numel(measurements.betaMm);

diagnostics = struct;
diagnostics.candidateNames = candidateNames;
diagnostics.description = ['Diagnostic only: candidate states scored by the same nonlinear MAP merit ', ...
    'and complementarity constraints used by the formulation. The truth candidate is not used by the estimator.'];

diagnostics.merit = nan(nc, nt);
diagnostics.mapCost = nan(nc, nt);
diagnostics.measurementResidualNorm = nan(nc, nt);
diagnostics.priorResidualNorm = nan(nc, nt);
diagnostics.maxEqualityResidual = nan(nc, nt);
diagnostics.maxInequalityViolation = nan(nc, nt);
diagnostics.gap = nan(nc, nt);
diagnostics.normalForce = nan(nc, nt);
diagnostics.contactArcLength = nan(nc, nt);
diagnostics.vTangential = nan(3, nc, nt);
diagnostics.minFrictionW = nan(nc, nt);
diagnostics.contactForce = nan(3, nc, nt);
diagnostics.tipForce = nan(3, nc, nt);
diagnostics.totalForce = nan(3, nc, nt);

if ~isfield(cfg, 'diagnostics') || ~isfield(cfg.diagnostics, 'runMapCandidateCosts') || ...
        ~cfg.diagnostics.runMapCandidateCosts
    return;
end

prevShape = [];
for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    z = measurementVectorForFrame(measurements, it);
    Rdiag = measurementStdVector(measurements, cfg) .^ 2;
    xInit = initializeMapState(tube, measurements, it, cfg, []);

    if it == 1
        xPrior = initialMapPriorState(xInit, cfg);
        Pminus = diag(stateStdVector(cfg) .^ 2);
    else
        xPrior = truthConsistency.state(:, it - 1);
        Pminus = diag(processStdVector(cfg) .^ 2);
    end

    seed = solveReducedShapeKnownMapUpdate(tube, xInit, xPrior, z, cfg, measurements.u(:, :, it));
    xZeroForce = xInit(:);
    xZeroForce(7:end) = 0;

    candidates = [truthConsistency.state(:, it), ...
        xInit(:), ...
        seed.x(:), ...
        projectFullComplementarity(seed.x(:), tube, cfg, prevShape), ...
        xZeroForce(:)];

    for jc = 1:nc
        xCandidate = projectMapState(candidates(:, jc), tube, cfg);
        h = mapMeasurementModel(tube, xCandidate, cfg, prevShape);
        [c, ceq] = fullMapConstraints(tube, xCandidate, cfg, prevShape);
        decoded = decodeMapState(xCandidate, tube, cfg, prevShape);
        PInv = pinvSym(Pminus);
        priorResidual = xCandidate - xPrior;
        measurementResidual = z - h;

        diagnostics.mapCost(jc, it) = fullMapCost(xCandidate, xPrior, Pminus, z, Rdiag, h);
        diagnostics.merit(jc, it) = diagnostics.mapCost(jc, it) + ...
            cfg.forceSensor.penalty.inequality * sum(max(c, 0) .^ 2) + ...
            cfg.forceSensor.penalty.complementarity * sum(ceq .^ 2);
        diagnostics.measurementResidualNorm(jc, it) = norm(measurementResidual ./ sqrt(max(Rdiag(:), eps)));
        diagnostics.priorResidualNorm(jc, it) = sqrt(max(0, priorResidual' * PInv * priorResidual));
        diagnostics.maxEqualityResidual(jc, it) = max(abs(ceq));
        diagnostics.maxInequalityViolation(jc, it) = max([0; c(:)]);
        diagnostics.gap(jc, it) = decoded.gap;
        diagnostics.normalForce(jc, it) = decoded.fn;
        diagnostics.contactArcLength(jc, it) = decoded.s1;
        diagnostics.vTangential(:, jc, it) = decoded.vTangential;
        diagnostics.minFrictionW(jc, it) = min(decoded.frictionW(:));
        diagnostics.contactForce(:, jc, it) = decoded.contactForce;
        diagnostics.tipForce(:, jc, it) = decoded.tipForce;
        diagnostics.totalForce(:, jc, it) = decoded.totalForce;
    end

    prevShape = struct('p', forward.p(:, :, it), 'R', forward.R(:, :, :, it), ...
        'u', forward.u(:, :, it));
end
end


function printFinalMapCandidateDiagnostics(diagnostics)
if ~isfield(diagnostics, 'merit') || isempty(diagnostics.merit)
    return;
end

finalFrame = size(diagnostics.merit, 2);
if finalFrame == 0
    return;
end

fprintf('Final MAP candidate diagnostics:\n');
for j = 1:numel(diagnostics.candidateNames)
    fprintf('  %-28s merit=%10.4g, meas_res=%8.3g, eq=%8.3g, ineq=%8.3g, minW=%8.3g, fn=%8.3g, s=%8.3f mm\n', ...
        diagnostics.candidateNames{j}, diagnostics.merit(j, finalFrame), ...
        diagnostics.measurementResidualNorm(j, finalFrame), ...
        diagnostics.maxEqualityResidual(j, finalFrame), ...
        diagnostics.maxInequalityViolation(j, finalFrame), ...
        diagnostics.minFrictionW(j, finalFrame), diagnostics.normalForce(j, finalFrame), ...
        diagnostics.contactArcLength(j, finalFrame));
end
end


function [fn, beta, lambda, residual] = contactVariablesFromForwardContact(contacts, ic, contactForce, n, cfg)
if isempty(ic) || isempty(contacts) || ic < 1 || ic > numel(contacts)
    [fn, beta, lambda, residual] = decomposeContactForceForState(contactForce, n, cfg);
    return;
end

contact = contacts(ic);
if ~isfield(contact, 'normal_force') || ~isfield(contact, 'friction_beta') || ...
        ~isfield(contact, 'friction_lambda') || ~isfield(contact, 'friction_directions')
    [fn, beta, lambda, residual] = decomposeContactForceForState(contactForce, n, cfg);
    return;
end

m = cfg.forceSensor.numFrictionDirs;
Dtarget = frictionDirections(n, m);
Dsource = contact.friction_directions;
betaSource = contact.friction_beta(:);
beta = zeros(m, 1);

for js = 1:min(size(Dsource, 2), numel(betaSource))
    dir = Dsource(:, js);
    if norm(dir) <= eps || betaSource(js) <= 0
        continue;
    end
    dir = dir / norm(dir);
    [alignment, jt] = max(Dtarget' * dir);
    if alignment > 0.98
        beta(jt) = beta(jt) + betaSource(js);
    end
end

fn = max(0, contact.normal_force);
lambda = max(0, contact.friction_lambda);

reconstructed = n(:) / norm(n) * fn + Dtarget * beta;
residual = norm(reconstructed - contactForce(:));
if residual > max(1e-5, 1e-3 * max(1, norm(contactForce)))
    [fn, beta, lambda, residual] = decomposeContactForceForState(contactForce, n, cfg);
end
end


function [fn, beta, lambda, residual] = decomposeContactForceForState(contactForce, n, cfg)
m = cfg.forceSensor.numFrictionDirs;
n = n(:) / norm(n);
D = frictionDirections(n, m);
fn = max(0, n' * contactForce(:));
tangential = contactForce(:) - n * fn;

try
    beta = lsqnonneg(D, tangential);
catch
    beta = zeros(m, 1);
    if norm(tangential) > eps
        [~, idx] = max(D' * tangential);
        beta(idx) = max(0, D(:, idx)' * tangential);
    end
end

coneLimit = cfg.frictionMu * fn;
if sum(beta) > coneLimit && sum(beta) > 0
    beta = beta * (coneLimit / sum(beta));
end
lambda = 0;
residual = norm(n * fn + D * beta - contactForce(:));
end


function uInterp = interpolateCurvature(s, sFbg, sparseU, smoothing)
uInterp = zeros(3, numel(s));
for j = 1:3
    y = sparseU(j, :);
    yi = interp1(sFbg, y, s, 'pchip', 'extrap');
    if smoothing > 0
        yi = smoothdata(yi, 'movmean', max(3, round(smoothing * numel(sFbg))));
    end
    uInterp(j, :) = yi(:)';
end
end


function ours = estimateForcesWithShapeAndEnvironment(tube0, measurements, cfg)
fprintf('\nEstimating contact/tip force with full constrained EKF-MAP formulation...\n');

nt = numel(measurements.betaMm);
nx = formulationStateSize(cfg);
state = zeros(nx, nt);
posteriorCovariance = zeros(nx, nx, nt);
contactForce = zeros(3, nt);
tipForce = zeros(3, nt);
totalForce = zeros(3, nt);
contactArcLength = nan(1, nt);
contactIndex = nan(1, nt);
gap = nan(1, nt);
frictionSlack = nan(1, nt);
normalComplementarity = nan(1, nt);
frictionComplementarity = nan(1, nt);
coneComplementarity = nan(1, nt);
measurementNorm = nan(1, nt);
cost = nan(1, nt);
contactPoint = nan(3, nt);
seedContactForce = zeros(3, nt);
seedTipForce = zeros(3, nt);
seedTotalForce = zeros(3, nt);
seedContactArcLength = nan(1, nt);
seedMeasurementNorm = nan(1, nt);
seedNormalComplementarity = nan(1, nt);
seedFrictionComplementarity = nan(1, nt);
seedConeComplementarity = nan(1, nt);
seedMerit = nan(1, nt);
finalMerit = nan(1, nt);
initializationName = cell(1, nt);
initializationMerit = nan(1, nt);
initializationResidualNorm = nan(1, nt);
priorMean = [];
priorCovariance = [];
prevShape = [];
Q = diag(processStdVector(cfg) .^ 2);
inverseTimer = tic;
showProgress = isfield(cfg.forceSensor, 'showProgress') && cfg.forceSensor.showProgress;

for it = 1:nt
    frameTimer = tic;
    if showProgress
        fprintf('  inverse frame %3d/%3d started, insertion %.2f mm, elapsed %.1f min\n', ...
            it, nt, measurements.betaMm(it), toc(inverseTimer) / 60);
    end
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    z = measurementVectorForFrame(measurements, it);
    Rdiag = measurementStdVector(measurements, cfg) .^ 2;
    xInit = initializeMapState(tube, measurements, it, cfg, priorMean);
    if isempty(priorMean)
        xPrior = initialMapPriorState(xInit, cfg);
        Pminus = diag(stateStdVector(cfg) .^ 2);
    else
        xPrior = priorMean;
        Pminus = priorCovariance + Q;
    end

    progress = struct('frame', it, 'numFrames', nt);
    est = solveConstrainedMapUpdate(tube, xInit, xPrior, Pminus, z, Rdiag, cfg, prevShape, measurements.u(:, :, it), progress);
    state(:, it) = est.x;
    posteriorCovariance(:, :, it) = est.Pplus;
    priorMean = est.x;
    priorCovariance = est.Pplus;

    seedDecoded = decodeMapState(est.seed, tube, cfg, prevShape);
    seedContactForce(:, it) = seedDecoded.contactForce;
    seedTipForce(:, it) = seedDecoded.tipForce;
    seedTotalForce(:, it) = seedDecoded.totalForce;
    seedContactArcLength(it) = seedDecoded.s1;
    seedMeasurementNorm(it) = norm((z - mapMeasurementModel(tube, est.seed, cfg, prevShape)) ./ sqrt(Rdiag));
    seedNormalComplementarity(it) = abs(seedDecoded.gap * seedDecoded.fn);
    seedFrictionComplementarity(it) = norm(seedDecoded.frictionW(:) .* seedDecoded.beta(:));
    seedConeComplementarity(it) = abs(seedDecoded.frictionConeSlack * seedDecoded.lambda);
    seedMerit(it) = est.seedMerit;
    finalMerit(it) = est.nonlinearMerit;
    initializationName{it} = est.initInfo.name;
    initializationMerit(it) = est.initInfo.cost;
    initializationResidualNorm(it) = est.initInfo.residualNorm;

    decoded = decodeMapState(est.x, tube, cfg, prevShape);
    contactForce(:, it) = decoded.contactForce;
    tipForce(:, it) = decoded.tipForce;
    totalForce(:, it) = decoded.contactForce + decoded.tipForce;
    contactArcLength(it) = tube.s(est.contactIdx);
    contactIndex(it) = est.contactIdx;
    contactPoint(:, it) = est.contactPoint;
    gap(it) = est.surfaceGap;
    frictionSlack(it) = decoded.frictionConeSlack;
    normalComplementarity(it) = abs(decoded.gap * decoded.fn);
    frictionComplementarity(it) = norm(decoded.frictionW(:) .* decoded.beta(:));
    coneComplementarity(it) = abs(decoded.frictionConeSlack * decoded.lambda);
    measurementNorm(it) = norm(est.measurementResidual ./ sqrt(Rdiag));
    cost(it) = est.cost;
    prevShape = measuredShapeForFrame(tube, measurements.u(:, :, it), decoded.s1);

    if showProgress
        fprintf(['  inverse frame %3d/%3d done in %.1f s, s=%.2f mm, ', ...
            'contact=[%.3f %.3f %.3f] N, tip=[%.3f %.3f %.3f] N, ', ...
            'res=%.3g, seedRes=%.3g, merit=%.3g/%.3g, comp=[%.2g %.2g %.2g]\n'], ...
            it, nt, toc(frameTimer), decoded.s1, decoded.contactForce, decoded.tipForce, ...
            measurementNorm(it), seedMeasurementNorm(it), finalMerit(it), seedMerit(it), ...
            normalComplementarity(it), frictionComplementarity(it), coneComplementarity(it));
    end
end

if showProgress
    fprintf('  inverse pass complete in %.1f min\n', toc(inverseTimer) / 60);
end

ours = struct;
ours.state = state;
ours.posteriorCovariance = posteriorCovariance;
ours.forceResultant = contactForce;
ours.contactForceResultant = contactForce;
ours.tipForce = tipForce;
ours.totalForceResultant = totalForce;
ours.contactArcLength = contactArcLength;
ours.contactIndex = contactIndex;
ours.contactPoint = contactPoint;
ours.seedContactForceResultant = seedContactForce;
ours.seedTipForce = seedTipForce;
ours.seedTotalForceResultant = seedTotalForce;
ours.seedContactArcLength = seedContactArcLength;
ours.seedMeasurementResidualNorm = seedMeasurementNorm;
ours.seedNormalComplementarity = seedNormalComplementarity;
ours.seedFrictionComplementarity = seedFrictionComplementarity;
ours.seedConeComplementarity = seedConeComplementarity;
ours.seedMerit = seedMerit;
ours.finalMerit = finalMerit;
ours.initializationName = initializationName;
ours.initializationMerit = initializationMerit;
ours.initializationResidualNorm = initializationResidualNorm;
ours.gap = gap;
ours.frictionConeSlack = frictionSlack;
ours.normalComplementarity = normalComplementarity;
ours.frictionComplementarity = frictionComplementarity;
ours.coneComplementarity = coneComplementarity;
ours.measurementResidualNorm = measurementNorm;
ours.cost = cost;
ours.stateDescription = '[p1(3); eta1(2); s1; f1n; beta1(m); lambda1; fe(3)]';
ours.methodDescription = 'Full iterated constrained EKF/MAP from Formulation.pdf eqs. (19)-(29).';
end


function nx = formulationStateSize(cfg)
nx = 3 + 2 + 1 + 1 + cfg.forceSensor.numFrictionDirs + 1 + 3;
end


function z = measurementVectorForFrame(measurements, it)
z = measurements.uSparse(:, :, it);
z = z(:);
planePoint = [0; 0; measurements.planeZMeasured(it)];
normalVector = measurements.planeNormalMeasured(:) / norm(measurements.planeNormalMeasured);
z = [z; planePoint; normalVector];
end


function shape = measuredShapeForFrame(tube, measuredU, s1)
[~, R, p] = solveShape(tube.T_base, measuredU, tube.s);
shape = struct('p', p, 'R', R, 'u', measuredU, 's1', s1);
end


function x = initializeMapState(tube, measurements, it, cfg, priorMean)
u = measurements.u(:, :, it);
p = measurements.p(:, :, it);
planeZ = measurements.planeZMeasured(it);
normal = measurements.planeNormalMeasured;
eta = normalToEta(normal);

planePoint = [0; 0; planeZ];
signedGap = normal(:)' * (p - planePoint);
[~, idx] = min(abs(signedGap));
if abs(signedGap(idx)) > cfg.forceSensor.contactSearchBandMm
    [~, idx] = min(signedGap);
end
s1 = tube.s(idx);

[~, R, pShape] = solveShape(measurements.baseTraj(:, :, it), u, tube.s);
feSeed = seedTipOnlyForceFromMeasuredCurvature(tube, u, R, pShape);
fSeed = zeros(3, 1);

m = cfg.forceSensor.numFrictionDirs;
D = frictionDirections(normal, m);
fn = max(0, normal' * fSeed);
beta = max(0, D' * fSeed);
lambda = max(0, cfg.frictionMu * fn - sum(beta));

if ~isempty(priorMean)
    old = decodeMapState(priorMean, tube, cfg);
    feSeed = 0.7 * old.tipForce + 0.3 * feSeed;
end

x = [0; 0; planeZ; eta; s1; fn; beta; lambda; feSeed(:)];
end


function xPrior = initialMapPriorState(xInit, cfg)
m = cfg.forceSensor.numFrictionDirs;
xPrior = xInit(:);
xPrior(7) = 0;
xPrior(8:7 + m) = 0;
xPrior(8 + m) = 0;
xPrior(9 + m:11 + m) = 0;
if isfield(cfg.forceSensor, 'initialTipPriorN')
    xPrior(9 + m:11 + m) = cfg.forceSensor.initialTipPriorN(:);
end
end


function tipSeed = seedTipOnlyForceFromMeasuredCurvature(tube, u, R, p)
J = computeJacobian(R, p);
K = getTubeK(tube);
mMeasured = K .* (u(:) - tube.uhat(:));

tipRows = 3 * length(tube.s) - (2:-1:0);
A = J(tipRows, :)';
G = A' * A;
ridge = 1e-8 * max(trace(G) / max(size(G, 1), 1), 1);
tipSeed = (G + ridge * eye(3)) \ (A' * mMeasured);
tipSeed(~isfinite(tipSeed)) = 0;
end


function [contactSeed, tipSeed] = seedForcesFromMeasuredCurvature(tube, u, R, p, contactIdx, normal, cfg)
J = computeJacobian(R, p);
K = getTubeK(tube);
mMeasured = K .* (u(:) - tube.uhat(:));

contactRows = 3 * contactIdx - (2:-1:0);
tipRows = 3 * length(tube.s) - (2:-1:0);
Jc = J(contactRows, :);
Jtip = J(tipRows, :);

t = contactTangentFromNormal(normal);
B = [normal(:), t];
A = [Jc' * B, Jtip'];
y = (A' * A + 1e-6 * eye(size(A, 2))) \ (A' * mMeasured);

contactComponents = projectPlanarFrictionCone(y(1:2), cfg.frictionMu);
contactSeed = B * contactComponents;
tipSeed = y(3:5);
end


function components = projectPlanarFrictionCone(components, mu)
fn = max(0, components(1));
ft = components(2);
limit = mu * fn;
ft = min(max(ft, -limit), limit);
components = [fn; ft];
end


function est = solveConstrainedMapUpdate(tube, xInit, xPrior, Pminus, z, Rdiag, cfg, prevShape, measuredU, progress)
% Full iterated constrained EKF/MAP update from Formulation.pdf eqs. (19)-(29).
% Each iteration linearizes h(x), solves the constrained MAP subproblem with
% unilateral contact and Coulomb-friction complementarity constraints, then
% updates the posterior covariance from the final measurement Jacobian.
if nargin < 10
    progress = struct('frame', nan, 'numFrames', nan);
end
showProgress = isfield(cfg.forceSensor, 'showProgress') && cfg.forceSensor.showProgress;
seed = solveReducedShapeKnownMapUpdate(tube, xInit, xPrior, z, cfg, measuredU);
[x, initInfo] = chooseInitialMapLinearization(tube, xInit, xPrior, seed.x, ...
    xPrior, Pminus, z, Rdiag, cfg, prevShape);
startCandidates = mapStartCandidates(x, xInit, xPrior, seed.x, cfg, tube, prevShape);
if showProgress
    fprintf('    frame %3d/%3d init selected %s, merit %.4g, residual %.3g\n', ...
        progress.frame, progress.numFrames, initInfo.name, initInfo.cost, initInfo.residualNorm);
end

for iter = 1:cfg.forceSensor.maxEkfIterations
    if showProgress
        fprintf('    frame %3d/%3d EKF iter %d/%d: linearizing h(x)\n', ...
            progress.frame, progress.numFrames, iter, cfg.forceSensor.maxEkfIterations);
    end
    xLinearization = projectMapState(x, tube, cfg);
    h0 = mapMeasurementModel(tube, xLinearization, cfg, prevShape);
    H = finiteDifferenceMeasurementJacobian(tube, xLinearization, cfg, prevShape);
    progress.ekfIter = iter;
    xProposal = solveLinearizedConstrainedMapSubproblem(tube, xLinearization, xPrior, Pminus, z, Rdiag, h0, H, cfg, prevShape, progress, startCandidates);
    [xNext, acceptedAlpha, acceptedCost] = acceptDampedMapStep(tube, xLinearization, xProposal, ...
        xPrior, Pminus, z, Rdiag, cfg, prevShape);
    step = (xNext - xLinearization) ./ stateScaleVector(cfg);
    x = projectMapState(xNext, tube, cfg);
    if showProgress
        fprintf('    frame %3d/%3d EKF iter %d/%d: alpha %.2g, nonlinear cost %.4g, scaled step %.3g\n', ...
            progress.frame, progress.numFrames, iter, cfg.forceSensor.maxEkfIterations, ...
            acceptedAlpha, acceptedCost, norm(step));
    end
    if norm(step) < cfg.forceSensor.convergenceTol
        break;
    end
    startCandidates = mapStartCandidates(x, xInit, xPrior, seed.x, cfg, tube, prevShape);
end

decoded = decodeMapState(x, tube, cfg, prevShape);
hFinal = mapMeasurementModel(tube, x, cfg, prevShape);
Hfinal = finiteDifferenceMeasurementJacobian(tube, x, cfg, prevShape);
Pplus = posteriorCovarianceFromLinearization(Pminus, Rdiag, Hfinal);
constraint = complementarityResidual(decoded);

est = struct;
est.x = x;
est.Pplus = Pplus;
est.cost = fullMapCost(x, xPrior, Pminus, z, Rdiag, hFinal);
est.iterations = iter;
est.contactIdx = decoded.idx;
est.contactPoint = decoded.pc;
est.surfaceGap = decoded.gap;
est.measurementResidual = z - hFinal;
est.constraintResidual = constraint;
est.seed = seed.x;
est.seedInfo = seed;
est.initInfo = initInfo;
est.nonlinearMerit = nonlinearMapMerit(tube, x, xPrior, Pminus, z, Rdiag, cfg, prevShape);
est.seedMerit = nonlinearMapMerit(tube, seed.x, xPrior, Pminus, z, Rdiag, cfg, prevShape);
end


function [xBest, info] = chooseInitialMapLinearization(tube, xInit, xPriorCandidate, xSeed, ...
    xPrior, Pminus, z, Rdiag, cfg, prevShape)
% Pick the EKF linearization point using only the MAP objective and
% complementarity residuals. The reduced shape-known force solve is allowed
% to help initialization, but it is not trusted blindly.
xSeedProjected = projectFullComplementarity(xSeed(:), tube, cfg, prevShape);
names = {'prior', 'measurement_init', 'reduced_seed', 'reduced_seed_projected', 'prior_seed_blend'};
xZeroForce = xInit(:);
xZeroForce(7:end) = 0;
names = [names, {'zero_force'}];
candidates = [xPriorCandidate(:), xInit(:), xSeed(:), xSeedProjected(:), ...
    0.5 * (xPriorCandidate(:) + xSeedProjected(:)), xZeroForce];

bestValue = inf;
xBest = projectMapState(candidates(:, 1), tube, cfg);
bestResidual = inf;
for j = 1:size(candidates, 2)
    xCandidate = projectMapState(candidates(:, j), tube, cfg);
    h = mapMeasurementModel(tube, xCandidate, cfg, prevShape);
    [c, ceq] = fullMapConstraints(tube, xCandidate, cfg, prevShape);
    priorResidual = xCandidate - xPrior;
    measurementResidual = z - h;
    PInv = pinvSym(Pminus);
    value = 0.5 * priorResidual' * PInv * priorResidual + ...
        0.5 * sum((measurementResidual .^ 2) ./ max(Rdiag(:), eps)) + ...
        cfg.forceSensor.penalty.inequality * sum(max(c, 0) .^ 2) + ...
        cfg.forceSensor.penalty.complementarity * sum(ceq .^ 2);
    residualNorm = norm(measurementResidual ./ sqrt(max(Rdiag(:), eps)));
    if value < bestValue
        bestValue = value;
        bestResidual = residualNorm;
        xBest = xCandidate;
        info = struct('name', names{j}, 'cost', value, 'residualNorm', residualNorm);
    end
end

if ~isfinite(bestValue)
    info = struct('name', 'prior', 'cost', bestValue, 'residualNorm', bestResidual);
end
end


function candidates = mapStartCandidates(xCurrent, xInit, xPrior, xSeed, cfg, tube, prevShape)
xZeroForce = xInit(:);
xZeroForce(7:end) = 0;
xSeedProjected = projectFullComplementarity(xSeed(:), tube, cfg, prevShape);
xCurrentProjected = projectFullComplementarity(xCurrent(:), tube, cfg, prevShape);
candidates = [xCurrent(:), xInit(:), xPrior(:), xSeed(:), ...
    xSeedProjected(:), xCurrentProjected(:), xZeroForce(:)];
if isfield(cfg.forceSensor, 'maxStartCandidates') && cfg.forceSensor.maxStartCandidates > 0
    candidates = candidates(:, 1:min(size(candidates, 2), cfg.forceSensor.maxStartCandidates));
end
end


function est = solveReducedShapeKnownMapUpdate(tube, xInit, xPrior, z, cfg, measuredU)
% Shape-known force-balance solve used only to initialize the full nonlinear
% constrained EKF/MAP iteration.
x = projectMapState(xInit, tube, cfg);
m = cfg.forceSensor.numFrictionDirs;
n = etaToNormal(x(4:5));
D = frictionDirections(n, m);

[~, R, p] = solveShape(tube.T_base, measuredU, tube.s);
J = computeJacobian(R, p);
K = getTubeK(tube);
mMeasured = K .* (measuredU(:) - tube.uhat(:));

forcePrior = [xPrior(7); xPrior(8:7 + m); xPrior(9 + m:11 + m)];
forceStd = [cfg.forceSensor.priorStd.normalForceN; ...
            cfg.forceSensor.priorStd.betaN * ones(m, 1); ...
            cfg.forceSensor.priorStd.tipForceN * ones(3, 1)];
tipRows = 3 * length(tube.s) - (2:-1:0);

candidateIdx = contactCandidateIndices(tube, p, x(1:3), n, cfg);
best = struct('score', inf, 'idx', candidateIdx(1), 'y', forcePrior, 'residual', inf);
normalizer = max(norm(mMeasured), 1);

for ic = 1:numel(candidateIdx)
    contactIdx = candidateIdx(ic);
    contactRows = 3 * contactIdx - (2:-1:0);
    A = [J(contactRows, :)' * n, J(contactRows, :)' * D, J(tipRows, :)'];
    G = (A' * A) / normalizer^2 + diag(1 ./ forceStd.^2);
    rhs = (A' * mMeasured) / normalizer^2 + forcePrior ./ forceStd.^2;
    yCandidate = solveProjectedForceQuadratic(G, rhs, forcePrior, cfg);

    residual = norm(A * yCandidate - mMeasured) / normalizer;
    surfaceGap = n' * (p(:, contactIdx) - x(1:3));
    priorPenalty = abs(tube.s(contactIdx) - xPrior(6)) / max(cfg.forceSensor.priorStd.sMm, eps);
    normalForcePenalty = 0.02 / max(yCandidate(1), 0.02);
    score = residual + 0.2 * abs(surfaceGap) + 0.03 * priorPenalty + normalForcePenalty;
    if score < best.score
        best.score = score;
        best.idx = contactIdx;
        best.y = yCandidate;
        best.residual = residual;
    end
end

y = best.y;

x(7) = y(1);
x(8:7 + m) = y(2:1 + m);
x(8 + m) = max(0, cfg.frictionMu * x(7) - sum(x(8:7 + m)));
x(9 + m:11 + m) = y(2 + m:end);
x(6) = tube.s(best.idx);
x = projectMapState(x, tube, cfg);

est = struct;
est.x = x;
est.cost = best.residual;
est.contactIdx = best.idx;
est.contactPoint = p(:, best.idx);
est.surfaceGap = n' * (p(:, best.idx) - x(1:3));
est.measurementResidual = z - mapMeasurementModel(tube, x, cfg);
end


function x = solveLinearizedConstrainedMapSubproblem(tube, xLinearization, xPrior, Pminus, z, Rdiag, h0, H, cfg, prevShape, progress, startCandidates)
if nargin < 11
    progress = struct('frame', nan, 'numFrames', nan, 'ekfIter', nan);
end
if nargin < 12 || isempty(startCandidates)
    startCandidates = xLinearization(:);
end
PInv = pinvSym(Pminus);
objective = @(xc) linearizedMapCost(xc(:), xLinearization, xPrior, PInv, z, Rdiag, h0, H);
nonlcon = @(xc) fullMapConstraints(tube, xc(:), cfg, prevShape);
[lb, ub] = stateBounds(tube, cfg);
solver = lower(char(cfg.forceSensor.solver));

if strcmp(solver, 'projected')
    x = solveProjectedLinearizedMap(tube, xLinearization, xPrior, PInv, z, Rdiag, h0, H, cfg, prevShape);
    return;
end

useFmincon = strcmp(solver, 'fmincon') && exist('fmincon', 'file') == 2;
if useFmincon
    try
        outputFcn = @(xcur, optimValues, state) fminconProgressOutput(xcur, optimValues, state, cfg, progress);
        if exist('optimoptions', 'file') == 2
            opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
                'MaxIterations', cfg.forceSensor.linearizedSolveMaxIter, ...
                'MaxFunctionEvaluations', 4000, ...
                'OptimalityTolerance', 1e-5, ...
                'ConstraintTolerance', 1e-5, ...
                'StepTolerance', 1e-5, ...
                'OutputFcn', outputFcn);
        else
            opts = optimset('Display', 'off', 'MaxIter', cfg.forceSensor.linearizedSolveMaxIter, ...
                'OutputFcn', outputFcn);
        end
        if isfield(cfg.forceSensor, 'showProgress') && cfg.forceSensor.showProgress
            fprintf('      frame %3d/%3d EKF iter %d: solving constrained MAP with fmincon\n', ...
                progress.frame, progress.numFrames, progress.ekfIter);
        end
        if ~isfield(cfg.forceSensor, 'useMultiStart') || ~cfg.forceSensor.useMultiStart
            startCandidates = xLinearization(:);
        end
        bestX = [];
        bestValue = inf;
        bestStart = 1;
        for istart = 1:size(startCandidates, 2)
            x0 = projectMapState(startCandidates(:, istart), tube, cfg);
            xCandidate = fmincon(objective, x0, [], [], [], [], lb, ub, nonlcon, opts);
            xCandidate = projectMapState(xCandidate, tube, cfg);
            value = constrainedSubproblemMerit(xCandidate, objective, nonlcon, cfg);
            if value < bestValue
                bestValue = value;
                bestX = xCandidate;
                bestStart = istart;
            end
        end
        if isempty(bestX)
            error('fmincon did not return a candidate solution.');
        end
        if isfield(cfg.forceSensor, 'showProgress') && cfg.forceSensor.showProgress && size(startCandidates, 2) > 1
            fprintf('      fmincon multistart selected start %d/%d, merit %.4g\n', ...
                bestStart, size(startCandidates, 2), bestValue);
        end
        x = bestX;
        return;
    catch solveErr
        if ~cfg.forceSensor.allowApproximateFallback
            error('Full formulation solve failed in fmincon: %s', solveErr.message);
        end
        fprintf('  fmincon failed (%s); using projected approximate fallback.\n', solveErr.message);
    end
end

if ~cfg.forceSensor.allowApproximateFallback
    error('The full formulation solver requires MATLAB fmincon. Set cfg.forceSensor.solver=''projected'' only for approximate smoke tests.');
end
x = solveProjectedLinearizedMap(tube, xLinearization, xPrior, PInv, z, Rdiag, h0, H, cfg, prevShape);
end


function value = constrainedSubproblemMerit(x, objective, nonlcon, cfg)
[c, ceq] = nonlcon(x);
value = objective(x) + ...
    cfg.forceSensor.penalty.inequality * sum(max(c, 0) .^ 2) + ...
    cfg.forceSensor.penalty.complementarity * sum(ceq .^ 2);
end


function [xAccepted, acceptedAlpha, acceptedCost] = acceptDampedMapStep(tube, xCurrent, xProposal, xPrior, Pminus, z, Rdiag, cfg, prevShape)
alphas = cfg.forceSensor.dampingScales;
if isempty(alphas)
    alphas = [1.0, 0.5, 0.25, 0.1, 0.0];
end

bestCost = inf;
bestX = projectMapState(xCurrent, tube, cfg);
bestAlpha = 0;
for ia = 1:numel(alphas)
    alpha = alphas(ia);
    xCandidate = xCurrent + alpha * (xProposal - xCurrent);
    xCandidate = projectMapState(xCandidate, tube, cfg);
    cost = nonlinearMapMerit(tube, xCandidate, xPrior, Pminus, z, Rdiag, cfg, prevShape);
    if cost < bestCost
        bestCost = cost;
        bestX = xCandidate;
        bestAlpha = alpha;
    end
end

xAccepted = bestX;
acceptedAlpha = bestAlpha;
acceptedCost = bestCost;
end


function value = nonlinearMapMerit(tube, x, xPrior, Pminus, z, Rdiag, cfg, prevShape)
h = mapMeasurementModel(tube, x, cfg, prevShape);
value = fullMapCost(x, xPrior, Pminus, z, Rdiag, h);
[c, ceq] = fullMapConstraints(tube, x, cfg, prevShape);
value = value + cfg.forceSensor.penalty.inequality * sum(max(c, 0) .^ 2) + ...
    cfg.forceSensor.penalty.complementarity * sum(ceq .^ 2);
end


function stop = fminconProgressOutput(~, optimValues, state, cfg, progress)
stop = false;
if ~isfield(cfg.forceSensor, 'showProgress') || ~cfg.forceSensor.showProgress
    return;
end

interval = cfg.forceSensor.fminconProgressInterval;
if isempty(interval) || interval < 1
    interval = 10;
end

switch state
    case 'init'
        fprintf('        fmincon started\n');
    case 'iter'
        iteration = progressFieldOrDefault(optimValues, 'iteration', 0);
        if iteration == 1 || mod(iteration, interval) == 0
            fval = progressFieldOrDefault(optimValues, 'fval', nan);
            constr = progressFieldOrDefault(optimValues, 'constrviolation', nan);
            step = progressFieldOrDefault(optimValues, 'stepsize', nan);
            fprintf('        fmincon iter %3d: objective %.4g, constraint %.3g, step %.3g\n', ...
                iteration, fval, constr, step);
        end
    case 'done'
        iteration = progressFieldOrDefault(optimValues, 'iteration', nan);
        fval = progressFieldOrDefault(optimValues, 'fval', nan);
        constr = progressFieldOrDefault(optimValues, 'constrviolation', nan);
        fprintf('        fmincon done: iter %.0f, objective %.4g, constraint %.3g\n', ...
            iteration, fval, constr);
end
end


function value = progressFieldOrDefault(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end


function x = solveProjectedLinearizedMap(tube, xLinearization, xPrior, PInv, z, Rdiag, h0, H, cfg, prevShape)
weightedH = bsxfun(@rdivide, H, max(Rdiag(:), eps));
G = PInv + H' * weightedH;
rhs = PInv * xPrior + H' * ((z - h0 + H * xLinearization) ./ max(Rdiag(:), eps));
xRaw = pinvSym(G) * rhs;

candidates = zeros(numel(xRaw), 4);
candidates(:, 1) = xRaw;
candidates(:, 2) = 0.5 * xRaw + 0.5 * xLinearization;
candidates(:, 3) = xLinearization;
candidates(:, 4) = xPrior;

bestX = projectFullComplementarity(candidates(:, 1), tube, cfg, prevShape);
bestValue = inf;
for j = 1:size(candidates, 2)
    xCandidate = projectFullComplementarity(candidates(:, j), tube, cfg, prevShape);
    [c, ceq] = fullMapConstraints(tube, xCandidate, cfg, prevShape);
    violation = max(c, 0);
    value = nonlinearCandidateMapCost(tube, xCandidate, xPrior, PInv, z, Rdiag, cfg, prevShape) + ...
        cfg.forceSensor.penalty.inequality * sum(violation .^ 2) + ...
        cfg.forceSensor.penalty.complementarity * sum(ceq .^ 2);
    if value < bestValue
        bestValue = value;
        bestX = xCandidate;
    end
end

x = bestX;
end


function value = nonlinearCandidateMapCost(tube, x, xPrior, PInv, z, Rdiag, cfg, prevShape)
h = mapMeasurementModel(tube, x, cfg, prevShape);
priorResidual = x - xPrior;
measurementResidual = z - h;
value = 0.5 * priorResidual' * PInv * priorResidual + ...
    0.5 * sum((measurementResidual .^ 2) ./ max(Rdiag(:), eps));
end


function x = projectFullComplementarity(x, tube, cfg, prevShape)
m = cfg.forceSensor.numFrictionDirs;
x = projectMapState(x, tube, cfg);
betaIdx = 8:7 + m;
lambdaIdx = 8 + m;

for iter = 1:4
    decoded = decodeMapState(x, tube, cfg, prevShape);

    % Enforce 0 <= g perpendicular f_n >= 0. When a contact force is active,
    % move the plane point along the estimated normal so the surface gap is zero.
    if decoded.fn > 1e-8 || decoded.gap < 0
        x(1:3) = x(1:3) + decoded.n * decoded.gap;
    end

    decoded = decodeMapState(x, tube, cfg, prevShape);
    tangentialDrift = decoded.frictionW(:) - decoded.lambda;
    if sum(decoded.beta) > eps
        [~, activeDir] = max(decoded.beta);
    else
        [~, activeDir] = min(tangentialDrift);
    end
    lambda = max(0, -tangentialDrift(activeDir));
    w = tangentialDrift + lambda;
    if any(w < -1e-8)
        lambda = max(lambda, -min(tangentialDrift));
        w = tangentialDrift + lambda;
    end

    fn = max(0, decoded.fn);
    totalBeta = min(sum(decoded.beta), cfg.frictionMu * fn);
    if lambda > 1e-8
        totalBeta = cfg.frictionMu * fn;
    end

    active = abs(w) <= max(1e-7, 1e-5 * max(1, norm(w)));
    if ~any(active)
        active(activeDir) = true;
    end
    beta = zeros(m, 1);
    oldActive = decoded.beta(active);
    if sum(oldActive) > eps
        beta(active) = totalBeta * oldActive / sum(oldActive);
    else
        beta(active) = totalBeta / nnz(active);
    end

    x(betaIdx) = beta;
    x(lambdaIdx) = lambda;
    x = projectMapState(x, tube, cfg);
end
end


function value = linearizedMapCost(x, xLinearization, xPrior, PInv, z, Rdiag, h0, H)
priorResidual = x - xPrior;
measurementResidual = z - h0 - H * (x - xLinearization);
value = 0.5 * priorResidual' * PInv * priorResidual + ...
    0.5 * sum((measurementResidual .^ 2) ./ max(Rdiag(:), eps));
end


function value = fullMapCost(x, xPrior, Pminus, z, Rdiag, h)
priorResidual = x - xPrior;
measurementResidual = z - h;
PInv = pinvSym(Pminus);
value = 0.5 * priorResidual' * PInv * priorResidual + ...
    0.5 * sum((measurementResidual .^ 2) ./ max(Rdiag(:), eps));
end


function x = solveLinearizedMapWithPenalty(tube, x0, objective, nonlcon, cfg)
scale = stateScaleVector(cfg);
y = projectMapState(x0, tube, cfg) ./ scale;
bestValue = penaltyObjective(y, scale, tube, objective, nonlcon, cfg);

for iter = 1:cfg.forceSensor.linearizedSolveMaxIter
    grad = finiteDifferencePenaltyGradient(y, scale, tube, objective, nonlcon, cfg);
    if norm(grad) < 1e-8
        break;
    end
    accepted = false;
    for alpha = cfg.forceSensor.penalty.lineSearch
        yCandidate = projectMapState((y - alpha * grad) .* scale, tube, cfg) ./ scale;
        candidateValue = penaltyObjective(yCandidate, scale, tube, objective, nonlcon, cfg);
        if candidateValue < bestValue
            y = yCandidate;
            bestValue = candidateValue;
            accepted = true;
            break;
        end
    end
    if ~accepted
        break;
    end
end

x = projectMapState(y .* scale, tube, cfg);
end


function value = penaltyObjective(y, scale, tube, objective, nonlcon, cfg)
x = projectMapState(y(:) .* scale, tube, cfg);
[c, ceq] = nonlcon(x);
ineqViolation = max(c, 0);
value = objective(x) + ...
    cfg.forceSensor.penalty.inequality * sum(ineqViolation .^ 2) + ...
    cfg.forceSensor.penalty.complementarity * sum(ceq .^ 2);
end


function grad = finiteDifferencePenaltyGradient(y, scale, tube, objective, nonlcon, cfg)
grad = zeros(size(y));
step = 1e-4;
for j = 1:numel(y)
    yp = y;
    ym = y;
    yp(j) = yp(j) + step;
    ym(j) = ym(j) - step;
    fp = penaltyObjective(yp, scale, tube, objective, nonlcon, cfg);
    fm = penaltyObjective(ym, scale, tube, objective, nonlcon, cfg);
    grad(j) = (fp - fm) / (2 * step);
end
grad = grad / max(1, norm(grad));
end


function [c, ceq] = fullMapConstraints(tube, x, cfg, prevShape)
decoded = decodeMapState(x, tube, cfg, prevShape);
c = [-decoded.gap; -decoded.frictionW(:); -decoded.frictionConeSlack];
ceq = [decoded.gap * decoded.fn; ...
       decoded.frictionW(:) .* decoded.beta(:); ...
       decoded.frictionConeSlack * decoded.lambda];
end


function H = finiteDifferenceMeasurementJacobian(tube, x, cfg, prevShape)
hBase = mapMeasurementModel(tube, x, cfg, prevShape);
nx = formulationStateSize(cfg);
H = zeros(numel(hBase), nx);

step = cfg.forceSensor.finiteDifferenceStep(:);
if numel(step) ~= nx
    step = max(stateScaleVector(cfg) * 1e-4, 1e-6 * ones(nx, 1));
end

for j = 1:nx
    xp = x;
    xm = x;
    xp(j) = xp(j) + step(j);
    xm(j) = xm(j) - step(j);
    xp = projectMapState(xp, tube, cfg);
    xm = projectMapState(xm, tube, cfg);
    denom = xp(j) - xm(j);
    if abs(denom) < eps
        continue;
    end
    hp = mapMeasurementModel(tube, xp, cfg, prevShape);
    hm = mapMeasurementModel(tube, xm, cfg, prevShape);
    H(:, j) = (hp - hm) / denom;
end
end


function Pplus = posteriorCovarianceFromLinearization(Pminus, Rdiag, H)
PInv = pinvSym(Pminus);
weightedH = bsxfun(@rdivide, H, max(Rdiag(:), eps));
information = PInv + H' * weightedH;
Pplus = pinvSym(information);
Pplus = 0.5 * (Pplus + Pplus');
end


function Ainv = pinvSym(A)
A = 0.5 * (A + A');
[V, D] = eig(A);
d = diag(D);
tol = max(size(A)) * eps(max(abs(d)));
dInv = zeros(size(d));
active = abs(d) > tol;
dInv(active) = 1 ./ d(active);
Ainv = V * diag(dInv) * V';
Ainv = 0.5 * (Ainv + Ainv');
end


function [lb, ub] = stateBounds(tube, cfg)
nx = formulationStateSize(cfg);
m = cfg.forceSensor.numFrictionDirs;
lb = -inf(nx, 1);
ub = inf(nx, 1);
lb(4) = -pi;
ub(4) = pi;
lb(5) = -pi / 2 + 1e-4;
ub(5) = pi / 2 - 1e-4;
lb(6) = tube.s(1);
ub(6) = tube.s(end);
lb(7) = 0;
lb(8:7 + m) = 0;
lb(8 + m) = 0;
if forceBoundsEnabled(cfg)
    ub(7) = cfg.forceSensor.forceBounds.normalForceN;
    ub(8:7 + m) = cfg.forceSensor.forceBounds.betaN;
    ub(8 + m) = cfg.forceSensor.forceBounds.lambda;
    lb(9 + m:11 + m) = -cfg.forceSensor.forceBounds.tipForceN;
    ub(9 + m:11 + m) = cfg.forceSensor.forceBounds.tipForceN;
end
end


function scale = stateScaleVector(cfg)
scale = stateStdVector(cfg);
scale(scale <= 0) = 1;
end


function enabled = forceBoundsEnabled(cfg)
enabled = isfield(cfg.forceSensor, 'useForceBounds') && cfg.forceSensor.useForceBounds;
end


function stdVec = stateStdVector(cfg)
m = cfg.forceSensor.numFrictionDirs;
stdVec = [cfg.forceSensor.priorStd.planePointMm(:); ...
          cfg.forceSensor.priorStd.normalParam(:); ...
          cfg.forceSensor.priorStd.sMm; ...
          cfg.forceSensor.priorStd.normalForceN; ...
          cfg.forceSensor.priorStd.betaN * ones(m, 1); ...
          cfg.forceSensor.priorStd.lambda; ...
          cfg.forceSensor.priorStd.tipForceN * ones(3, 1)];
end


function stdVec = processStdVector(cfg)
m = cfg.forceSensor.numFrictionDirs;
stdVec = [cfg.forceSensor.processStd.planePointMm(:); ...
          cfg.forceSensor.processStd.normalParam(:); ...
          cfg.forceSensor.processStd.sMm; ...
          cfg.forceSensor.processStd.normalForceN; ...
          cfg.forceSensor.processStd.betaN * ones(m, 1); ...
          cfg.forceSensor.processStd.lambda; ...
          cfg.forceSensor.processStd.tipForceN * ones(3, 1)];
end


function stdVec = measurementStdVector(measurements, cfg)
numFbg = numel(measurements.fbgIdx);
stdVec = [cfg.forceSensor.measurementStd.curvature * ones(3 * numFbg, 1); ...
          cfg.forceSensor.measurementStd.planePointMm(:); ...
          cfg.forceSensor.measurementStd.normalVector(:)];
end


function residual = complementarityResidual(decoded)
residual = struct;
residual.normal = abs(decoded.gap * decoded.fn);
residual.friction = norm(decoded.frictionW(:) .* decoded.beta(:));
residual.cone = abs(decoded.frictionConeSlack * decoded.lambda);
residual.minInequality = min([decoded.gap; decoded.frictionW(:); decoded.frictionConeSlack]);
end


function p = measurementsFreeShape(tube)
[~, ~, p] = solveShape(tube.T_base, tube.uhat, tube.s);
end


function candidateIdx = contactCandidateIndices(tube, p, planePoint, n, cfg)
surfaceGap = n(:)' * (p - planePoint(:));
band = max(cfg.forceSensor.contactSearchBandMm, tube.rout);
candidateIdx = find(abs(surfaceGap) <= band);

if isempty(candidateIdx)
    [~, idx] = min(abs(surfaceGap));
    lo = max(1, idx - 12);
    hi = min(length(tube.s), idx + 12);
    candidateIdx = lo:hi;
else
    lo = max(1, min(candidateIdx) - 8);
    hi = min(length(tube.s), max(candidateIdx) + 8);
    candidateIdx = lo:hi;
end
end


function y = solveProjectedForceQuadratic(G, rhs, y0, cfg)
m = cfg.forceSensor.numFrictionDirs;
y = projectForceVariables(y0, cfg);
L = norm(G, 2);
step = 1 / max(L, eps);

for k = 1:120
    yPrev = y;
    y = y - step * (G * y - rhs);
    y = projectForceVariables(y, cfg);
    if norm(y - yPrev) <= 1e-10 * max(1, norm(yPrev))
        break;
    end
end

% Make sure the active friction direction is not split across opposite
% basis vectors after projection.
if m == 2
    netTangential = y(2) - y(3);
    y(2:3) = [max(netTangential, 0); max(-netTangential, 0)];
    y = projectForceVariables(y, cfg);
end
end


function y = projectForceVariables(y, cfg)
m = cfg.forceSensor.numFrictionDirs;
y(1) = max(0, y(1));
y(2:1 + m) = max(0, y(2:1 + m));
if forceBoundsEnabled(cfg)
    y(1) = min(y(1), cfg.forceSensor.forceBounds.normalForceN);
    y(2:1 + m) = min(y(2:1 + m), cfg.forceSensor.forceBounds.betaN);
    y(2 + m:end) = min(max(y(2 + m:end), -cfg.forceSensor.forceBounds.tipForceN), ...
        cfg.forceSensor.forceBounds.tipForceN);
end

sumBeta = sum(y(2:1 + m));
limit = cfg.frictionMu * y(1);
if sumBeta > limit && sumBeta > 0
    y(2:1 + m) = y(2:1 + m) * (limit / sumBeta);
end
end


function h = mapMeasurementModel(tube, x, cfg, prevShape)
if nargin < 4
    prevShape = [];
end
decoded = decodeMapState(x, tube, cfg, prevShape);
h = decoded.u(:, decoded.fbgIdx);
h = h(:);
h = [h; decoded.p1; decoded.n];
end


function decoded = decodeMapState(x, tube, cfg, prevShape)
if nargin < 4
    prevShape = [];
end
m = cfg.forceSensor.numFrictionDirs;
p1 = x(1:3);
eta = x(4:5);
s1 = min(max(x(6), tube.s(1)), tube.s(end));
fn = max(0, x(7));
beta = max(0, x(8:7 + m));
lambda = max(0, x(8 + m));
fe = x(9 + m:11 + m);

n = etaToNormal(eta);
D = frictionDirections(n, m);
contactForce = n * fn + D * beta;

[~, idx] = min(abs(tube.s - s1));
u = solveShapeFromStateForces(tube, s1, contactForce, fe, prevShape);
[~, R, p] = solveShape(tube.T_base, u, tube.s);
pcCenter = interpolateVectorByArc(tube.s, p, s1);
pc = pcCenter;

if isempty(cfg) || ~isfield(cfg, 'sensing')
    fbgIdx = 1:length(tube.s);
else
    fbgIdx = unique(round(linspace(1, length(tube.s), cfg.sensing.numFbgPoints)));
end

if isempty(prevShape)
    prevPcCenter = pcCenter;
elseif isstruct(prevShape) && isfield(prevShape, 'p')
    prevPcCenter = interpolateVectorByArc(tube.s, prevShape.p, s1);
elseif isstruct(prevShape)
    prevPcCenter = pcCenter;
else
    prevPcCenter = interpolateVectorByArc(tube.s, prevShape, s1);
end
gap = n' * (pc - p1);
v = (eye(3) - n * n') * (pcCenter - prevPcCenter);
wFriction = D' * v + lambda * ones(m, 1);
coneSlack = cfg.frictionMu * fn - sum(beta);

decoded = struct;
decoded.p1 = p1;
decoded.eta = eta;
decoded.n = n;
decoded.D = D;
decoded.s1 = s1;
decoded.idx = idx;
decoded.fn = fn;
decoded.beta = beta;
decoded.lambda = lambda;
decoded.tipForce = fe;
decoded.contactForce = contactForce;
decoded.totalForce = contactForce + fe;
decoded.u = u;
decoded.R = R;
decoded.p = p;
decoded.pc = pc;
decoded.pcCenter = pcCenter;
decoded.gap = gap;
decoded.vTangential = v;
decoded.frictionW = wFriction;
decoded.frictionConeSlack = coneSlack;
decoded.fbgIdx = fbgIdx;
end


function u = solveShapeFromStateForces(tube, contactS, contactForce, tipForce, prevShape)
K = getTubeK(tube);
invK = 1 ./ K;

if nargin >= 5 && isstruct(prevShape) && isfield(prevShape, 'R') && isfield(prevShape, 'p')
    Rlin = prevShape.R;
    plin = prevShape.p;
else
    [~, Rlin, plin] = solveShape(tube.T_base, tube.uhat, tube.s);
end

J = computeJacobian(Rlin, plin);
Jcontact = interpolateJacobianAtArc(J, tube.s, contactS);
tipRows = 3 * length(tube.s) - (2:-1:0);
m = Jcontact' * contactForce(:) + J(tipRows, :)' * tipForce(:);
u = reshape(invK .* m, 3, []) + tube.uhat;
end


function value = interpolateVectorByArc(s, values, queryS)
s = s(:);
queryS = min(max(queryS, s(1)), s(end));
if size(values, 2) ~= numel(s)
    error('interpolateVectorByArc expects values to be 3-by-numel(s).');
end
value = interp1(s, values', queryS, 'linear')';
end


function Jarc = interpolateJacobianAtArc(J, s, queryS)
s = s(:);
queryS = min(max(queryS, s(1)), s(end));
if queryS <= s(1)
    rows = 1:3;
    Jarc = J(rows, :);
    return;
end
if queryS >= s(end)
    rows = 3 * numel(s) - (2:-1:0);
    Jarc = J(rows, :);
    return;
end

idx = find(s <= queryS, 1, 'last');
idx = min(idx, numel(s) - 1);
alpha = (queryS - s(idx)) / max(s(idx + 1) - s(idx), eps);
rows0 = 3 * idx - (2:-1:0);
rows1 = 3 * (idx + 1) - (2:-1:0);
Jarc = (1 - alpha) * J(rows0, :) + alpha * J(rows1, :);
end


function x = projectMapState(x, tube, cfg)
m = cfg.forceSensor.numFrictionDirs;
x = real(x(:));
x(~isfinite(x)) = 0;
x(4) = atan2(sin(x(4)), cos(x(4)));
x(5) = min(max(x(5), -pi / 2 + 1e-4), pi / 2 - 1e-4);
x(6) = min(max(x(6), tube.s(1)), tube.s(end));
x(7) = max(0, x(7));
x(8:7 + m) = max(0, x(8:7 + m));
x(8 + m) = max(0, x(8 + m));
if forceBoundsEnabled(cfg)
    x(7) = min(x(7), cfg.forceSensor.forceBounds.normalForceN);
    x(8:7 + m) = min(x(8:7 + m), cfg.forceSensor.forceBounds.betaN);
    x(8 + m) = min(x(8 + m), cfg.forceSensor.forceBounds.lambda);
    x(9 + m:11 + m) = min(max(x(9 + m:11 + m), -cfg.forceSensor.forceBounds.tipForceN), ...
        cfg.forceSensor.forceBounds.tipForceN);
end

fn = x(7);
betaIdx = 8:7 + m;
beta = x(betaIdx);
sumBeta = sum(beta);
limit = cfg.frictionMu * fn;
if sumBeta > limit && sumBeta > 0
    x(betaIdx) = beta * (limit / sumBeta);
end
end


function eta = normalToEta(n)
n = n(:) / norm(n);
eta = [atan2(n(2), n(1)); asin(max(-1, min(1, n(3))))];
end


function n = etaToNormal(eta)
az = eta(1);
el = eta(2);
n = [cos(el) * cos(az); cos(el) * sin(az); sin(el)];
n = n / norm(n);
end


function D = frictionDirections(n, m)
n = n(:) / norm(n);
t1 = contactTangentFromNormal(n);
if m == 1
    D = t1;
elseif m == 2
    D = [t1, -t1];
else
    B = null(n');
    theta = linspace(0, 2 * pi, m + 1);
    theta = theta(1:end - 1);
    D = B * [cos(theta); sin(theta)];
end
end


function t = contactTangentFromNormal(n)
t = [1; 0; 0];
t = t - n * (n' * t);
if norm(t) < 1e-12
    B = null(n');
    t = B(:, 1);
end
t = t / norm(t);
end


function aloi = estimateForcesWithAloiBaseline(tube0, measurements, cfg)
fprintf('\nEstimating shape-only Aloi-style Gaussian baseline...\n');

nt = numel(measurements.betaMm);
forceResultant = zeros(3, nt);
centerMm = nan(1, nt);
sigmaMm = nan(1, nt);
tipSigmaMm = nan(1, nt);
cost = nan(1, nt);
componentResultants = zeros(3, 2, nt);
nodalFinal = [];

for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    u = measurements.u(:, :, it);
    [~, R, p] = solveShape(tube.T_base, u, tube.s);

    fit = fitAloiGaussianFromMeasuredShape(tube, u, R, p, cfg);
    forceResultant(:, it) = fit.forceResultant;
    centerMm(it) = fit.centerMm;
    sigmaMm(it) = fit.sigmaMm;
    tipSigmaMm(it) = fit.tipSigmaMm;
    componentResultants(:, :, it) = fit.componentResultants;
    cost(it) = fit.cost;
    if it == nt
        nodalFinal = fit.nodalForces;
    end
end

aloi = struct;
aloi.forceResultant = forceResultant;
aloi.totalForceResultant = forceResultant;
aloi.centerMm = centerMm;
aloi.sigmaMm = sigmaMm;
aloi.tipSigmaMm = tipSigmaMm;
aloi.componentResultants = componentResultants;
aloi.cost = cost;
aloi.finalNodalForces = nodalFinal;
end


function fit = fitAloiGaussianFromMeasuredShape(tube, u, R, p, cfg)
s = tube.s(:);
J = computeJacobian(R, p);
K = getTubeK(tube);
mMeasured = K .* (u(:) - tube.uhat(:));

centerCandidates = linspace(s(1), s(end), cfg.aloi.numCenterCandidates);
basisFields = localTransverseBasisFromShape(p);
ds = median(diff(s));

best = struct('cost', inf, 'centerMm', centerCandidates(1), ...
              'sigmaMm', cfg.aloi.sigmaCandidatesMm(1), ...
              'tipSigmaMm', cfg.aloi.tipSigmaCandidatesMm(1), ...
              'forceResultant', zeros(3, 1), ...
              'componentResultants', zeros(3, 2), ...
              'nodalForces', zeros(3, numel(s)));

for sigma = cfg.aloi.sigmaCandidatesMm
    for center = centerCandidates
        densityContact = gaussianDensityOnArc(s, center, sigma);
        for tipSigma = cfg.aloi.tipSigmaCandidatesMm
            densityTip = gaussianDensityOnArc(s, s(end), tipSigma);

            basisMatrix = zeros(numel(mMeasured), 2 * numel(basisFields));
            nodalBasis = cell(1, 2 * numel(basisFields));
            col = 0;
            for componentId = 1:2
                if componentId == 1
                    density = densityContact;
                else
                    density = densityTip;
                end
                weights = reshape(density' * ds, 1, []);
                for ib = 1:numel(basisFields)
                    col = col + 1;
                    nodalBasis{col} = basisFields{ib} .* weights;
                    basisMatrix(:, col) = J' * nodalBasis{col}(:);
                end
            end

            amp = (basisMatrix' * basisMatrix + cfg.aloi.ridge * eye(size(basisMatrix, 2))) \ ...
                (basisMatrix' * mMeasured);
            residual = basisMatrix * amp - mMeasured;
            currentCost = norm(residual) / max(1, norm(mMeasured));

            if currentCost < best.cost
                nodalForces = zeros(3, numel(s));
                componentResultants = zeros(3, 2);
                col = 0;
                for componentId = 1:2
                    componentNodal = zeros(3, numel(s));
                    for ib = 1:numel(basisFields)
                        col = col + 1;
                        componentNodal = componentNodal + nodalBasis{col} * amp(col);
                    end
                    nodalForces = nodalForces + componentNodal;
                    componentResultants(:, componentId) = sum(componentNodal, 2);
                end

                best.cost = currentCost;
                best.centerMm = center;
                best.sigmaMm = sigma;
                best.tipSigmaMm = tipSigma;
                best.forceResultant = sum(nodalForces, 2);
                best.componentResultants = componentResultants;
                best.nodalForces = nodalForces;
            end
        end
    end
end

fit = best;
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


function basisFields = localTransverseBasisFromShape(p)
ns = size(p, 2);
tangent = zeros(3, ns);
for i = 1:ns
    if i == 1
        v = p(:, 2) - p(:, 1);
    elseif i == ns
        v = p(:, ns) - p(:, ns - 1);
    else
        v = p(:, i + 1) - p(:, i - 1);
    end
    tangent(:, i) = v / max(norm(v), eps);
end

normal1 = [tangent(3, :); zeros(1, ns); -tangent(1, :)];
normal2 = repmat([0; 1; 0], 1, ns);
for i = 1:ns
    normal1(:, i) = normal1(:, i) / max(norm(normal1(:, i)), eps);
    normal2(:, i) = normal2(:, i) - tangent(:, i) * (tangent(:, i)' * normal2(:, i));
    normal2(:, i) = normal2(:, i) / max(norm(normal2(:, i)), eps);
end
basisFields = {normal1, normal2};
end


function metrics = summarizeExperiment(results)
trueContact = results.forward.contactForceResultant;
trueTip = results.forward.tipLoad;
trueTotal = results.forward.totalForceResultant;

oursContact = results.ours.contactForceResultant;
oursTip = results.ours.tipForce;
oursTotal = results.ours.totalForceResultant;
aloiTotal = results.aloi.totalForceResultant;

active = vecnorm(trueTotal, 2, 1) > 1e-8;
if ~any(active)
    active = true(1, size(trueTotal, 2));
end

oursContactErr = oursContact - trueContact;
oursTipErr = oursTip - trueTip;
oursTotalErr = oursTotal - trueTotal;
aloiTotalErr = aloiTotal - trueTotal;

metrics = struct;
metrics.ours.contactRmse = rmseByFrame(oursContactErr(:, active));
metrics.ours.tipRmse = rmseByFrame(oursTipErr(:, active));
metrics.ours.resultantRmse = rmseByFrame(oursTotalErr(:, active));
metrics.ours.totalRmse = metrics.ours.resultantRmse;
metrics.ours.totalMaxError = max(vecnorm(oursTotalErr(:, active), 2, 1));
metrics.ours.finalContactErrorNorm = norm(oursContactErr(:, end));
metrics.ours.finalTipErrorNorm = norm(oursTipErr(:, end));
metrics.ours.finalErrorNorm = norm(oursTotalErr(:, end));
metrics.ours.finalRelativeErrorPct = 100 * metrics.ours.finalErrorNorm / max(norm(trueTotal(:, end)), eps);
metrics.ours.finalEstimatedContactForce = oursContact(:, end);
metrics.ours.finalTrueContactForce = trueContact(:, end);
metrics.ours.finalEstimatedTipForce = oursTip(:, end);
metrics.ours.finalTrueTipForce = trueTip(:, end);
metrics.ours.finalEstimatedForce = oursTotal(:, end);
metrics.ours.finalTrueForce = trueTotal(:, end);
metrics.ours.finalNormalComplementarity = results.ours.normalComplementarity(end);
metrics.ours.finalFrictionComplementarity = results.ours.frictionComplementarity(end);
metrics.ours.finalConeComplementarity = results.ours.coneComplementarity(end);
metrics.ours.maxNormalComplementarity = max(results.ours.normalComplementarity(active));
metrics.ours.maxFrictionComplementarity = max(results.ours.frictionComplementarity(active));
metrics.ours.maxConeComplementarity = max(results.ours.coneComplementarity(active));
metrics.ours.finalMeasurementResidualNorm = results.ours.measurementResidualNorm(end);
metrics.ours.maxMeasurementResidualNorm = max(results.ours.measurementResidualNorm(active));

metrics.aloi.resultantRmse = rmseByFrame(aloiTotalErr(:, active));
metrics.aloi.finalErrorNorm = norm(aloiTotalErr(:, end));
metrics.aloi.finalRelativeErrorPct = 100 * metrics.aloi.finalErrorNorm / max(norm(trueTotal(:, end)), eps);
metrics.aloi.finalEstimatedForce = aloiTotal(:, end);
metrics.aloi.finalTrueForce = trueTotal(:, end);

metrics.forward.finalContactArcLengthMm = results.forward.contactArcLength(end);
metrics.forward.maxTangentialContactForce = max(abs(trueContact(1, :)));
metrics.forward.maxNormalContactForce = max(abs(trueContact(3, :)));
metrics.forward.finalTrueContactForce = trueContact(:, end);
metrics.forward.finalTrueTipForce = trueTip(:, end);
metrics.forward.finalTrueTotalForce = trueTotal(:, end);
end


function value = rmseByFrame(errorByFrame)
value = sqrt(mean(vecnorm(errorByFrame, 2, 1).^2));
end


function plotExperimentResults(results)
cfg = results.config;
colors = parula(8);
frameIds = unique(round(linspace(1, numel(results.forward.betaMm), 8)));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1250 760]);
subplot(2, 2, 1);
hold on; axis equal; grid on; box on;
for k = 1:numel(frameIds)
    it = frameIds(k);
    p = results.forward.p(:, :, it);
    plot(p(1, :), p(3, :), 'LineWidth', 1.5, 'Color', colors(k, :));
end
yline(cfg.wallDistanceMm, 'k--', 'True plane');
yline(mean(results.measurements.planeZMeasured), 'Color', [0.5 0.5 0.5], 'LineStyle', ':');
xlabel('x [mm]');
ylabel('z [mm]');
title('Forward shape trajectory');

subplot(2, 2, 2);
plot(results.forward.betaMm, results.forward.contactForceResultant(1, :), 'k-', 'LineWidth', 1.8); hold on;
plot(results.forward.betaMm, results.ours.contactForceResultant(1, :), 'r--', 'LineWidth', 1.5);
plot(results.forward.betaMm, results.forward.contactForceResultant(3, :), 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8);
plot(results.forward.betaMm, results.ours.contactForceResultant(3, :), 'b--', 'LineWidth', 1.5);
grid on; box on;
xlabel('Base insertion [mm]');
ylabel('Contact force [N]');
title('Contact force estimate');
legend({'True F_x', 'Ours F_x', 'True F_z', 'Ours F_z'}, 'Location', 'best');
axis tight;

subplot(2, 2, 3);
oursErr = vecnorm(results.ours.totalForceResultant - results.forward.totalForceResultant, 2, 1);
aloiErr = vecnorm(results.aloi.totalForceResultant - results.forward.totalForceResultant, 2, 1);
contactErr = vecnorm(results.ours.contactForceResultant - results.forward.contactForceResultant, 2, 1);
tipErr = vecnorm(results.ours.tipForce - results.forward.tipLoad, 2, 1);
plot(results.forward.betaMm, oursErr, 'LineWidth', 1.8); hold on;
plot(results.forward.betaMm, aloiErr, 'LineWidth', 1.8);
plot(results.forward.betaMm, contactErr, 'LineStyle', '--', 'LineWidth', 1.2);
plot(results.forward.betaMm, tipErr, 'LineStyle', '--', 'LineWidth', 1.2);
grid on; box on;
xlabel('Base insertion [mm]');
ylabel('Force error norm [N]');
title('Error along trajectory');
legend({'Ours total', 'Aloi total', 'Ours contact', 'Ours tip'}, 'Location', 'best');
axis tight;

subplot(2, 2, 4);
barData = [results.forward.totalForceResultant(:, end), ...
           results.ours.totalForceResultant(:, end), ...
           results.aloi.totalForceResultant(:, end)];
bar(barData');
grid on; box on;
set(gca, 'XTickLabel', {'True', 'Shape+env', 'Aloi'});
ylabel('Final resultant force [N]');
legend({'F_x', 'F_y', 'F_z'}, 'Location', 'best');
title('Final total-load comparison');

saveFigure(fig, fullfile(cfg.outputDir, 'rod_plane_force_sensing_overview.png'));

fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 520]);
hold on; axis equal; grid on; box on;
p = results.forward.p(:, :, end);
plot(p(1, :), p(3, :), 'k-', 'LineWidth', 2.2);
yline(cfg.wallDistanceMm, 'k--');
yline(results.measurements.planeZMeasured(end), 'Color', [0.5 0.5 0.5], 'LineStyle', ':');

scale = 1.6;
trueContact = results.forward.contactForceResultant(:, end);
oursContact = results.ours.contactForceResultant(:, end);
trueTip = results.forward.tipLoad(:, end);
oursTip = results.ours.tipForce(:, end);
aloiTotal = results.aloi.totalForceResultant(:, end);
idx = results.ours.contactIndex(end);
if isnan(idx)
    idx = round(size(p, 2) / 2);
end
contactOrigin = p(:, idx);
tipOrigin = p(:, end);
quiver(contactOrigin(1), contactOrigin(3), trueContact(1) * scale, trueContact(3) * scale, 0, 'Color', [0 0.45 0.75], 'LineWidth', 2.2);
quiver(contactOrigin(1), contactOrigin(3), oursContact(1) * scale, oursContact(3) * scale, 0, 'Color', [0.1 0.6 0.2], 'LineWidth', 2.0);
quiver(tipOrigin(1), tipOrigin(3), trueTip(1) * scale, trueTip(3) * scale, 0, 'Color', [0 0.45 0.75], 'LineStyle', '--', 'LineWidth', 2.0);
quiver(tipOrigin(1), tipOrigin(3), oursTip(1) * scale, oursTip(3) * scale, 0, 'Color', [0.1 0.6 0.2], 'LineStyle', '--', 'LineWidth', 1.8);
quiver(tipOrigin(1), tipOrigin(3), aloiTotal(1) * scale, aloiTotal(3) * scale, 0, 'Color', [0.85 0.2 0.1], 'LineWidth', 1.6);
xlabel('x [mm]');
ylabel('z [mm]');
title('Final shape, contact force, and tip load');
legend({'Rod shape', 'True plane', 'Measured plane', 'True contact', 'Estimated contact', 'True tip load', 'Estimated tip load', 'Aloi total'}, 'Location', 'best');
saveFigure(fig2, fullfile(cfg.outputDir, 'rod_plane_final_force_comparison.png'));
end


function writeTrajectoryCsv(results)
cfg = results.config;
nt = numel(results.forward.betaMm);
tip = squeeze(results.forward.p(:, end, :));

T = table;
T.frame = (1:nt)';
T.insertion_mm = results.forward.betaMm(:);
T.tip_x_mm = tip(1, :)';
T.tip_y_mm = tip(2, :)';
T.tip_z_mm = tip(3, :)';
T.plane_z_measured_mm = results.measurements.planeZMeasured(:);
T.true_contact_s_mm = results.forward.contactArcLength(:);
T.estimated_contact_s_mm = results.ours.contactArcLength(:);

T.true_contact_Fx_N = results.forward.contactForceResultant(1, :)';
T.true_contact_Fy_N = results.forward.contactForceResultant(2, :)';
T.true_contact_Fz_N = results.forward.contactForceResultant(3, :)';

T.ours_contact_Fx_N = results.ours.contactForceResultant(1, :)';
T.ours_contact_Fy_N = results.ours.contactForceResultant(2, :)';
T.ours_contact_Fz_N = results.ours.contactForceResultant(3, :)';

T.true_tip_Fx_N = results.forward.tipLoad(1, :)';
T.true_tip_Fy_N = results.forward.tipLoad(2, :)';
T.true_tip_Fz_N = results.forward.tipLoad(3, :)';

T.ours_tip_Fx_N = results.ours.tipForce(1, :)';
T.ours_tip_Fy_N = results.ours.tipForce(2, :)';
T.ours_tip_Fz_N = results.ours.tipForce(3, :)';

T.seed_contact_s_mm = results.ours.seedContactArcLength(:);
T.seed_contact_Fx_N = results.ours.seedContactForceResultant(1, :)';
T.seed_contact_Fy_N = results.ours.seedContactForceResultant(2, :)';
T.seed_contact_Fz_N = results.ours.seedContactForceResultant(3, :)';
T.seed_tip_Fx_N = results.ours.seedTipForce(1, :)';
T.seed_tip_Fy_N = results.ours.seedTipForce(2, :)';
T.seed_tip_Fz_N = results.ours.seedTipForce(3, :)';

T.true_total_Fx_N = results.forward.totalForceResultant(1, :)';
T.true_total_Fy_N = results.forward.totalForceResultant(2, :)';
T.true_total_Fz_N = results.forward.totalForceResultant(3, :)';

T.ours_total_Fx_N = results.ours.totalForceResultant(1, :)';
T.ours_total_Fy_N = results.ours.totalForceResultant(2, :)';
T.ours_total_Fz_N = results.ours.totalForceResultant(3, :)';

T.aloi_total_Fx_N = results.aloi.totalForceResultant(1, :)';
T.aloi_total_Fy_N = results.aloi.totalForceResultant(2, :)';
T.aloi_total_Fz_N = results.aloi.totalForceResultant(3, :)';

T.ours_contact_error_norm_N = vecnorm(results.ours.contactForceResultant - results.forward.contactForceResultant, 2, 1)';
T.ours_tip_error_norm_N = vecnorm(results.ours.tipForce - results.forward.tipLoad, 2, 1)';
T.ours_total_error_norm_N = vecnorm(results.ours.totalForceResultant - results.forward.totalForceResultant, 2, 1)';
T.aloi_total_error_norm_N = vecnorm(results.aloi.totalForceResultant - results.forward.totalForceResultant, 2, 1)';
T.ours_gap_mm = results.ours.gap(:);
T.ours_friction_cone_slack_N = results.ours.frictionConeSlack(:);
T.ours_normal_complementarity = results.ours.normalComplementarity(:);
T.ours_friction_complementarity = results.ours.frictionComplementarity(:);
T.ours_cone_complementarity = results.ours.coneComplementarity(:);
T.ours_measurement_residual_norm = results.ours.measurementResidualNorm(:);
T.seed_measurement_residual_norm = results.ours.seedMeasurementResidualNorm(:);
T.seed_normal_complementarity = results.ours.seedNormalComplementarity(:);
T.seed_friction_complementarity = results.ours.seedFrictionComplementarity(:);
T.seed_cone_complementarity = results.ours.seedConeComplementarity(:);
T.seed_merit = results.ours.seedMerit(:);
T.final_merit = results.ours.finalMerit(:);
T.initialization_name = results.ours.initializationName(:);
T.initialization_merit = results.ours.initializationMerit(:);
T.initialization_residual_norm = results.ours.initializationResidualNorm(:);

writetable(T, fullfile(cfg.outputDir, 'rod_plane_force_sensing_trajectory.csv'));
end


function writeSummary(results)
cfg = results.config;
path = fullfile(cfg.outputDir, 'rod_plane_force_sensing_summary.txt');
fid = fopen(path, 'w');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'Rod-plane force sensing validation\n');
fprintf(fid, '==================================\n\n');
fprintf(fid, 'Forward model: copied rod-plane setup with a local tip-load extension; original LCP-Continuum files are unchanged.\n');
fprintf(fid, 'Inverse input: sparse FBG-like curvature samples plus a measured plane point and plane normal.\n\n');

fprintf(fid, 'Rod length: %.1f mm\n', cfg.exposedLengthMm);
fprintf(fid, 'Target bend: %.1f deg\n', cfg.targetBendDeg);
fprintf(fid, 'Actual integrated precurvature: %.2f deg\n', results.setup.actualBendDeg);
fprintf(fid, 'True plane z: %.1f mm, measured plane bias: %.2f mm\n', ...
    cfg.wallDistanceMm, cfg.sensing.planeOffsetBiasMm);
fprintf(fid, 'Friction coefficient: %.2f\n', cfg.frictionMu);
fprintf(fid, 'Applied tip load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', cfg.tipLoadN);
fprintf(fid, 'Frames: %d, insertion: %.1f mm\n\n', cfg.numTimeSteps, cfg.betaMaxMm);

fprintf(fid, 'Final true contact force [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.forward.contactForceResultant(:, end));
fprintf(fid, 'Final estimated contact force [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.ours.contactForceResultant(:, end));
fprintf(fid, 'Final true tip load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.forward.tipLoad(:, end));
fprintf(fid, 'Final estimated tip load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.ours.tipForce(:, end));
fprintf(fid, 'Final reduced-seed contact force [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.ours.seedContactForceResultant(:, end));
fprintf(fid, 'Final reduced-seed tip load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.ours.seedTipForce(:, end));
fprintf(fid, 'Final true total load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.forward.totalForceResultant(:, end));
fprintf(fid, 'Final estimated total load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.ours.totalForceResultant(:, end));
fprintf(fid, 'Final Aloi-style shape-only total load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.aloi.totalForceResultant(:, end));
fprintf(fid, 'Final Aloi-style body Gaussian resultant [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.aloi.componentResultants(:, 1, end));
fprintf(fid, 'Final Aloi-style tip Gaussian resultant [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.aloi.componentResultants(:, 2, end));
fprintf(fid, 'Final Aloi-style center/sigma/tip-sigma: %.3f / %.3f / %.3f mm\n', ...
    results.aloi.centerMm(end), results.aloi.sigmaMm(end), results.aloi.tipSigmaMm(end));
fprintf(fid, 'Final Aloi-style normalized shape-fit residual: %.6g\n\n', ...
    results.aloi.cost(end));

fprintf(fid, 'Shape+environment contact-force RMSE: %.6g N\n', results.metrics.ours.contactRmse);
fprintf(fid, 'Shape+environment tip-load RMSE: %.6g N\n', results.metrics.ours.tipRmse);
fprintf(fid, 'Shape+environment total-load RMSE: %.6g N\n', results.metrics.ours.resultantRmse);
fprintf(fid, 'Shape+environment final total-load relative error: %.4f %%\n', results.metrics.ours.finalRelativeErrorPct);
fprintf(fid, 'Final normal complementarity |g*f_n|: %.6g\n', results.metrics.ours.finalNormalComplementarity);
fprintf(fid, 'Final friction complementarity ||(D^T v + lambda e).*beta||: %.6g\n', results.metrics.ours.finalFrictionComplementarity);
fprintf(fid, 'Final cone complementarity |(mu*f_n - e^T beta)*lambda|: %.6g\n', results.metrics.ours.finalConeComplementarity);
fprintf(fid, 'Final normalized measurement residual norm: %.6g\n', results.metrics.ours.finalMeasurementResidualNorm);
fprintf(fid, 'Max normalized measurement residual norm: %.6g\n', results.metrics.ours.maxMeasurementResidualNorm);
fprintf(fid, 'Final reduced-seed normalized measurement residual norm: %.6g\n', ...
    results.ours.seedMeasurementResidualNorm(end));
fprintf(fid, 'Final reduced-seed MAP merit: %.6g\n', results.ours.seedMerit(end));
fprintf(fid, 'Final accepted MAP merit: %.6g\n', results.ours.finalMerit(end));
fprintf(fid, 'Final initialization selected: %s\n', results.ours.initializationName{end});
fprintf(fid, 'Aloi total-load RMSE: %.6g N\n', results.metrics.aloi.resultantRmse);
fprintf(fid, 'Aloi final total-load relative error: %.4f %%\n\n', results.metrics.aloi.finalRelativeErrorPct);

if isfield(results, 'mapCandidateDiagnostics') && ...
        isfield(results.mapCandidateDiagnostics, 'merit') && ...
        ~isempty(results.mapCandidateDiagnostics.merit)
    diag = results.mapCandidateDiagnostics;
    finalFrame = size(diag.merit, 2);
    fprintf(fid, 'Final MAP candidate diagnostics (diagnostic only; truth is not used by the estimator):\n');
    for j = 1:numel(diag.candidateNames)
        fprintf(fid, '  %-28s merit=%10.6g, meas_res=%10.6g, eq=%10.6g, ineq=%10.6g, minW=%10.6g, fn=%10.6g, s=%10.6g mm\n', ...
            diag.candidateNames{j}, diag.merit(j, finalFrame), ...
            diag.measurementResidualNorm(j, finalFrame), ...
            diag.maxEqualityResidual(j, finalFrame), ...
            diag.maxInequalityViolation(j, finalFrame), ...
            diag.minFrictionW(j, finalFrame), diag.normalForce(j, finalFrame), ...
            diag.contactArcLength(j, finalFrame));
    end
    fprintf(fid, '\n');
end

if results.metrics.ours.finalRelativeErrorPct > cfg.forceSensor.warningRelativeErrorPct || ...
        results.metrics.ours.finalMeasurementResidualNorm > cfg.forceSensor.warningMeasurementResidualNorm
    fprintf(fid, 'WARNING: inverse estimate failed numerical sanity checks. Do not use this run as a valid result without debugging.\n\n');
end

fprintf(fid, 'Implementation note:\n');
fprintf(fid, 'Solver: %s, friction directions m=%d, force bounds enabled: %d.\n', ...
    results.config.forceSensor.solver, results.config.forceSensor.numFrictionDirs, ...
    forceBoundsEnabled(results.config));
fprintf(fid, 'The inverse estimate does not use the forward solver contact force or contact index. ');
fprintf(fid, 'It solves the iterated constrained EKF/MAP update in Formulation.pdf eqs. (19)-(29) ');
fprintf(fid, 'with state x=[p1; eta1; s1; f1n; beta1; lambda1; fe], random-walk prior covariance, ');
fprintf(fid, 'the nonlinear Cosserat forward map [p(s;x),u(s;x)]=F(s1,f1,fe), finite-difference ');
fprintf(fid, 'linearization of h(x), posterior covariance update, and the normal/friction/cone ');
fprintf(fid, 'complementarity constraints. The plane gap follows Formulation.pdf eqs. (5)-(6), ');
fprintf(fid, 'pc=p(s1;x), which is also the centerline point used by the copied LCP rod-plane contact code. ');
fprintf(fid, 'Aloi is used only as a shape-only Gaussian total-load baseline and does not ');
fprintf(fid, 'receive the plane/friction constraints.\n');
end


function [resultant, meanArcLength] = summarizeContacts(contacts, s)
resultant = zeros(3, 1);
meanArcLength = nan;
if isempty(contacts)
    return;
end

forces = reshape([contacts.force], 3, []);
resultant = sum(forces, 2);
ids = [contacts.tube_point_id];
meanArcLength = mean(s(ids));
end


function ensureDir(pathStr)
if ~exist(pathStr, 'dir')
    mkdir(pathStr);
end
end


function base = mergeStructRecursive(base, overrides)
if isempty(overrides)
    return;
end
if ~isstruct(overrides)
    error('Configuration overrides must be a struct.');
end

names = fieldnames(overrides);
for i = 1:numel(names)
    name = names{i};
    if isfield(base, name) && isstruct(base.(name)) && isstruct(overrides.(name)) && ...
            isscalar(base.(name)) && isscalar(overrides.(name))
        base.(name) = mergeStructRecursive(base.(name), overrides.(name));
    else
        base.(name) = overrides.(name);
    end
end
end


function saveFigure(figHandle, filePath)
drawnow;
try
    exportgraphics(figHandle, filePath, 'Resolution', 200);
catch
    print(figHandle, filePath, '-dpng', '-r200');
end
fprintf('Saved figure: %s\n', filePath);
close(figHandle);
end
