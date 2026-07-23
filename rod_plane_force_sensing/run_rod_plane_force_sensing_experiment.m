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

packageDir = fileparts(mfilename('fullpath'));
rootDir = resolveRepositoryRoot(packageDir);
lcpDir = fullfile(rootDir, 'LCP-Continuum');
addpath(genpath(lcpDir));

cfg = defaultExperimentConfig(rootDir, quickMode);
cfg = mergeStructRecursive(cfg, overrides);
cfg = normalizeExperimentConfig(cfg);
ensureDir(cfg.outputDir);

rng(cfg.randomSeed, 'twister');

fprintf('\n=== Rod-plane force sensing validation ===\n');
fprintf('Scenario: %s\n', cfg.scenarioName);
fprintf('Output directory: %s\n', cfg.outputDir);

[tube0, obstaclesTruth, setupInfo] = makeRodPlaneScenario(cfg);
if cfg.scalePrecurvature
    fprintf('Rod length %.1f mm, target bend %.1f deg, actual bend %.1f deg\n', ...
        tube0.s(end), cfg.targetBendDeg, setupInfo.actualBendDeg);
else
    fprintf('Rod length %.1f mm, intrinsic precurvature retained, actual bend %.1f deg\n', ...
        tube0.s(end), setupInfo.actualBendDeg);
end
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
aloi = estimate_aloi_gaussian_baseline(tube0, measurements, cfg);

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
if cfg.video.enabled
    createForceSensingVideo(results);
    createAloiForceSensingVideo(results);
end

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
cfg.outputDir = fullfile(rootDir, 'force_outputs', ...
    'rod_plane_displacement_force_sensing');
cfg.randomSeed = 7;
cfg.scenarioName = 'validated_vertical_wall_body_contact';

% Forward rod-plane setup. LCP-Continuum uses millimeters and Newtons.
% The default case is the separated-contact validation used in the README.
cfg.exposedLengthMm = 150;
cfg.targetBendDeg = 180;
cfg.scalePrecurvature = false;
cfg.wallDistanceMm = 20;
cfg.planePointMm = [20; 0; 0];
cfg.planeNormal = [-1; 0; 0];
cfg.basePositionMm = [-120; 0; 0];
cfg.baseRotation = [0 0 1; 0 -1 0; 1 0 0];
cfg.frictionMu = 0.5;
cfg.initialLowFrictionMu = 0.0;
cfg.cornerRange = 0.5;
cfg.frictionConeEdges = 16;
cfg.betaMaxMm = 35;
cfg.numTimeSteps = 16;
cfg.tipLoadN = zeros(3, 1);
cfg.forward.motionMode = 'push-slide';
cfg.forward.pushDistanceMm = 45;
cfg.forward.slideDistanceMm = 1;
cfg.forward.pushFraction = 0.5;
cfg.forward.pushDirection = [1; 0; 0];
cfg.forward.slideDirection = [0; 0; 1];
cfg.forward.maxPushStepMm = 0.1;
cfg.forward.maxSlideStepMm = 0.02;
cfg.forward.sourceRevision = 'Jia0Shen/LCP-Continuum@14806b3';

cfg.video.enabled = true;
cfg.video.frameRate = 10;
cfg.video.durationSeconds = 6;
cfg.video.renderFrameCount = [];
cfg.video.forceScaleMmPerN = 3.0;
cfg.video.fileName = 'rod_plane_displacement_force_prediction.mp4';
cfg.video.aloiFileName = 'rod_plane_displacement_aloi_prediction.mp4';

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
cfg.forceSensor.complementaritySolver = 'active-set';
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

% Aloi comparison: one Gaussian load term is sufficient for the single true
% contact in this scenario. The fit uses sparse positions only, matching the
% weighted shape-fit objective in Aloi et al. and force.m.
cfg.aloi.numCenterCandidates = 21;
cfg.aloi.sigmaCandidatesMm = [3, 6, 12, 22, 35];
cfg.aloi.amplitudeBoundN = 100;
cfg.aloi.positionStdMm = 0.2;
cfg.aloi.maxIterations = 35;
cfg.aloi.maxFunctionEvaluations = 300;
cfg.aloi.showProgress = true;

if quickMode
    cfg.outputDir = fullfile(rootDir, 'force_outputs', ...
        'rod_plane_displacement_force_sensing_smoke_tmp');
    cfg.numTimeSteps = 6;
    cfg.betaMaxMm = 5;
    cfg.sensing.numFbgPoints = 12;
    cfg.forceSensor.maxEkfIterations = 1;
    cfg.forceSensor.linearizedSolveMaxIter = 25;
    cfg.forceSensor.solver = 'projected';
    cfg.forceSensor.allowApproximateFallback = true;
    cfg.aloi.numCenterCandidates = 11;
    cfg.aloi.sigmaCandidatesMm = [6, 15, 30];
    cfg.aloi.maxIterations = 15;
    cfg.aloi.maxFunctionEvaluations = 120;
end
end


function cfg = normalizeExperimentConfig(cfg)
cfg.planeNormal = cfg.planeNormal(:) / max(norm(cfg.planeNormal), eps);
if ~isfield(cfg, 'planePointMm') || isempty(cfg.planePointMm)
    cfg.planePointMm = [0; 0; cfg.wallDistanceMm];
else
    cfg.planePointMm = cfg.planePointMm(:);
end
cfg.wallDistanceMm = cfg.planePointMm(3);
cfg.basePositionMm = cfg.basePositionMm(:);
cfg.tipLoadN = cfg.tipLoadN(:);
if isempty(cfg.forward.pushDirection)
    cfg.forward.pushDirection = -cfg.planeNormal;
end
cfg.forward.pushDirection = unitVector(cfg.forward.pushDirection, -cfg.planeNormal);
cfg.forward.slideDirection = cfg.forward.slideDirection(:);
cfg.forward.slideDirection = cfg.forward.slideDirection - ...
    cfg.planeNormal * (cfg.planeNormal' * cfg.forward.slideDirection);
cfg.forward.slideDirection = unitVector(cfg.forward.slideDirection, contactTangentFromNormal(cfg.planeNormal));
cfg.forceSensor.normalReference = cfg.planeNormal;
end


function [tube, obstacles, info] = makeRodPlaneScenario(cfg)
tube = CreatTube(cfg.exposedLengthMm);

baseBendRad = trapz(tube.s, sqrt(sum(tube.uhat(1:2, :).^2, 1)));
if cfg.scalePrecurvature
    targetBendRad = deg2rad(cfg.targetBendDeg);
    curvatureScale = targetBendRad / max(baseBendRad, eps);
    tube.uhat = tube.uhat * curvatureScale;
else
    curvatureScale = 1;
end

tube.T_base = [cfg.baseRotation, cfg.basePositionMm; zeros(1, 3), 1];
obstacles = {createPlane(cfg.planePointMm, cfg.planeNormal, cfg.initialLowFrictionMu)};

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
[baseTrajInternal, actuationInternalMm, phaseInternal, frictionMuInternal] = ...
    buildForwardBaseTrajectory(tube0, cfg);
sampleIdx = selectForwardOutputFrames(phaseInternal, cfg.numTimeSteps, ...
    cfg.forward.pushFraction);
baseTraj = baseTrajInternal(:, :, sampleIdx);
actuationMm = actuationInternalMm(sampleIdx);
phase = phaseInternal(sampleIdx);
frictionMu = frictionMuInternal(sampleIdx);
ntInternal = size(baseTrajInternal, 3);
nt = numel(sampleIdx);

fprintf(['\nForward displacement-aware LCP simulation (%d internal steps, ', ...
    '%d output frames)...\n'], ntInternal, nt);

ns = length(tube.s);
pTraj = zeros(3, ns, nt);
uTraj = zeros(3, ns, nt);
RTraj = zeros(3, 3, ns, nt);
forceResultant = zeros(3, nt);
contactCount = zeros(1, nt);
contactArcLength = nan(1, nt);
contactTraj = cell(1, nt);
previousStateTraj = cell(1, nt);

tube.T_base = baseTrajInternal(:, :, 1);
[~, R0, p0] = solveShape(tube.T_base, tube.uhat, tube.s);
state = struct('p', p0, 'R', R0, 'u', tube.uhat, 'T_base', tube.T_base);
contacts = [];

outputFrame = 0;
for internalFrame = 1:ntInternal
    previousState = state;
    tube.T_base = baseTrajInternal(:, :, internalFrame);
    obstacles{1}.mu = frictionMuInternal(internalFrame);
    [state, contacts] = getFrictionalContactShape3WithTipForce( ...
        tube, obstacles, state, contacts, cfg.cornerRange, param, cfg.tipLoadN);

    if outputFrame < nt && internalFrame == sampleIdx(outputFrame + 1)
        outputFrame = outputFrame + 1;
        pTraj(:, :, outputFrame) = state.p;
        uTraj(:, :, outputFrame) = state.u;
        RTraj(:, :, :, outputFrame) = state.R;
        [forceResultant(:, outputFrame), contactArcLength(outputFrame)] = ...
            summarizeContacts(contacts, tube.s);
        contactCount(outputFrame) = length(contacts);
        contactTraj{outputFrame} = contacts;
        previousStateTraj{outputFrame} = previousState;

        fprintf(['  output %3d/%3d (step %4d/%4d), phase=%-10s, mu=%.2f, ', ...
            'contacts=%d, s=%7.2f mm, resultant=[%8.3f %8.3f %8.3f] N\n'], ...
            outputFrame, nt, internalFrame, ntInternal, phase{outputFrame}, ...
            frictionMu(outputFrame), contactCount(outputFrame), ...
            contactArcLength(outputFrame), forceResultant(:, outputFrame));
    end
end

forward = struct;
forward.s = tube.s;
forward.betaMm = actuationMm;
forward.actuationMm = actuationMm;
forward.phase = phase;
forward.frictionMu = frictionMu;
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
forward.previousState = previousStateTraj;
forward.baseTraj = baseTraj;
forward.internalFrameCount = ntInternal;
forward.internalSampleIndices = sampleIdx;
forward.internalActuationMm = actuationInternalMm;
forward.internalPhase = phaseInternal;
forward.internalFrictionMu = frictionMuInternal;
forward.finalR = state.R;
forward.motionMode = cfg.forward.motionMode;
forward.sourceRevision = cfg.forward.sourceRevision;
end


function [baseTraj, actuationMm, phase, frictionMu] = buildForwardBaseTrajectory(tube, cfg)
switch lower(char(cfg.forward.motionMode))
    case 'push-slide'
        nPush = max(2, ceil(cfg.forward.pushDistanceMm / ...
            max(cfg.forward.maxPushStepMm, eps)) + 1);
        nSlide = max(1, ceil(cfg.forward.slideDistanceMm / ...
            max(cfg.forward.maxSlideStepMm, eps)));
        pushMm = linspace(0, cfg.forward.pushDistanceMm, nPush);
        slideMm = (1:nSlide) * (cfg.forward.slideDistanceMm / nSlide);
        nt = nPush + nSlide;
        baseTraj = repmat(tube.T_base, 1, 1, nt);
        phase = repmat({'push'}, 1, nt);
        frictionMu = cfg.initialLowFrictionMu * ones(1, nt);

        for it = 1:nPush
            baseTraj(1:3, 4, it) = cfg.basePositionMm + cfg.forward.pushDirection * pushMm(it);
        end
        for j = 1:nSlide
            it = nPush + j;
            baseTraj(1:3, 4, it) = cfg.basePositionMm + ...
                cfg.forward.pushDirection * cfg.forward.pushDistanceMm + ...
                cfg.forward.slideDirection * slideMm(j);
            phase{it} = 'slide';
            frictionMu(it) = cfg.frictionMu;
        end
        actuationMm = [pushMm, cfg.forward.pushDistanceMm + slideMm];

    otherwise
        nt = max(1, round(cfg.numTimeSteps));
        baseTraj = repmat(tube.T_base, 1, 1, nt);
        phase = repmat({'insertion'}, 1, nt);
        frictionMu = cfg.frictionMu * ones(1, nt);
        actuationMm = linspace(0, cfg.betaMaxMm, nt);
        for it = 1:nt
            baseTraj(1:3, 4, it) = cfg.basePositionMm + [actuationMm(it); 0; 0];
        end
        frictionMu(1) = cfg.initialLowFrictionMu;
end
end


function sampleIdx = selectForwardOutputFrames(phase, numOutputFrames, pushFraction)
numInternal = numel(phase);
numOutputFrames = min(numInternal, max(1, round(numOutputFrames)));
if numOutputFrames == numInternal
    sampleIdx = 1:numInternal;
    return;
end

pushIdx = find(strcmp(phase, 'push'));
slideIdx = find(strcmp(phase, 'slide'));
if isempty(pushIdx) || isempty(slideIdx)
    sampleIdx = unique(round(linspace(1, numInternal, numOutputFrames)), 'stable');
    return;
end

numPushOutput = min(numel(pushIdx), ...
    max(1, round((numOutputFrames - 1) * pushFraction) + 1));
numSlideOutput = min(numel(slideIdx), numOutputFrames - numPushOutput);
numPushOutput = numOutputFrames - numSlideOutput;

pushSample = pushIdx(round(linspace(1, numel(pushIdx), numPushOutput)));
slideSample = slideIdx(round(linspace(1, numel(slideIdx), numSlideOutput)));
sampleIdx = [pushSample, slideSample];
end


function [state, contacts] = getFrictionalContactShape3WithTipForce(tube, obstacles, statePrev, contactsPrev, cornerRange, param, tipForce)
% Local displacement-aware copy of Jia Shen's updated frictional contact
% solve, with the known tip-force term retained. Upstream files are not
% modified.

if nargin < 6 || isempty(param)
    d = 6;
else
    d = param.d;
end
if nargin < 7
    tipForce = zeros(3, 1);
end
tipForce = tipForce(:);

[pMove, RMove] = rigidlyMovePreviousState(statePrev, tube.T_base);
contactsPrev = detectAdditionalContactsFromShape( ...
    tube, pMove, obstacles, contactsPrev, cornerRange, d);

uPrev = statePrev.u;
J = computeJacobian(RMove, pMove);
K = getTubeK(tube);
invK = 1 ./ K;
ds = tube.s(end) - tube.s(end - 1);

tipRows = 3 * length(tube.s) - (2:-1:0);
Jtip = J(tipRows, :);
mTip = Jtip' * tipForce;

if isempty(contactsPrev)
    u = reshape(invK .* mTip, 3, []) + tube.uhat;
    contacts = contactsPrev;
    [~, R, p] = solveShape(tube.T_base, u, tube.s);
    state = struct('p', p, 'R', R, 'u', u, 'T_base', tube.T_base);
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
        plc = (diCur * pMove(:, itc(1)) + diPrev * pMove(:, itc(2))) / dsum;
    else
        itc3 = [3 * itc - 2; 3 * itc - 1; 3 * itc];
        pProj = obs.project(pt);
        plc = pMove(:, itc);
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
    B(:, :, ic) = stableTangentBasis(nCur);
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

[~, R, p] = solveShape(tube.T_base, u, tube.s);
state = struct('p', p, 'R', R, 'u', u, 'T_base', tube.T_base);

contacts = contactsPrev;
dp = A * fContact + q;
pContactsNew = pcVec + reshape(dp, 3, []);

icToRemove = [];
for ic = 1:nc
    if fn(ic) > 0
        % The force was solved with Jc at the pre-refresh contact index.
        % Keep that index as the force application point for this frame;
        % tube_point_id is refreshed below only for tracking the next frame.
        contacts(ic).applied_tube_point_id = contactsPrev(ic).tube_point_id;
        contacts(ic) = refreshContactGeometry(contacts(ic), tube, obstacles, p, cornerRange);
        contacts(ic).force = fContact(3 * ic - 2:3 * ic);
        if any(~isfinite(contacts(ic).point))
            contacts(ic).point = pContactsNew(:, ic);
        end
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


function [pMove, RMove] = rigidlyMovePreviousState(statePrev, currentBase)
deltaT = currentBase / statePrev.T_base;
p4 = deltaT * [statePrev.p; ones(1, size(statePrev.p, 2))];
pMove = p4(1:3, :);
deltaR = deltaT(1:3, 1:3);
RMove = pagemtimes(deltaR, statePrev.R);
end


function contacts = detectAdditionalContactsFromShape(tube, p, obstacles, contacts, cornerRange, d)
ds = tube.s(end) - tube.s(end - 1);
idTol = ceil(5 / max(ds, eps));

for io = 1:numel(obstacles)
    if obstacles{io}.cornerFlag
        cornerRangeI = cornerRange;
    else
        cornerRangeI = 0;
    end
    detected = obstacles{io}.detectContact(tube, p, cornerRangeI);
    for id = 1:numel(detected)
        exists = false;
        for ic = 1:numel(contacts)
            if contactIdDistance(contacts(ic).tube_point_id, detected(id).tube_point_id) <= idTol
                exists = true;
                break;
            end
        end
        if ~exists
            c = contactRecordFromDetection(detected(id), io, d);
            if isempty(contacts)
                contacts = c;
            else
                contacts(end + 1) = c;
            end
        end
    end
end
end


function c = contactRecordFromDetection(detected, obstacleId, d)
c = struct;
c.force = zeros(3, 1);
c.obstacle_id = obstacleId;
c.tube_point_id = detected.tube_point_id;
c.applied_tube_point_id = detected.tube_point_id;
c.point = detected.point;
c.normal = detected.normal;
c.type = detected.type;
c.tube_point = detected.tube_point;
c.penetrateDepth = detected.penetrateDepth;
c.normal_force = 0;
c.friction_beta = zeros(d, 1);
c.friction_lambda = 0;
c.friction_directions = zeros(3, d);
end


function contact = refreshContactGeometry(contact, tube, obstacles, p, cornerRange)
io = contact.obstacle_id;
if obstacles{io}.cornerFlag
    cornerRangeI = cornerRange;
else
    cornerRangeI = 0;
end
detected = obstacles{io}.detectContact(tube, p, cornerRangeI);
best = [];
bestDistance = inf;
for id = 1:numel(detected)
    if strcmp(contact.type, detected(id).type)
        distance = contactIdDistance(contact.tube_point_id, detected(id).tube_point_id);
        if distance < bestDistance
            best = id;
            bestDistance = distance;
        end
    end
end
if isempty(best) || bestDistance >= 5
    return;
end
contact.tube_point_id = detected(best).tube_point_id;
contact.point = detected(best).point;
contact.normal = detected(best).normal;
contact.type = detected(best).type;
contact.tube_point = detected(best).tube_point;
contact.penetrateDepth = detected(best).penetrateDepth;
end


function distance = contactIdDistance(id1, id2)
id1 = id1(:);
id2 = id2(:);
if numel(id1) ~= numel(id2)
    distance = inf;
else
    distance = norm(id1 - id2);
end
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
planePointMeasured = zeros(3, nt);

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
    offset = cfg.sensing.planeOffsetBiasMm + cfg.sensing.planeOffsetNoiseStdMm * randn();
    planePointMeasured(:, it) = cfg.planePointMm + cfg.planeNormal * offset;
end

measurements = struct;
measurements.fbgIdx = fbgIdx;
measurements.sFbg = s(fbgIdx);
measurements.uSparse = uSparse;
measurements.pSparse = pSparse;
measurements.u = uMeasured;
measurements.p = pMeasured;
measurements.planePointMeasured = planePointMeasured;
measurements.planeZMeasured = planePointMeasured(3, :);
measurements.planeNormalMeasured = cfg.planeNormal(:) / norm(cfg.planeNormal);
measurements.baseTraj = forward.baseTraj;
measurements.betaMm = forward.betaMm;
measurements.frictionMu = forward.frictionMu;
measurements.previousShape = forward.previousState;
measurements.processStepCount = [forward.internalSampleIndices(1), ...
    diff(forward.internalSampleIndices)];
measurements.description = ['Sparse curvature/position samples plus a measured plane offset. ', ...
    'The dense curvature field is interpolated only from the sparse FBG samples. ', ...
    'Each frame retains the immediately preceding internal forward state for Eq. (7).'];
end


function cfgFrame = configForMeasurementFrame(cfg, measurements, it)
cfgFrame = cfg;
if isfield(measurements, 'frictionMu') && numel(measurements.frictionMu) >= it
    cfgFrame.frictionMu = measurements.frictionMu(it);
end
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

for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    cfgFrame = configForMeasurementFrame(cfg, measurements, it);
    prevShape = measurements.previousShape{it};
    contacts = forward.contacts{it};
    planePoint = measurements.planePointMeasured(:, it);
    n = measurements.planeNormalMeasured(:) / norm(measurements.planeNormalMeasured);
    eta = normalToEta(n, cfgFrame);

    if isempty(contacts)
        s1 = tube.s(end);
        contactForce = zeros(3, 1);
        ic = [];
    else
        contactForces = reshape([contacts.force], 3, []);
        [~, ic] = max(vecnorm(contactForces, 2, 1));
        contactForce = contacts(ic).force(:);
        s1 = contactAppliedArcLength(contacts(ic), tube.s);
    end

    [fn, beta, lambda, reconResidual] = contactVariablesFromForwardContact(contacts, ic, contactForce, n, cfgFrame);
    x = [planePoint; eta; s1; fn; beta; lambda; forward.tipLoad(:, it)];
    x = projectMapState(x, tube, cfgFrame);

    % When beta is zero, lambda is an auxiliary variable. Choose its smallest
    % feasible value so D'v + lambda >= 0 in Eqs. (16) and (21).
    decoded = decodeMapState(x, tube, cfgFrame, prevShape);
    if sum(decoded.beta) <= 1e-12
        x(8 + cfgFrame.forceSensor.numFrictionDirs) = ...
            max(0, -min(decoded.D' * decoded.vTangential));
        x = projectMapState(x, tube, cfgFrame);
    end
    state(:, it) = x;

    z = measurementVectorForFrame(measurements, it);
    Rdiag = measurementStdVector(measurements, cfgFrame) .^ 2;
    h = mapMeasurementModel(tube, x, cfgFrame, prevShape);
    [c, ceq] = fullMapConstraints(tube, x, cfgFrame, prevShape);
    decoded = decodeMapState(x, tube, cfgFrame, prevShape);

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
diagnostics.enabled = false;
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
diagnostics.enabled = true;

for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    cfgFrame = configForMeasurementFrame(cfg, measurements, it);
    prevShape = measurements.previousShape{it};
    z = measurementVectorForFrame(measurements, it);
    Rdiag = measurementStdVector(measurements, cfgFrame) .^ 2;
    xInit = initializeMapState(tube, measurements, it, cfgFrame, []);

    if it == 1
        xPrior = initialMapPriorState(xInit, cfgFrame);
        Pminus = diag(stateStdVector(cfgFrame) .^ 2);
    else
        xPrior = truthConsistency.state(:, it - 1);
        Pminus = measurements.processStepCount(it) * ...
            diag(processStdVector(cfgFrame) .^ 2);
    end

    seed = solveReducedShapeKnownMapUpdate(tube, xInit, xPrior, z, cfgFrame, measurements.u(:, :, it));
    xZeroForce = xInit(:);
    xZeroForce(7:end) = 0;

    candidates = [truthConsistency.state(:, it), ...
        xInit(:), ...
        seed.x(:), ...
        projectFullComplementarity(seed.x(:), tube, cfgFrame, prevShape), ...
        xZeroForce(:)];

    for jc = 1:nc
        xCandidate = projectMapState(candidates(:, jc), tube, cfgFrame);
        h = mapMeasurementModel(tube, xCandidate, cfgFrame, prevShape);
        [c, ceq] = fullMapConstraints(tube, xCandidate, cfgFrame, prevShape);
        decoded = decodeMapState(xCandidate, tube, cfgFrame, prevShape);
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

end
end


function printFinalMapCandidateDiagnostics(diagnostics)
if ~isfield(diagnostics, 'enabled') || ~diagnostics.enabled || ...
        ~isfield(diagnostics, 'merit') || isempty(diagnostics.merit)
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
estimatedP = zeros(3, numel(tube0.s), nt);
estimatedU = zeros(3, numel(tube0.s), nt);
estimatedNormalDirection = zeros(3, nt);
shapeRmseMm = nan(1, nt);
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
complementarityModeName = cell(1, nt);
initializationMerit = nan(1, nt);
initializationResidualNorm = nan(1, nt);
priorMean = [];
priorCovariance = [];
inverseTimer = tic;
showProgress = isfield(cfg.forceSensor, 'showProgress') && cfg.forceSensor.showProgress;

for it = 1:nt
    frameTimer = tic;
    cfgFrame = configForMeasurementFrame(cfg, measurements, it);
    prevShape = measurements.previousShape{it};
    if showProgress
        fprintf('  inverse frame %3d/%3d started, insertion %.2f mm, mu %.2f, elapsed %.1f min\n', ...
            it, nt, measurements.betaMm(it), cfgFrame.frictionMu, toc(inverseTimer) / 60);
    end
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    z = measurementVectorForFrame(measurements, it);
    Rdiag = measurementStdVector(measurements, cfgFrame) .^ 2;
    xInit = initializeMapState(tube, measurements, it, cfgFrame, priorMean);
    if isempty(priorMean)
        xPrior = initialMapPriorState(xInit, cfgFrame);
        Pminus = diag(stateStdVector(cfgFrame) .^ 2);
    else
        xPrior = priorMean;
        Qframe = measurements.processStepCount(it) * ...
            diag(processStdVector(cfgFrame) .^ 2);
        Pminus = priorCovariance + Qframe;
    end

    progress = struct('frame', it, 'numFrames', nt);
    est = solveConstrainedMapUpdate(tube, xInit, xPrior, Pminus, z, Rdiag, cfgFrame, prevShape, measurements.u(:, :, it), progress);
    state(:, it) = est.x;
    posteriorCovariance(:, :, it) = est.Pplus;
    priorMean = est.x;
    priorCovariance = est.Pplus;

    seedDecoded = decodeMapState(est.seed, tube, cfgFrame, prevShape);
    seedContactForce(:, it) = seedDecoded.contactForce;
    seedTipForce(:, it) = seedDecoded.tipForce;
    seedTotalForce(:, it) = seedDecoded.totalForce;
    seedContactArcLength(it) = seedDecoded.s1;
    seedMeasurementNorm(it) = norm((z - mapMeasurementModel(tube, est.seed, cfgFrame, prevShape)) ./ sqrt(Rdiag));
    seedNormalComplementarity(it) = abs(seedDecoded.gap * seedDecoded.fn);
    seedFrictionComplementarity(it) = norm(seedDecoded.frictionW(:) .* seedDecoded.beta(:));
    seedConeComplementarity(it) = abs(seedDecoded.frictionConeSlack * seedDecoded.lambda);
    seedMerit(it) = est.seedMerit;
    finalMerit(it) = est.nonlinearMerit;
    initializationName{it} = est.initInfo.name;
    complementarityModeName{it} = est.complementarityMode.name;
    initializationMerit(it) = est.initInfo.cost;
    initializationResidualNorm(it) = est.initInfo.residualNorm;

    decoded = decodeMapState(est.x, tube, cfgFrame, prevShape);
    contactForce(:, it) = decoded.contactForce;
    tipForce(:, it) = decoded.tipForce;
    totalForce(:, it) = decoded.contactForce + decoded.tipForce;
    contactArcLength(it) = tube.s(est.contactIdx);
    contactIndex(it) = est.contactIdx;
    contactPoint(:, it) = est.contactPoint;
    estimatedP(:, :, it) = decoded.p;
    estimatedU(:, :, it) = decoded.u;
    estimatedNormalDirection(:, it) = decoded.n;
    shapeError = decoded.p - measurements.p(:, :, it);
    shapeRmseMm(it) = sqrt(mean(sum(shapeError .^ 2, 1)));
    gap(it) = est.surfaceGap;
    frictionSlack(it) = decoded.frictionConeSlack;
    normalComplementarity(it) = abs(decoded.gap * decoded.fn);
    frictionComplementarity(it) = norm(decoded.frictionW(:) .* decoded.beta(:));
    coneComplementarity(it) = abs(decoded.frictionConeSlack * decoded.lambda);
    measurementNorm(it) = norm(est.measurementResidual ./ sqrt(Rdiag));
    cost(it) = est.cost;
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
ours.p = estimatedP;
ours.u = estimatedU;
ours.planeNormal = estimatedNormalDirection;
ours.shapeRmseMm = shapeRmseMm;
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
ours.complementarityMode = complementarityModeName;
ours.initializationMerit = initializationMerit;
ours.initializationResidualNorm = initializationResidualNorm;
ours.gap = gap;
ours.frictionConeSlack = frictionSlack;
ours.normalComplementarity = normalComplementarity;
ours.frictionComplementarity = frictionComplementarity;
ours.coneComplementarity = coneComplementarity;
ours.measurementResidualNorm = measurementNorm;
ours.cost = cost;
ours.frictionMu = measurements.frictionMu;
ours.processStepCount = measurements.processStepCount;
ours.stateDescription = '[p1(3); eta1(2); s1; f1n; beta1(m); lambda1; fe(3)]';
ours.methodDescription = 'Full iterated constrained EKF/MAP from Formulation.pdf eqs. (19)-(29).';
end


function nx = formulationStateSize(cfg)
nx = 3 + 2 + 1 + 1 + cfg.forceSensor.numFrictionDirs + 1 + 3;
end


function z = measurementVectorForFrame(measurements, it)
z = measurements.uSparse(:, :, it);
z = z(:);
planePoint = measurements.planePointMeasured(:, it);
normalVector = measurements.planeNormalMeasured(:) / norm(measurements.planeNormalMeasured);
z = [z; planePoint; normalVector];
end


function shape = measuredShapeForFrame(tube, measuredU, s1)
[~, R, p] = solveShape(tube.T_base, measuredU, tube.s);
shape = struct('p', p, 'R', R, 'u', measuredU, 's1', s1, 'T_base', tube.T_base);
end


function x = initializeMapState(tube, measurements, it, cfg, priorMean)
u = measurements.u(:, :, it);
p = measurements.p(:, :, it);
normal = measurements.planeNormalMeasured;
eta = normalToEta(normal, cfg);

planePoint = measurements.planePointMeasured(:, it);
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

x = [planePoint; eta; s1; fn; beta; lambda; feSeed(:)];
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
modeReference = xPrior;
if seed.x(7) > modeReference(7)
    modeReference = seed.x;
end
complementarityMode = selectComplementarityMode( ...
    tube, modeReference, cfg, prevShape, measuredU, xInit);
[x, initInfo] = chooseInitialMapLinearization(tube, xInit, xPrior, seed.x, ...
    xPrior, Pminus, z, Rdiag, cfg, prevShape, complementarityMode);
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
    xProposal = solveLinearizedConstrainedMapSubproblem(tube, xLinearization, ...
        xPrior, Pminus, z, Rdiag, h0, H, cfg, prevShape, progress, ...
        startCandidates, complementarityMode);
    [xNext, acceptedAlpha, acceptedCost] = acceptDampedMapStep(tube, ...
        xLinearization, xProposal, xPrior, Pminus, z, Rdiag, cfg, ...
        prevShape, complementarityMode);
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
est.complementarityMode = complementarityMode;
est.nonlinearMerit = nonlinearMapMerit(tube, x, xPrior, Pminus, z, Rdiag, cfg, prevShape);
est.seedMerit = nonlinearMapMerit(tube, seed.x, xPrior, Pminus, z, Rdiag, cfg, prevShape);
end


function [xBest, info] = chooseInitialMapLinearization(tube, xInit, xPriorCandidate, xSeed, ...
    xPrior, Pminus, z, Rdiag, cfg, prevShape, complementarityMode)
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
    if nargin >= 11 && ~isempty(complementarityMode) && ...
            ~strcmp(complementarityMode.name, 'product-mpcc')
        xCandidate = projectToComplementarityMode( ...
            xCandidate, tube, cfg, prevShape, complementarityMode);
    end
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
n = etaToNormal(x(4:5), cfg);
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


function x = solveLinearizedConstrainedMapSubproblem(tube, xLinearization, xPrior, Pminus, z, Rdiag, h0, H, cfg, prevShape, progress, startCandidates, complementarityMode)
if nargin < 11
    progress = struct('frame', nan, 'numFrames', nan, 'ekfIter', nan);
end
if nargin < 12 || isempty(startCandidates)
    startCandidates = xLinearization(:);
end
if nargin < 13 || isempty(complementarityMode)
    complementarityMode = selectComplementarityMode( ...
        tube, xLinearization, cfg, prevShape);
end
PInv = pinvSym(Pminus);
objective = @(xc) linearizedMapCost(xc(:), xLinearization, xPrior, PInv, z, Rdiag, h0, H);
if isfield(cfg.forceSensor, 'complementaritySolver') && ...
        strcmpi(cfg.forceSensor.complementaritySolver, 'active-set')
    nonlcon = @(xc) activeSetMapConstraints( ...
        tube, xc(:), cfg, prevShape, complementarityMode);
else
    complementarityMode = struct('name', 'product-mpcc');
    nonlcon = @(xc) fullMapConstraints(tube, xc(:), cfg, prevShape);
end
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
            fprintf(['      frame %3d/%3d EKF iter %d: solving constrained MAP ', ...
                'with fmincon (%s)\n'], ...
                progress.frame, progress.numFrames, progress.ekfIter, ...
                complementarityMode.name);
        end
        if ~isfield(cfg.forceSensor, 'useMultiStart') || ~cfg.forceSensor.useMultiStart
            startCandidates = xLinearization(:);
        end
        bestX = [];
        bestValue = inf;
        bestStart = 1;
        for istart = 1:size(startCandidates, 2)
            x0 = projectMapState(startCandidates(:, istart), tube, cfg);
            if ~strcmp(complementarityMode.name, 'product-mpcc')
                x0 = projectToComplementarityMode( ...
                    x0, tube, cfg, prevShape, complementarityMode);
            end
            [xCandidate, ~, exitflag, output] = fmincon( ...
                objective, x0, [], [], [], [], lb, ub, nonlcon, opts);
            xCandidate = projectMapState(xCandidate, tube, cfg);
            if ~strcmp(complementarityMode.name, 'product-mpcc')
                xCandidate = projectToComplementarityMode( ...
                    xCandidate, tube, cfg, prevShape, complementarityMode);
            end
            value = constrainedSubproblemMerit(xCandidate, objective, nonlcon, cfg);
            if isfield(cfg.forceSensor, 'showProgress') && cfg.forceSensor.showProgress
                constraintViolation = nan;
                if isstruct(output) && isfield(output, 'constrviolation')
                    constraintViolation = output.constrviolation;
                end
                fprintf(['        fmincon exit %d, iterations %d, ', ...
                    'constraint violation %.3g\n'], ...
                    exitflag, output.iterations, constraintViolation);
            end
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


function [xAccepted, acceptedAlpha, acceptedCost] = acceptDampedMapStep(tube, xCurrent, xProposal, xPrior, Pminus, z, Rdiag, cfg, prevShape, complementarityMode)
alphas = cfg.forceSensor.dampingScales;
if isempty(alphas)
    alphas = [1.0, 0.5, 0.25, 0.1, 0.0];
end

bestCost = inf;
bestX = projectMapState(xCurrent, tube, cfg);
if nargin >= 10 && ~isempty(complementarityMode) && ...
        ~strcmp(complementarityMode.name, 'product-mpcc')
    bestX = projectToComplementarityMode( ...
        bestX, tube, cfg, prevShape, complementarityMode);
end
bestAlpha = 0;
for ia = 1:numel(alphas)
    alpha = alphas(ia);
    xCandidate = xCurrent + alpha * (xProposal - xCurrent);
    xCandidate = projectMapState(xCandidate, tube, cfg);
    if nargin >= 10 && ~isempty(complementarityMode) && ...
            ~strcmp(complementarityMode.name, 'product-mpcc')
        xCandidate = projectToComplementarityMode( ...
            xCandidate, tube, cfg, prevShape, complementarityMode);
    end
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


function mode = selectComplementarityMode(tube, x, cfg, prevShape, measuredU, geometryState)
decoded = decodeMapState(x, tube, cfg, prevShape);
forceTol = 5e-2;
slipTolMm = 5e-2;
vForMode = decoded.vTangential;
DForMode = decoded.D;

% The force seed can reproduce the measured curvature poorly before the MAP
% update. Determine stick/slip from the observed shape used by Eq. (7), while
% retaining the predicted force only to decide whether contact is active.
if nargin >= 5 && ~isempty(measuredU) && ~isempty(prevShape)
    if nargin < 6 || isempty(geometryState)
        geometryState = x;
    end
    sObserved = min(max(geometryState(6), tube.s(1)), tube.s(end));
    nObserved = etaToNormal(geometryState(4:5), cfg);
    measuredShape = measuredShapeForFrame(tube, measuredU, sObserved);
    currentPoint = interpolateVectorByArc(tube.s, measuredShape.p, sObserved);
    previousPoint = interpolateVectorByArc(tube.s, prevShape.p, sObserved);
    vForMode = (eye(3) - nObserved * nObserved') * ...
        (currentPoint - previousPoint);
    DForMode = frictionDirections(nObserved, cfg.forceSensor.numFrictionDirs);
end

if decoded.fn <= forceTol
    mode = struct('name', 'no-contact', 'activeDirection', []);
elseif cfg.frictionMu <= eps
    mode = struct('name', 'frictionless-contact', 'activeDirection', []);
elseif norm(vForMode) <= slipTolMm
    mode = struct('name', 'sticking-contact', 'activeDirection', []);
else
    [~, activeDirection] = min(DForMode' * vForMode);
    mode = struct('name', 'sliding-contact', ...
        'activeDirection', activeDirection);
end
end


function x = projectToComplementarityMode(x, tube, cfg, prevShape, mode)
x = projectMapState(x, tube, cfg);
m = cfg.forceSensor.numFrictionDirs;
betaIdx = 8:7 + m;
lambdaIdx = 8 + m;
decoded = decodeMapState(x, tube, cfg, prevShape);

switch mode.name
    case 'no-contact'
        x(7) = 0;
        x(betaIdx) = 0;

    case 'frictionless-contact'
        x(1:3) = x(1:3) + decoded.n * decoded.gap;
        x(betaIdx) = 0;

    case 'sticking-contact'
        x(1:3) = x(1:3) + decoded.n * decoded.gap;
        x(lambdaIdx) = 0;
        x = projectStickingKinematics(x, tube, cfg, prevShape);

    case 'sliding-contact'
        x(1:3) = x(1:3) + decoded.n * decoded.gap;
        x(betaIdx) = 0;
        x(betaIdx(mode.activeDirection)) = cfg.frictionMu * x(7);
end

decoded = decodeMapState(x, tube, cfg, prevShape);
if ~strcmp(mode.name, 'no-contact')
    x(1:3) = x(1:3) + decoded.n * decoded.gap;
    decoded = decodeMapState(x, tube, cfg, prevShape);
end
if any(strcmp(mode.name, {'no-contact', 'frictionless-contact', 'sliding-contact'}))
    x(lambdaIdx) = max(0, -min(decoded.D' * decoded.vTangential));
end
x = projectMapState(x, tube, cfg);
end


function x = projectStickingKinematics(x, tube, cfg, prevShape)
m = cfg.forceSensor.numFrictionDirs;
lambdaIdx = 8 + m;
variableIdx = [6:7 + m, 9 + m:11 + m];
scale = stateScaleVector(cfg);

for iter = 1:8
    x(lambdaIdx) = 0;
    decoded = decodeMapState(x, tube, cfg, prevShape);
    tangentBasis = stableTangentBasis(decoded.n);
    residual = tangentBasis' * decoded.vTangential;
    if norm(residual) < 1e-7
        break;
    end

    jacobian = zeros(2, numel(variableIdx));
    for j = 1:numel(variableIdx)
        idx = variableIdx(j);
        step = max(1e-6, 1e-4 * scale(idx));
        xp = x;
        xp(idx) = xp(idx) + step;
        xp = projectMapState(xp, tube, cfg);
        actualStep = xp(idx) - x(idx);
        if abs(actualStep) < eps
            continue;
        end
        decodedP = decodeMapState(xp, tube, cfg, prevShape);
        residualP = stableTangentBasis(decodedP.n)' * decodedP.vTangential;
        jacobian(:, j) = (residualP - residual) / actualStep;
    end

    normalMatrix = jacobian * jacobian' + 1e-10 * eye(2);
    delta = -jacobian' * (normalMatrix \ residual);
    scaledNorm = norm(delta ./ scale(variableIdx));
    if scaledNorm > 0.75
        delta = delta * (0.75 / scaledNorm);
    end
    x(variableIdx) = x(variableIdx) + delta;
    x = projectMapState(x, tube, cfg);
end

x(lambdaIdx) = 0;
decoded = decodeMapState(x, tube, cfg, prevShape);
x(1:3) = x(1:3) + decoded.n * decoded.gap;
x = projectMapState(x, tube, cfg);
end


function [c, ceq] = activeSetMapConstraints(tube, x, cfg, prevShape, mode)
decoded = decodeMapState(x, tube, cfg, prevShape);
m = cfg.forceSensor.numFrictionDirs;

switch mode.name
    case 'no-contact'
        c = [-decoded.gap; -decoded.frictionW(:); -decoded.frictionConeSlack];
        ceq = [decoded.fn; decoded.beta(:)];

    case 'frictionless-contact'
        c = [-decoded.frictionW(:); -decoded.frictionConeSlack];
        ceq = [decoded.gap; decoded.beta(:)];

    case 'sticking-contact'
        tangentBasis = stableTangentBasis(decoded.n);
        c = -decoded.frictionConeSlack;
        ceq = [decoded.gap; tangentBasis' * decoded.vTangential; decoded.lambda];

    case 'sliding-contact'
        active = mode.activeDirection;
        inactive = setdiff(1:m, active);
        c = -decoded.frictionW(:);
        ceq = [decoded.gap; decoded.frictionConeSlack; ...
            decoded.frictionW(active); decoded.beta(inactive)];

    otherwise
        [c, ceq] = fullMapConstraints(tube, x, cfg, prevShape);
end
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
lb(4) = -pi / 2 + 1e-4;
ub(4) = pi / 2 - 1e-4;
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

n = etaToNormal(eta, cfg);
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
    if isfield(prevShape, 'T_base')
        [plin, Rlin] = rigidlyMovePreviousState(prevShape, tube.T_base);
    else
        Rlin = prevShape.R;
        plin = prevShape.p;
    end
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
x(4:5) = min(max(x(4:5), -pi / 2 + 1e-4), pi / 2 - 1e-4);
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


function eta = normalToEta(n, cfg)
n = unitVector(n, [0; 0; 1]);
n0 = unitVector(cfg.forceSensor.normalReference, [0; 0; 1]);
B = stableTangentBasis(n0);
cosTheta = max(-1, min(1, n0' * n));
theta = acos(cosTheta);
if theta < 1e-10
    eta = zeros(2, 1);
    return;
end
tangentDirection = n - cosTheta * n0;
tangentDirection = tangentDirection / max(norm(tangentDirection), eps);
eta = theta * (B' * tangentDirection);
end


function n = etaToNormal(eta, cfg)
n0 = unitVector(cfg.forceSensor.normalReference, [0; 0; 1]);
B = stableTangentBasis(n0);
tangent = B * eta(:);
theta = norm(tangent);
if theta < 1e-10
    n = n0;
else
    n = cos(theta) * n0 + sin(theta) * tangent / theta;
    n = unitVector(n, n0);
end
end


function D = frictionDirections(n, m)
n = n(:) / norm(n);
t1 = contactTangentFromNormal(n);
if m == 1
    D = t1;
elseif m == 2
    D = [t1, -t1];
else
    B = stableTangentBasis(n);
    theta = linspace(0, 2 * pi, m + 1);
    theta = theta(1:end - 1);
    D = B * [cos(theta); sin(theta)];
end
end


function B = stableTangentBasis(n)
n = unitVector(n, [0; 0; 1]);
t1 = contactTangentFromNormal(n);
t2 = unitVector(cross(n, t1), [0; 1; 0]);
B = [t1, t2];
end


function value = unitVector(value, fallback)
value = value(:);
if norm(value) < 1e-12
    value = fallback(:);
end
value = value / max(norm(value), eps);
end


function t = contactTangentFromNormal(n)
n = unitVector(n, [0; 0; 1]);
[~, axisIdx] = min(abs(n));
referenceAxis = zeros(3, 1);
referenceAxis(axisIdx) = 1;
t = referenceAxis;
t = t - n * (n' * t);
t = t / norm(t);
end


function aloi = estimateForcesWithAloiBaseline(tube0, measurements, cfg)
fprintf('\nEstimating Aloi Gaussian position-fit baseline...\n');

nt = numel(measurements.betaMm);
forceResultant = zeros(3, nt);
centerMm = nan(1, nt);
sigmaMm = nan(1, nt);
cost = nan(1, nt);
shapeRmseMm = nan(1, nt);
parameters = nan(4, nt);
componentResultants = zeros(3, 1, nt);
nodalFinal = [];
previousTheta = [];

for it = 1:nt
    tube = tube0;
    tube.T_base = measurements.baseTraj(:, :, it);
    fit = fitAloiGaussianToSparsePositions(tube, ...
        measurements.pSparse(:, :, it), measurements.fbgIdx, cfg, previousTheta);
    forceResultant(:, it) = fit.forceResultant;
    centerMm(it) = fit.centerMm;
    sigmaMm(it) = fit.sigmaMm;
    shapeRmseMm(it) = fit.shapeRmseMm;
    parameters(:, it) = fit.theta;
    componentResultants(:, 1, it) = fit.forceResultant;
    cost(it) = fit.cost;
    previousTheta = fit.theta;
    if it == nt
        nodalFinal = fit.nodalForces;
    end
    if cfg.aloi.showProgress
        fprintf(['  Aloi frame %3d/%3d: s=%.2f mm, sigma=%.2f mm, ', ...
            'resultant=[%.3f %.3f %.3f] N, position RMSE=%.4f mm\n'], ...
            it, nt, fit.centerMm, fit.sigmaMm, fit.forceResultant, fit.shapeRmseMm);
    end
end

aloi = struct;
aloi.forceResultant = forceResultant;
aloi.totalForceResultant = forceResultant;
aloi.centerMm = centerMm;
aloi.sigmaMm = sigmaMm;
aloi.tipSigmaMm = nan(1, nt);
aloi.componentResultants = componentResultants;
aloi.cost = cost;
aloi.shapeRmseMm = shapeRmseMm;
aloi.parameters = parameters;
aloi.finalNodalForces = nodalFinal;
aloi.methodDescription = ['Single-Gaussian local-transverse load fitted to sparse positions ', ...
    'with the weighted nonlinear least-squares objective used by Aloi et al.; ', ...
    'no plane or friction measurements are used.'];
end


function fit = fitAloiGaussianToSparsePositions(tube, targetPositions, measurementIdx, cfg, previousTheta)
s = tube.s(:);
[~, Rreference, preference] = solveShape(tube.T_base, tube.uhat, tube.s);
Jreference = computeJacobian(Rreference, preference);
model = struct;
model.s = s;
model.tube = tube;
model.referenceP = preference;
model.referenceR = Rreference;
model.J = Jreference;
model.invK = 1 ./ getTubeK(tube);
model.integrationWeightsMm = trapezoidalIntegrationWeights(s);
model.measurementIdx = measurementIdx(:)';
model.targetPositions = targetPositions;
model.positionStdMm = cfg.aloi.positionStdMm;

amplitudeBound = cfg.aloi.amplitudeBoundN;
sigmaMin = min(cfg.aloi.sigmaCandidatesMm);
sigmaMax = max(cfg.aloi.sigmaCandidatesMm);
lb = [-amplitudeBound; -amplitudeBound; s(1); sigmaMin];
ub = [ amplitudeBound;  amplitudeBound; s(end); sigmaMax];
centerCandidates = linspace(s(1), s(end), cfg.aloi.numCenterCandidates);
targetDelta = targetPositions - preference(:, model.measurementIdx);
targetDelta = targetDelta(:);
theta0 = [0; 0; centerCandidates(1); cfg.aloi.sigmaCandidatesMm(1)];
bestInitialCost = inf;

for sigma = cfg.aloi.sigmaCandidatesMm
    for center = centerCandidates
        thetaBasis1 = [1; 0; center; sigma];
        thetaBasis2 = [0; 1; center; sigma];
        pBasis1 = predictAloiGaussianShape(thetaBasis1, model);
        pBasis2 = predictAloiGaussianShape(thetaBasis2, model);
        response1 = pBasis1(:, model.measurementIdx) - preference(:, model.measurementIdx);
        response2 = pBasis2(:, model.measurementIdx) - preference(:, model.measurementIdx);
        A = [response1(:), response2(:)];
        amplitude = pinv(A) * targetDelta;
        amplitude = min(max(amplitude, -amplitudeBound), amplitudeBound);
        candidate = [amplitude; center; sigma];
        residual = aloiPositionResidual(candidate, model);
        currentCost = residual' * residual;
        if currentCost < bestInitialCost
            bestInitialCost = currentCost;
            theta0 = candidate;
        end
    end
end

if nargin >= 5 && ~isempty(previousTheta)
    previousTheta = min(max(previousTheta(:), lb), ub);
    previousCost = sum(aloiPositionResidual(previousTheta, model) .^ 2);
    if previousCost < bestInitialCost
        theta0 = previousTheta;
    end
end

residualFunction = @(theta) aloiPositionResidual(theta, model);
if exist('lsqnonlin', 'file') == 2
    options = optimoptions('lsqnonlin', 'Display', 'off', ...
        'MaxIterations', cfg.aloi.maxIterations, ...
        'MaxFunctionEvaluations', cfg.aloi.maxFunctionEvaluations, ...
        'FunctionTolerance', 1e-9, 'StepTolerance', 1e-8);
    theta = lsqnonlin(residualFunction, theta0, lb, ub, options);
else
    objective = @(theta) sum(residualFunction(theta) .^ 2);
    options = optimoptions('fmincon', 'Display', 'off', ...
        'MaxIterations', cfg.aloi.maxIterations, ...
        'MaxFunctionEvaluations', cfg.aloi.maxFunctionEvaluations);
    theta = fmincon(objective, theta0, [], [], [], [], lb, ub, [], options);
end

[predictedP, nodalForces] = predictAloiGaussianShape(theta, model);
positionError = predictedP(:, model.measurementIdx) - targetPositions;
fit = struct;
fit.theta = theta(:);
fit.centerMm = theta(3);
fit.sigmaMm = theta(4);
fit.forceResultant = sum(nodalForces, 2);
fit.nodalForces = nodalForces;
fit.shapeRmseMm = sqrt(mean(sum(positionError .^ 2, 1)));
fit.cost = sum((positionError(:) / model.positionStdMm) .^ 2);
end


function residual = aloiPositionResidual(theta, model)
p = predictAloiGaussianShape(theta, model);
residual = p(:, model.measurementIdx) - model.targetPositions;
residual = residual(:) / model.positionStdMm;
end


function [p, nodalForces] = predictAloiGaussianShape(theta, model)
density = gaussianDensityOnArc(model.s, theta(3), theta(4));
ns = numel(model.s);
localLoad = [theta(1) * density'; theta(2) * density'; zeros(1, ns)];
distributedLoad = zeros(3, ns);
for i = 1:ns
    distributedLoad(:, i) = model.referenceR(:, :, i) * localLoad(:, i);
end
nodalForces = distributedLoad .* reshape(model.integrationWeightsMm, 1, []);
moment = model.J' * nodalForces(:);
u = reshape(model.invK .* moment, 3, []) + model.tube.uhat;
[~, ~, p] = solveShape(model.tube.T_base, u, model.tube.s);
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
trueContactS = results.forward.contactArcLength;
validContactLocation = active & isfinite(trueContactS) & isfinite(results.aloi.centerMm);
locationErrorMm = abs(results.aloi.centerMm - trueContactS);
metrics.aloi.contactLocationErrorMm = locationErrorMm;
if any(validContactLocation)
    metrics.aloi.contactLocationRmseMm = ...
        sqrt(mean(locationErrorMm(validContactLocation) .^ 2));
else
    metrics.aloi.contactLocationRmseMm = nan;
end
metrics.aloi.finalContactLocationErrorMm = locationErrorMm(end);
metrics.aloi.finalShapeRmseMm = results.aloi.shapeRmseMm(end);
metrics.aloi.widthAtLowerBound = abs(results.aloi.sigmaMm - ...
    min(results.config.aloi.sigmaCandidatesMm)) <= 1e-6;

n = unitVector(results.config.planeNormal, [0; 0; 1]);
normalComponent = n' * trueContact;
tangentialComponent = trueContact - n * normalComponent;
metrics.forward.finalContactArcLengthMm = results.forward.contactArcLength(end);
metrics.forward.maxTangentialContactForce = max(vecnorm(tangentialComponent, 2, 1));
metrics.forward.maxNormalContactForce = max(abs(normalComponent));
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
axis tight;
plotPlaneSectionXZ(cfg.planePointMm, cfg.planeNormal, 'k--', 'LineWidth', 1.1);
plotPlaneSectionXZ(mean(results.measurements.planePointMeasured, 2), ...
    results.measurements.planeNormalMeasured, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.1);
xlabel('x [mm]');
ylabel('z [mm]');
title('Forward shape trajectory');

subplot(2, 2, 2);
plot(results.forward.betaMm, results.forward.contactForceResultant(1, :), 'k-', 'LineWidth', 1.8); hold on;
plot(results.forward.betaMm, results.ours.contactForceResultant(1, :), 'r--', 'LineWidth', 1.5);
plot(results.forward.betaMm, results.forward.contactForceResultant(3, :), 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8);
plot(results.forward.betaMm, results.ours.contactForceResultant(3, :), 'b--', 'LineWidth', 1.5);
grid on; box on;
xlabel('Base motion [mm]');
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
xlabel('Base motion [mm]');
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

fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 520]);
hold on; axis equal; grid on; box on;
p = results.forward.p(:, :, end);
pEstimated = results.ours.p(:, :, end);
hTrueShape = plot(p(1, :), p(3, :), 'k-', 'LineWidth', 2.2);
hEstimatedShape = plot(pEstimated(1, :), pEstimated(3, :), '--', ...
    'Color', [0.15 0.45 0.85], 'LineWidth', 1.8);
axis tight;
hTruePlane = plotPlaneSectionXZ(cfg.planePointMm, cfg.planeNormal, 'k--', 'LineWidth', 1.1);
hMeasuredPlane = plotPlaneSectionXZ(results.measurements.planePointMeasured(:, end), ...
    results.measurements.planeNormalMeasured, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.1);

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
estimatedContactOrigin = results.ours.contactPoint(:, end);
if any(~isfinite(estimatedContactOrigin))
    estimatedContactOrigin = pEstimated(:, idx);
end
tipOrigin = p(:, end);
hTrueContact = quiver(contactOrigin(1), contactOrigin(3), trueContact(1) * scale, trueContact(3) * scale, 0, 'Color', [0 0.45 0.75], 'LineWidth', 2.2);
hEstimatedContact = quiver(estimatedContactOrigin(1), estimatedContactOrigin(3), oursContact(1) * scale, oursContact(3) * scale, 0, 'Color', [0.1 0.6 0.2], 'LineWidth', 2.0);
hTrueTip = quiver(tipOrigin(1), tipOrigin(3), trueTip(1) * scale, trueTip(3) * scale, 0, 'Color', [0 0.45 0.75], 'LineStyle', '--', 'LineWidth', 2.0);
hEstimatedTip = quiver(tipOrigin(1), tipOrigin(3), oursTip(1) * scale, oursTip(3) * scale, 0, 'Color', [0.1 0.6 0.2], 'LineStyle', '--', 'LineWidth', 1.8);
hAloi = quiver(tipOrigin(1), tipOrigin(3), aloiTotal(1) * scale, aloiTotal(3) * scale, 0, 'Color', [0.85 0.2 0.1], 'LineWidth', 1.6);
xlabel('x [mm]');
ylabel('z [mm]');
title('Final shape, contact force, and tip load');
legend([hTrueShape, hEstimatedShape, hTruePlane, hMeasuredPlane, hTrueContact, ...
    hEstimatedContact, hTrueTip, hEstimatedTip, hAloi], ...
    {'True shape', 'Estimated shape', 'True plane', 'Measured plane', ...
     'True contact', 'Estimated contact', 'True tip load', ...
     'Estimated tip load', 'Aloi total'}, 'Location', 'eastoutside');
saveFigure(fig2, fullfile(cfg.outputDir, 'rod_plane_final_force_comparison.png'));
plotAloiDiagnostics(results);
end


function plotAloiDiagnostics(results)
cfg = results.config;
motion = results.forward.actuationMm;
trueForce = results.forward.totalForceResultant;
estimatedForce = results.aloi.totalForceResultant;
forceError = vecnorm(estimatedForce - trueForce, 2, 1);
relativeError = 100 * forceError ./ max(vecnorm(trueForce, 2, 1), eps);
trueContactS = results.forward.contactArcLength;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [90 90 1250 760]);
subplot(2, 2, 1);
plot(motion, trueForce(1, :), 'k-', 'LineWidth', 1.8); hold on;
plot(motion, estimatedForce(1, :), 'r--', 'LineWidth', 1.6);
plot(motion, trueForce(3, :), '-', 'Color', [0.15 0.45 0.80], 'LineWidth', 1.8);
plot(motion, estimatedForce(3, :), '--', 'Color', [0.55 0.20 0.70], 'LineWidth', 1.6);
grid on; box on; axis tight;
xlabel('Base motion [mm]'); ylabel('Total force [N]');
title('Aloi force components');
legend({'True F_x', 'Aloi F_x', 'True F_z', 'Aloi F_z'}, 'Location', 'best');

subplot(2, 2, 2);
yyaxis left;
plot(motion, forceError, 'Color', [0.80 0.15 0.10], 'LineWidth', 1.8);
ylabel('Force error norm [N]');
yyaxis right;
plot(motion, relativeError, 'Color', [0.20 0.45 0.75], 'LineWidth', 1.4);
ylabel('Relative force error [%]');
grid on; box on; axis tight;
xlabel('Base motion [mm]');
title('Aloi force error');

subplot(2, 2, 3);
plot(motion, results.aloi.shapeRmseMm, 'Color', [0.10 0.55 0.35], 'LineWidth', 1.8);
grid on; box on; axis tight;
xlabel('Base motion [mm]'); ylabel('Sparse-position RMSE [mm]');
title('Aloi shape fit');

subplot(2, 2, 4);
plot(motion, trueContactS, 'k-', 'LineWidth', 1.8); hold on;
plot(motion, results.aloi.centerMm, 'r--', 'LineWidth', 1.8);
plot(motion, results.aloi.centerMm - results.aloi.sigmaMm, ':', ...
    'Color', [0.85 0.45 0.40], 'LineWidth', 1.1);
plot(motion, results.aloi.centerMm + results.aloi.sigmaMm, ':', ...
    'Color', [0.85 0.45 0.40], 'LineWidth', 1.1);
grid on; box on; axis tight;
xlabel('Base motion [mm]'); ylabel('Arc length s [mm]');
title('True contact and fitted Gaussian location');
legend({'True contact', 'Aloi center', 'Center - sigma', 'Center + sigma'}, ...
    'Location', 'best');

saveFigure(fig, fullfile(cfg.outputDir, 'rod_plane_aloi_error_analysis.png'));
end


function createForceSensingVideo(results)
cfg = results.config;
videoPath = fullfile(cfg.outputDir, cfg.video.fileName);
[samplePositions, nVideoFrames] = videoSamplePositions( ...
    numel(results.forward.actuationMm), cfg.video);
fprintf('\nWriting formulation force-sensing video: %s\n', videoPath);
fprintf('  %d rendered frames at %.1f fps (%.2f s)\n', ...
    nVideoFrames, cfg.video.frameRate, nVideoFrames / cfg.video.frameRate);

writer = VideoWriter(videoPath, 'MPEG-4');
writer.FrameRate = cfg.video.frameRate;
open(writer);
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1180 620]);
tempFramePath = fullfile(cfg.outputDir, '.formulation_video_frame.png');

trueP = results.forward.p;
estimatedP = results.ours.p;
[xLimits, zLimits] = videoAxesLimits(trueP, estimatedP);
trueForce = results.forward.contactForceResultant;
estimatedForce = results.ours.contactForceResultant;
trueOrigins = contactPointTrajectory(results.forward.contacts, trueP);
estimatedOrigins = results.ours.contactPoint;
maxForce = max([vecnorm(trueForce, 2, 1), vecnorm(estimatedForce, 2, 1), 1]);
shapeSpan = max(diff(xLimits), diff(zLimits));
forceScale = min(cfg.video.forceScaleMmPerN, 0.18 * shapeSpan / maxForce);
n = unitVector(cfg.planeNormal, [0; 0; 1]);
t = unitVector(cfg.forward.slideDirection, contactTangentFromNormal(n));
trueNormal = n' * trueForce;
estimatedNormal = n' * estimatedForce;
trueTangent = t' * trueForce;
estimatedTangent = t' * estimatedForce;
motion = results.forward.actuationMm;
nt = numel(motion);

try
    for frameIdx = 1:nVideoFrames
        q = samplePositions(frameIdx);
        pTrue = interpolateLastDimension(trueP, q);
        pEstimated = interpolateLastDimension(estimatedP, q);
        trueForceNow = interpolateLastDimension(trueForce, q);
        estimatedForceNow = interpolateLastDimension(estimatedForce, q);
        trueOrigin = interpolateLastDimension(trueOrigins, q);
        estimatedOrigin = interpolateLastDimension(estimatedOrigins, q);
        if any(~isfinite(estimatedOrigin))
            estimatedOrigin = pEstimated(:, max(1, round(size(pEstimated, 2) / 2)));
        end
        motionNow = interpolateLastDimension(motion, q);
        shapeRmseNow = interpolateLastDimension(results.ours.shapeRmseMm, q);
        phase = results.forward.phase{min(nt, max(1, round(q)))};
        [motionHistory, estimatedNormalHistory] = timeHistoryAt(motion, estimatedNormal, q);
        [~, estimatedTangentHistory] = timeHistoryAt(motion, estimatedTangent, q);

        clf(fig);
        layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        axShape = nexttile(layout, 1);
        hold(axShape, 'on'); grid(axShape, 'on'); box(axShape, 'on'); axis(axShape, 'equal');
        hTrueShape = plot(axShape, pTrue(1, :), pTrue(3, :), '-', ...
            'Color', [0.55 0.78 0.95], 'LineWidth', 2.6);
        hEstimatedShape = plot(axShape, pEstimated(1, :), pEstimated(3, :), '-', ...
            'Color', [0.05 0.28 0.72], 'LineWidth', 2.0);
        xlim(axShape, xLimits); ylim(axShape, zLimits);
        hPlane = plotPlaneSectionXZ(cfg.planePointMm, cfg.planeNormal, ...
            'k--', 'LineWidth', 1.1);
        hTrueForce = quiver(axShape, trueOrigin(1), trueOrigin(3), ...
            trueForceNow(1) * forceScale, trueForceNow(3) * forceScale, 0, ...
            'Color', [0.15 0.60 0.82], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        hEstimatedForce = quiver(axShape, estimatedOrigin(1), estimatedOrigin(3), ...
            estimatedForceNow(1) * forceScale, estimatedForceNow(3) * forceScale, 0, ...
            'Color', [0.85 0.15 0.10], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        xlabel(axShape, 'x [mm]'); ylabel(axShape, 'z [mm]');
        title(axShape, sprintf(['Video frame %d/%d, simulation %.2f/%d, %s, ', ...
            'shape RMSE %.3f mm'], frameIdx, nVideoFrames, q, nt, phase, shapeRmseNow));
        legend(axShape, [hTrueShape, hEstimatedShape, hPlane, hTrueForce, hEstimatedForce], ...
            {'True shape', 'Estimated shape', 'Plane', 'True contact force', ...
            'Estimated contact force'}, 'Location', 'southoutside');

        axForce = nexttile(layout, 2);
        hold(axForce, 'on'); grid(axForce, 'on'); box(axForce, 'on');
        hTrueNormal = plot(axForce, motion, trueNormal, '-', ...
            'Color', [0.15 0.15 0.15], 'LineWidth', 1.4);
        hEstimatedNormal = plot(axForce, motionHistory, estimatedNormalHistory, '--', ...
            'Color', [0.85 0.15 0.10], 'LineWidth', 1.8);
        hTrueTangent = plot(axForce, motion, trueTangent, '-', ...
            'Color', [0.10 0.45 0.80], 'LineWidth', 1.4);
        hEstimatedTangent = plot(axForce, motionHistory, estimatedTangentHistory, '--', ...
            'Color', [0.55 0.20 0.70], 'LineWidth', 1.8);
        xline(axForce, motionNow, ':', 'Color', [0.35 0.35 0.35]);
        xlabel(axForce, 'Base motion [mm]'); ylabel(axForce, 'Contact force [N]');
        title(axForce, sprintf('true=[%.2f %.2f %.2f] N, estimated=[%.2f %.2f %.2f] N', ...
            trueForceNow, estimatedForceNow));
        legend(axForce, [hTrueNormal, hEstimatedNormal, hTrueTangent, hEstimatedTangent], ...
            {'True normal', 'Estimated normal', 'True tangential', ...
            'Estimated tangential'}, 'Location', 'best');

        writeFigureVideoFrame(writer, fig, tempFramePath);
        printVideoProgress(frameIdx, nVideoFrames, 'formulation');
    end
    close(writer);
    close(fig);
catch videoError
    close(writer);
    close(fig);
    deleteIfPresent(tempFramePath);
    rethrow(videoError);
end
deleteIfPresent(tempFramePath);
fprintf('Saved formulation video: %s\n', videoPath);
end


function createAloiForceSensingVideo(results)
cfg = results.config;
videoPath = fullfile(cfg.outputDir, cfg.video.aloiFileName);
[samplePositions, nVideoFrames] = videoSamplePositions( ...
    numel(results.forward.actuationMm), cfg.video);
fprintf('\nWriting Aloi comparison video: %s\n', videoPath);
fprintf('  %d rendered frames at %.1f fps (%.2f s)\n', ...
    nVideoFrames, cfg.video.frameRate, nVideoFrames / cfg.video.frameRate);

writer = VideoWriter(videoPath, 'MPEG-4');
writer.FrameRate = cfg.video.frameRate;
open(writer);
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1180 620]);
tempFramePath = fullfile(cfg.outputDir, '.aloi_video_frame.png');

trueP = results.forward.p;
estimatedP = results.aloi.p;
[xLimits, zLimits] = videoAxesLimits(trueP, estimatedP);
trueForce = results.forward.totalForceResultant;
estimatedForce = results.aloi.totalForceResultant;
trueOrigins = contactPointTrajectory(results.forward.contacts, trueP);
estimatedOrigins = results.aloi.loadPoint;
maxForce = max([vecnorm(trueForce, 2, 1), vecnorm(estimatedForce, 2, 1), 1]);
shapeSpan = max(diff(xLimits), diff(zLimits));
forceScale = min(cfg.video.forceScaleMmPerN, 0.18 * shapeSpan / maxForce);
forceError = vecnorm(estimatedForce - trueForce, 2, 1);
motion = results.forward.actuationMm;
nt = numel(motion);

try
    for frameIdx = 1:nVideoFrames
        q = samplePositions(frameIdx);
        pTrue = interpolateLastDimension(trueP, q);
        pEstimated = interpolateLastDimension(estimatedP, q);
        trueForceNow = interpolateLastDimension(trueForce, q);
        estimatedForceNow = interpolateLastDimension(estimatedForce, q);
        trueOrigin = interpolateLastDimension(trueOrigins, q);
        estimatedOrigin = interpolateLastDimension(estimatedOrigins, q);
        motionNow = interpolateLastDimension(motion, q);
        shapeRmseNow = interpolateLastDimension(results.aloi.shapeRmseMm, q);
        forceErrorNow = interpolateLastDimension(forceError, q);
        [motionHistory, estimatedFxHistory] = timeHistoryAt(motion, estimatedForce(1, :), q);
        [~, estimatedFzHistory] = timeHistoryAt(motion, estimatedForce(3, :), q);
        [~, forceErrorHistory] = timeHistoryAt(motion, forceError, q);

        clf(fig);
        layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        axShape = nexttile(layout, 1);
        hold(axShape, 'on'); grid(axShape, 'on'); box(axShape, 'on'); axis(axShape, 'equal');
        hTrueShape = plot(axShape, pTrue(1, :), pTrue(3, :), '-', ...
            'Color', [0.55 0.78 0.95], 'LineWidth', 2.6);
        hEstimatedShape = plot(axShape, pEstimated(1, :), pEstimated(3, :), '-', ...
            'Color', [0.05 0.28 0.72], 'LineWidth', 2.0);
        xlim(axShape, xLimits); ylim(axShape, zLimits);
        hPlane = plotPlaneSectionXZ(cfg.planePointMm, cfg.planeNormal, ...
            'k--', 'LineWidth', 1.1);
        hTrueForce = quiver(axShape, trueOrigin(1), trueOrigin(3), ...
            trueForceNow(1) * forceScale, trueForceNow(3) * forceScale, 0, ...
            'Color', [0.15 0.60 0.82], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        hEstimatedForce = quiver(axShape, estimatedOrigin(1), estimatedOrigin(3), ...
            estimatedForceNow(1) * forceScale, estimatedForceNow(3) * forceScale, 0, ...
            'Color', [0.85 0.15 0.10], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        xlabel(axShape, 'x [mm]'); ylabel(axShape, 'z [mm]');
        title(axShape, sprintf(['Aloi frame %d/%d, simulation %.2f/%d, ', ...
            'shape RMSE %.3f mm'], frameIdx, nVideoFrames, q, nt, shapeRmseNow));
        legend(axShape, [hTrueShape, hEstimatedShape, hPlane, hTrueForce, hEstimatedForce], ...
            {'True shape', 'Aloi fitted shape', 'Plane', 'True total force', ...
            'Aloi resultant'}, 'Location', 'southoutside');

        axForce = nexttile(layout, 2);
        hold(axForce, 'on'); grid(axForce, 'on'); box(axForce, 'on');
        hTrueFx = plot(axForce, motion, trueForce(1, :), 'k-', 'LineWidth', 1.4);
        hAloiFx = plot(axForce, motionHistory, estimatedFxHistory, 'r--', 'LineWidth', 1.8);
        hTrueFz = plot(axForce, motion, trueForce(3, :), '-', ...
            'Color', [0.10 0.45 0.80], 'LineWidth', 1.4);
        hAloiFz = plot(axForce, motionHistory, estimatedFzHistory, '--', ...
            'Color', [0.55 0.20 0.70], 'LineWidth', 1.8);
        hError = plot(axForce, motionHistory, forceErrorHistory, ':', ...
            'Color', [0.10 0.55 0.35], 'LineWidth', 1.8);
        xline(axForce, motionNow, ':', 'Color', [0.35 0.35 0.35]);
        xlabel(axForce, 'Base motion [mm]'); ylabel(axForce, 'Force [N]');
        title(axForce, sprintf('force error %.3f N, true=[%.2f %.2f %.2f], Aloi=[%.2f %.2f %.2f]', ...
            forceErrorNow, trueForceNow, estimatedForceNow));
        legend(axForce, [hTrueFx, hAloiFx, hTrueFz, hAloiFz, hError], ...
            {'True F_x', 'Aloi F_x', 'True F_z', 'Aloi F_z', 'Error norm'}, ...
            'Location', 'best');

        writeFigureVideoFrame(writer, fig, tempFramePath);
        printVideoProgress(frameIdx, nVideoFrames, 'Aloi');
    end
    close(writer);
    close(fig);
catch videoError
    close(writer);
    close(fig);
    deleteIfPresent(tempFramePath);
    rethrow(videoError);
end
deleteIfPresent(tempFramePath);
fprintf('Saved Aloi video: %s\n', videoPath);
end


function [samplePositions, nVideoFrames] = videoSamplePositions(nt, videoCfg)
minimumFrames = ceil(videoCfg.durationSeconds * videoCfg.frameRate);
if isfield(videoCfg, 'renderFrameCount') && ~isempty(videoCfg.renderFrameCount)
    requestedFrames = round(videoCfg.renderFrameCount);
else
    requestedFrames = minimumFrames;
end
nVideoFrames = max([nt, minimumFrames, requestedFrames]);
samplePositions = linspace(1, nt, nVideoFrames);
end


function value = interpolateLastDimension(data, samplePosition)
nt = size(data, ndims(data));
i0 = max(1, min(nt, floor(samplePosition)));
i1 = max(1, min(nt, ceil(samplePosition)));
alpha = samplePosition - i0;
subs0 = repmat({':'}, 1, ndims(data));
subs1 = subs0;
subs0{end} = i0;
subs1{end} = i1;
value = (1 - alpha) * data(subs0{:}) + alpha * data(subs1{:});
end


function [motionHistory, valueHistory] = timeHistoryAt(motion, values, samplePosition)
lastComplete = max(1, min(numel(motion), floor(samplePosition)));
motionHistory = motion(1:lastComplete);
valueHistory = values(1:lastComplete);
if samplePosition > lastComplete + 1e-10 && lastComplete < numel(motion)
    motionHistory(end + 1) = interpolateLastDimension(motion, samplePosition);
    valueHistory(end + 1) = interpolateLastDimension(values, samplePosition);
end
end


function points = contactPointTrajectory(contacts, shapeTrajectory)
nt = numel(contacts);
points = nan(3, nt);
for it = 1:nt
    points(:, it) = strongestForwardContactPoint(contacts{it}, shapeTrajectory(:, :, it));
end
end


function [xLimits, zLimits] = videoAxesLimits(shapeA, shapeB)
allX = [reshape(shapeA(1, :, :), 1, []), reshape(shapeB(1, :, :), 1, [])];
allZ = [reshape(shapeA(3, :, :), 1, []), reshape(shapeB(3, :, :), 1, [])];
xMargin = max(10, 0.08 * max(max(allX) - min(allX), 1));
zMargin = max(10, 0.08 * max(max(allZ) - min(allZ), 1));
xLimits = [min(allX) - xMargin, max(allX) + xMargin];
zLimits = [min(allZ) - zMargin, max(allZ) + zMargin];
end


function writeFigureVideoFrame(writer, fig, tempFramePath)
drawnow;
try
    frame = getframe(fig);
    imageData = frame.cdata;
catch
    exportgraphics(fig, tempFramePath, 'Resolution', 120);
    imageData = imread(tempFramePath);
end
writeVideo(writer, imageData);
end


function printVideoProgress(frameIdx, nVideoFrames, label)
if mod(frameIdx, max(1, floor(nVideoFrames / 10))) == 0 || frameIdx == nVideoFrames
    fprintf('  %s video frame %3d/%3d\n', label, frameIdx, nVideoFrames);
end
end


function deleteIfPresent(path)
if exist(path, 'file')
    delete(path);
end
end


function createForceSensingVideoLegacy(results)
cfg = results.config;
videoPath = fullfile(cfg.outputDir, cfg.video.fileName);
fprintf('\nWriting force-sensing video: %s\n', videoPath);

writer = VideoWriter(videoPath, 'MPEG-4');
writer.FrameRate = cfg.video.frameRate;
open(writer);
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1180 620]);

trueP = results.forward.p;
estimatedP = results.ours.p;
allX = [reshape(trueP(1, :, :), 1, []), reshape(estimatedP(1, :, :), 1, [])];
allZ = [reshape(trueP(3, :, :), 1, []), reshape(estimatedP(3, :, :), 1, [])];
xMargin = max(10, 0.08 * max(max(allX) - min(allX), 1));
zMargin = max(10, 0.08 * max(max(allZ) - min(allZ), 1));
xLimits = [min(allX) - xMargin, max(allX) + xMargin];
zLimits = [min(allZ) - zMargin, max(allZ) + zMargin];

trueForce = results.forward.contactForceResultant;
estimatedForce = results.ours.contactForceResultant;
maxForce = max([vecnorm(trueForce, 2, 1), vecnorm(estimatedForce, 2, 1), 1]);
shapeSpan = max(diff(xLimits), diff(zLimits));
forceScale = min(cfg.video.forceScaleMmPerN, 0.18 * shapeSpan / maxForce);
n = unitVector(cfg.planeNormal, [0; 0; 1]);
t = unitVector(cfg.forward.slideDirection, contactTangentFromNormal(n));
trueNormal = n' * trueForce;
estimatedNormal = n' * estimatedForce;
trueTangent = t' * trueForce;
estimatedTangent = t' * estimatedForce;
nt = numel(results.forward.actuationMm);
tempFramePath = fullfile(cfg.outputDir, '.force_sensing_video_frame.png');

try
    for it = 1:nt
        clf(fig);
        layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

        axShape = nexttile(layout, 1);
        hold(axShape, 'on');
        grid(axShape, 'on');
        box(axShape, 'on');
        axis(axShape, 'equal');
        pTrue = trueP(:, :, it);
        pEstimated = estimatedP(:, :, it);
        hTrueShape = plot(axShape, pTrue(1, :), pTrue(3, :), '-', ...
            'Color', [0.55 0.78 0.95], 'LineWidth', 2.6);
        hEstimatedShape = plot(axShape, pEstimated(1, :), pEstimated(3, :), '-', ...
            'Color', [0.05 0.28 0.72], 'LineWidth', 2.0);
        xlim(axShape, xLimits);
        ylim(axShape, zLimits);
        hPlane = plotPlaneSectionXZ(cfg.planePointMm, cfg.planeNormal, 'k--', 'LineWidth', 1.1);

        trueOrigin = strongestForwardContactPoint(results.forward.contacts{it}, pTrue);
        estimatedOrigin = results.ours.contactPoint(:, it);
        if any(~isfinite(estimatedOrigin))
            estimatedOrigin = pEstimated(:, max(1, round(size(pEstimated, 2) / 2)));
        end
        hTrueForce = quiver(axShape, trueOrigin(1), trueOrigin(3), ...
            trueForce(1, it) * forceScale, trueForce(3, it) * forceScale, 0, ...
            'Color', [0.15 0.60 0.82], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        hEstimatedForce = quiver(axShape, estimatedOrigin(1), estimatedOrigin(3), ...
            estimatedForce(1, it) * forceScale, estimatedForce(3, it) * forceScale, 0, ...
            'Color', [0.85 0.15 0.10], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        xlabel(axShape, 'x [mm]');
        ylabel(axShape, 'z [mm]');
        title(axShape, sprintf('Frame %d/%d, %s, shape RMSE %.2f mm', ...
            it, nt, results.forward.phase{it}, results.ours.shapeRmseMm(it)));
        legend(axShape, [hTrueShape, hEstimatedShape, hPlane, hTrueForce, hEstimatedForce], ...
            {'True shape', 'Estimated shape', 'Plane', 'True contact force', 'Estimated contact force'}, ...
            'Location', 'southoutside');

        axForce = nexttile(layout, 2);
        hold(axForce, 'on');
        grid(axForce, 'on');
        box(axForce, 'on');
        motion = results.forward.actuationMm;
        hTrueNormal = plot(axForce, motion, trueNormal, '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.4);
        hEstimatedNormal = plot(axForce, motion(1:it), estimatedNormal(1:it), '--', ...
            'Color', [0.85 0.15 0.10], 'LineWidth', 1.8);
        hTrueTangent = plot(axForce, motion, trueTangent, '-', 'Color', [0.10 0.45 0.80], 'LineWidth', 1.4);
        hEstimatedTangent = plot(axForce, motion(1:it), estimatedTangent(1:it), '--', ...
            'Color', [0.55 0.20 0.70], 'LineWidth', 1.8);
        xline(axForce, motion(it), ':', 'Color', [0.35 0.35 0.35]);
        xlabel(axForce, 'Base motion [mm]');
        ylabel(axForce, 'Contact force [N]');
        title(axForce, sprintf('true=[%.2f %.2f %.2f] N, estimated=[%.2f %.2f %.2f] N', ...
            trueForce(:, it), estimatedForce(:, it)));
        legend(axForce, [hTrueNormal, hEstimatedNormal, hTrueTangent, hEstimatedTangent], ...
            {'True normal', 'Estimated normal', 'True tangential', ...
            'Estimated tangential'}, 'Location', 'best');

        drawnow;
        try
            frame = getframe(fig);
            imageData = frame.cdata;
        catch
            exportgraphics(fig, tempFramePath, 'Resolution', 120);
            imageData = imread(tempFramePath);
        end
        writeVideo(writer, imageData);
        if mod(it, max(1, floor(nt / 5))) == 0 || it == nt
            fprintf('  video frame %3d/%3d\n', it, nt);
        end
    end
    close(writer);
    close(fig);
catch videoError
    close(writer);
    close(fig);
    if exist(tempFramePath, 'file')
        delete(tempFramePath);
    end
    rethrow(videoError);
end
if exist(tempFramePath, 'file')
    delete(tempFramePath);
end
fprintf('Saved video: %s\n', videoPath);
end


function point = strongestForwardContactPoint(contacts, fallbackShape)
if isempty(contacts)
    point = fallbackShape(:, end);
    return;
end
forces = reshape([contacts.force], 3, []);
[~, idx] = max(vecnorm(forces, 2, 1));
point = contacts(idx).point(:);
end


function h = plotPlaneSectionXZ(point, normal, varargin)
ax = gca;
point = point(:);
normal = unitVector(normal, [0; 0; 1]);
normalXZ = [normal(1); normal(3)];
if norm(normalXZ) < 1e-12
    h = plot(ax, nan, nan, varargin{:});
    return;
end
directionXZ = unitVector([-normalXZ(2); normalXZ(1)], [1; 0]);
xl = xlim(ax);
zl = ylim(ax);
span = 2 * hypot(diff(xl), diff(zl));
linePoints = [point(1); point(3)] + directionXZ * [-span, span];
h = plot(ax, linePoints(1, :), linePoints(2, :), varargin{:});
set(ax, 'XLim', xl, 'YLim', zl);
end


function writeTrajectoryCsv(results)
cfg = results.config;
nt = numel(results.forward.betaMm);
tip = squeeze(results.forward.p(:, end, :));

T = table;
T.frame = (1:nt)';
T.insertion_mm = results.forward.betaMm(:);
T.phase = results.forward.phase(:);
T.friction_mu = results.forward.frictionMu(:);
T.tip_x_mm = tip(1, :)';
T.tip_y_mm = tip(2, :)';
T.tip_z_mm = tip(3, :)';
T.plane_x_measured_mm = results.measurements.planePointMeasured(1, :)';
T.plane_y_measured_mm = results.measurements.planePointMeasured(2, :)';
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
T.aloi_center_s_mm = results.aloi.centerMm(:);
T.aloi_sigma_mm = results.aloi.sigmaMm(:);
T.aloi_shape_rmse_mm = results.aloi.shapeRmseMm(:);
T.aloi_contact_location_error_mm = results.metrics.aloi.contactLocationErrorMm(:);

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
T.ours_shape_rmse_mm = results.ours.shapeRmseMm(:);
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
fprintf(fid, 'Scenario: %s\n', cfg.scenarioName);
fprintf(fid, ['Forward model: displacement-aware local copy of Jia Shen''s updated rod-plane LCP ', ...
    'with a retained tip-load extension; original LCP-Continuum files are unchanged.\n']);
fprintf(fid, 'Inverse input: sparse FBG-like curvature samples plus a measured plane point and plane normal.\n\n');

fprintf(fid, 'Rod length: %.1f mm\n', cfg.exposedLengthMm);
fprintf(fid, 'Precurvature scaling enabled: %d\n', cfg.scalePrecurvature);
if cfg.scalePrecurvature
    fprintf(fid, 'Target bend: %.1f deg\n', cfg.targetBendDeg);
end
fprintf(fid, 'Actual integrated precurvature: %.2f deg\n', results.setup.actualBendDeg);
fprintf(fid, 'Plane point [x y z] mm: [%.6g %.6g %.6g]\n', cfg.planePointMm);
fprintf(fid, 'Plane normal [nx ny nz]: [%.6g %.6g %.6g]\n', cfg.planeNormal);
fprintf(fid, 'Measured plane offset bias: %.2f mm\n', cfg.sensing.planeOffsetBiasMm);
fprintf(fid, 'Friction coefficient schedule: %s\n', mat2str(results.forward.frictionMu, 4));
fprintf(fid, 'Push/slide friction coefficients: %.2f / %.2f\n', ...
    cfg.initialLowFrictionMu, cfg.frictionMu);
fprintf(fid, 'Applied tip load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', cfg.tipLoadN);
fprintf(fid, 'Motion mode: %s\n', cfg.forward.motionMode);
fprintf(fid, 'Forward source revision: %s\n', cfg.forward.sourceRevision);
fprintf(fid, 'Frames: %d, commanded path length: %.1f mm\n\n', ...
    numel(results.forward.actuationMm), results.forward.actuationMm(end));

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
fprintf(fid, 'Final Aloi Gaussian position-fit total load [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.aloi.totalForceResultant(:, end));
fprintf(fid, 'Final Aloi Gaussian resultant [Fx Fy Fz] N: [%.6g %.6g %.6g]\n', ...
    results.aloi.componentResultants(:, 1, end));
fprintf(fid, 'Final Aloi Gaussian center/sigma: %.3f / %.3f mm\n', ...
    results.aloi.centerMm(end), results.aloi.sigmaMm(end));
fprintf(fid, 'Final Aloi sparse-position RMSE: %.6g mm\n', ...
    results.aloi.shapeRmseMm(end));
fprintf(fid, 'Final Aloi normalized position-fit cost: %.6g\n\n', results.aloi.cost(end));

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
fprintf(fid, 'Aloi final total-load relative error: %.4f %%\n', results.metrics.aloi.finalRelativeErrorPct);
fprintf(fid, 'Aloi contact-location RMSE: %.6g mm\n', results.metrics.aloi.contactLocationRmseMm);
fprintf(fid, 'Aloi final contact-location error: %.6g mm\n', results.metrics.aloi.finalContactLocationErrorMm);
fprintf(fid, 'Aloi Gaussian width at configured lower bound in final frame: %d\n\n', ...
    results.metrics.aloi.widthAtLowerBound(end));

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
arcLengths = arrayfun(@(contact) contactAppliedArcLength(contact, s), contacts);
meanArcLength = mean(arcLengths);
end


function arcLength = contactAppliedArcLength(contact, s)
if isfield(contact, 'applied_tube_point_id') && ~isempty(contact.applied_tube_point_id)
    ids = contact.applied_tube_point_id;
else
    ids = contact.tube_point_id;
end
arcLength = mean(s(ids));
end


function ensureDir(pathStr)
if ~exist(pathStr, 'dir')
    mkdir(pathStr);
end
end


function rootDir = resolveRepositoryRoot(packageDir)
rootDir = packageDir;
if ~exist(fullfile(rootDir, 'LCP-Continuum'), 'dir')
    rootDir = fileparts(packageDir);
end
if ~exist(fullfile(rootDir, 'LCP-Continuum'), 'dir')
    error('Could not locate the LCP-Continuum dependency from %s.', packageDir);
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
