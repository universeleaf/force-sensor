function results = simu_rod_plane_upward_force_sensing(quickMode, inverseOnly)
%SIMU_ROD_PLANE_UPWARD_FORCE_SENSING Rigidly rotate wall by -90 deg about y.
% A translation puts the initial base at the origin. This is a controlled
% rotation experiment, not a parameter-exact reconstruction of fail.mp4.
if nargin < 1, quickMode = false; end
if nargin < 2, inverseOnly = false; end
root = fileparts(fileparts(mfilename('fullpath')));
Q = [0 0 -1; 0 1 0; 1 0 0];
t = [0; 0; 120];
o.scenarioName = 'wall_rotated_90deg_upward';
o.outputDir = fullfile(root, 'out', 'upward');
if quickMode, o.outputDir = fullfile(root, 'out', 'upward_tmp'); end
o.exposedLengthMm = 150;
o.scalePrecurvature = false;
o.basePositionMm = Q * [-120; 0; 0] + t;
o.baseRotation = Q * [0 0 1; 0 -1 0; 1 0 0];
o.planePointMm = Q * [20; 0; 0] + t;
o.planeNormal = Q * [-1; 0; 0];
o.tipLoadN = zeros(3, 1);
o.frictionMu = 0.5;
o.initialLowFrictionMu = 0;
o.numTimeSteps = 18;
if quickMode, o.numTimeSteps = 6; end
o.forward.motionMode = 'push-slide';
o.forward.pushDistanceMm = 45;
o.forward.slideDistanceMm = 1;
o.forward.pushFraction = 0.5;
o.forward.pushDirection = Q * [1; 0; 0];
o.forward.slideDirection = Q * [0; 0; 1];
% These standard deviations are in WORLD axes. Rotate their diagonal
% covariances too; abs(Q) is exact here because Q is a signed permutation.
o.forceSensor.priorStd.planePointMm = abs(Q) * [8; 8; 0.25];
o.forceSensor.processStd.planePointMm = abs(Q) * [0.25; 0.25; 0.10];
o.forceSensor.measurementStd.planePointMm = abs(Q) * [8; 8; 0.13];
o.diagnostics.runMapCandidateCosts = false;
o.diagnostics.stopAfterInverse = inverseOnly;
o.video.enabled = true;
o.video.frameRate = 10;
o.video.durationSeconds = 6;
o.video.renderFrameCount = 60;
o.provenance.rotationFromWall = Q;
o.provenance.translationFromWallMm = t;
o.provenance.videoMatch = 'qualitative upward-push geometry; video parameters unavailable';
results = run_rod_plane_force_sensing_experiment(quickMode, o);
if inverseOnly
    results=save_result_checkpoint(results,o.outputDir,'inverse_results.mat');
end
end
