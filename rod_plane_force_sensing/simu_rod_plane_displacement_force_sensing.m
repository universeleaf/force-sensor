function results = simu_rod_plane_displacement_force_sensing(quickMode)
%SIMU_ROD_PLANE_DISPLACEMENT_FORCE_SENSING
% Displacement-aware push-then-slide rod-plane force-sensing experiment.
% The original Jia0Shen/LCP-Continuum source tree is not modified.

if nargin < 1
    quickMode = false;
end

packageDir = fileparts(mfilename('fullpath'));
rootDir = packageDir;
if ~exist(fullfile(rootDir, 'LCP-Continuum'), 'dir')
    rootDir = fileparts(packageDir);
end
overrides = struct;
overrides.scenarioName = 'validated_vertical_wall_body_contact';
if quickMode
    overrides.outputDir = fullfile(rootDir, 'force_outputs', ...
        'rod_plane_displacement_force_sensing_smoke_tmp');
else
    overrides.outputDir = fullfile(rootDir, 'force_outputs', ...
        'rod_plane_displacement_force_sensing');
end

overrides.exposedLengthMm = 150;
overrides.scalePrecurvature = false;
overrides.basePositionMm = [-120; 0; 0];
overrides.planePointMm = [20; 0; 0];
overrides.planeNormal = [-1; 0; 0];
overrides.frictionMu = 0.5;
overrides.initialLowFrictionMu = 0.0;
overrides.tipLoadN = zeros(3, 1);
overrides.numTimeSteps = 18;

overrides.forward.motionMode = 'push-slide';
overrides.forward.pushDistanceMm = 45;
overrides.forward.slideDistanceMm = 1;
overrides.forward.pushFraction = 0.5;
overrides.forward.slideDirection = [0; 0; 1];

overrides.video.enabled = true;
overrides.video.frameRate = 10;
overrides.video.durationSeconds = 6;
overrides.video.renderFrameCount = 60;
overrides.video.fileName = 'rod_plane_displacement_force_prediction.mp4';
overrides.video.aloiFileName = 'rod_plane_displacement_aloi_prediction.mp4';

if quickMode
    overrides.numTimeSteps = 6;
end

results = run_rod_plane_force_sensing_experiment(quickMode, overrides);
results.validation = validate_rod_plane_displacement_results(results);
end
