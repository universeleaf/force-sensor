function results = simu_rod_plane_senior_geometry_force_sensing(quickMode)
%SIMU_ROD_PLANE_SENIOR_GEOMETRY_FORCE_SENSING
% Horizontal-plane diagnostic based on the senior's displacement video.
% Friction is reduced from 2.8 to 0.5 and the true tip load is zero.

if nargin < 1
    quickMode = false;
end

packageDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(packageDir);
overrides = struct;
overrides.scenarioName = 'senior_horizontal_plane_tip_contact_mu05';
if quickMode
    overrides.outputDir = fullfile(rootDir, 'force_outputs', ...
        'rod_plane_senior_geometry_force_sensing_smoke_tmp');
else
    overrides.outputDir = fullfile(rootDir, 'force_outputs', ...
        'rod_plane_senior_geometry_force_sensing');
end

overrides.exposedLengthMm = 150;
overrides.scalePrecurvature = false;
overrides.basePositionMm = [-120; 0; 0];
overrides.planePointMm = [0; 0; 50];
overrides.planeNormal = [0; 0; -1];
overrides.frictionMu = 0.5;
overrides.initialLowFrictionMu = 0.0;
overrides.tipLoadN = zeros(3, 1);
overrides.numTimeSteps = 10;

overrides.forward.motionMode = 'push-slide';
overrides.forward.pushDistanceMm = 40;
overrides.forward.slideDistanceMm = 20;
overrides.forward.pushFraction = 0.4;
overrides.forward.pushDirection = [0; 0; 1];
overrides.forward.slideDirection = [1; 0; 0];

overrides.video.enabled = true;
overrides.video.frameRate = 10;
overrides.video.durationSeconds = 6;
overrides.video.renderFrameCount = 60;
overrides.video.fileName = 'senior_geometry_formulation_prediction.mp4';
overrides.video.aloiFileName = 'senior_geometry_aloi_prediction.mp4';

if quickMode
    overrides.numTimeSteps = 6;
end

results = run_rod_plane_force_sensing_experiment(quickMode, overrides);
results.validation = validate_rod_plane_displacement_results(results);
end
