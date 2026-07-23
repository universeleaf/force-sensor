function results = validate_rod_plane_displacement_inverse()
%VALIDATE_ROD_PLANE_DISPLACEMENT_INVERSE Strict six-frame inverse check.

packageDir = fileparts(mfilename('fullpath'));
rootDir = packageDir;
if ~exist(fullfile(rootDir, 'LCP-Continuum'), 'dir')
    rootDir = fileparts(packageDir);
end
overrides = struct;
overrides.outputDir = fullfile(rootDir, 'force_outputs', ...
    'rod_plane_displacement_inverse_validation_tmp');
overrides.exposedLengthMm = 150;
overrides.scalePrecurvature = false;
overrides.basePositionMm = [-120; 0; 0];
overrides.planePointMm = [20; 0; 0];
overrides.planeNormal = [-1; 0; 0];
overrides.frictionMu = 0.5;
overrides.initialLowFrictionMu = 0.0;
overrides.tipLoadN = zeros(3, 1);
overrides.numTimeSteps = 6;

overrides.forward.motionMode = 'push-slide';
overrides.forward.pushDistanceMm = 30;
overrides.forward.slideDistanceMm = 1;
overrides.forward.pushFraction = 0.5;
overrides.forward.slideDirection = [0; 0; 1];

overrides.video.enabled = false;
overrides.sensing.numFbgPoints = 24;
overrides.forceSensor.maxEkfIterations = 2;
overrides.forceSensor.solver = 'fmincon';
overrides.forceSensor.allowApproximateFallback = false;
overrides.forceSensor.useMultiStart = false;
overrides.forceSensor.maxStartCandidates = 1;
overrides.diagnostics.runMapCandidateCosts = true;

results = run_rod_plane_force_sensing_experiment(false, overrides);
results.validation = validate_rod_plane_displacement_results(results);
end
