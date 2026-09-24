function output = estimate_sensor_forces(sensorInput)
%ESTIMATE_SENSOR_FORCES Public estimation API: calibrated model + observations.
% No simulation truth accepted. Quality flags describe numerical consistency,
% not force accuracy, identifiability, uncertainty coverage or safety.
assert(isstruct(sensorInput) && all(isfield(sensorInput,{'tube','packet','config'})), ...
    'Provide sensorInput with tube, packet and estimator config.');
assert(~any(isfield(sensorInput,{'forward','truth','results'})), ...
    'Truth/scoring containers do not belong in the estimator input.');
output=run_rod_plane_force_sensing_experiment('replay-sensors',sensorInput);
o=output.ours; nt=size(o.state,2);
q=struct('scope','Numerical and observation consistency only; force accuracy and calibrated uncertainty are not certified.', ...
    'hasOptimizationWarning',false(1,nt),'shapeConsistencyMm',o.shapeRmseMm, ...
    'complementarityResidual',max([o.normalComplementarity;o.frictionComplementarity;o.coneComplementarity],[],1), ...
    'shapeConsistent',o.shapeRmseMm<0.5,'finite',false(1,nt));
for k=1:nt
    trace=o.solverTrace{k};
    flags=cellfun(@(v)v.exitflag,trace);
    q.hasOptimizationWarning(k)=isempty(flags)||any(~isfinite(flags) | flags<=0);
    q.finite(k)=all(isfinite(o.state(:,k))) && all(isfinite(o.p(:,:,k)),'all');
end
q.frictionKinematicsEnforced=o.frictionKinematicsEnforced;
q.frictionKinematicsConsistent=o.frictionKinematicsEnforced & ...
    q.complementarityResidual<1e-3 & o.frictionWMin>=-1e-5;
direction=friction_direction_quality(o,q.frictionKinematicsConsistent);
q.frictionDirectionResolutionAssessed=direction.frictionDirectionResolutionAssessed;
q.frictionDirectionResolved=direction.frictionDirectionResolved;
q.hasFrictionObservationWarning=direction.hasFrictionObservationWarning;
q.activeConstraintResidual=o.activeConstraintResidual;
q.minimumSampledRodGapMm=cellfun(@(d)d.planeGeometry.minimumSampledGapMm,o.mechanicsDiagnostics);
q.rodGeometryEnforced=cellfun(@(d)d.planeGeometry.enforced,o.mechanicsDiagnostics);
q.hasRodPenetration=q.minimumSampledRodGapMm < -1e-5;
q.numericalChecksPassed=q.finite & q.shapeConsistent & q.activeConstraintResidual<1e-3 & ...
    o.frictionConeViolation<1e-4 & o.gap>=-1e-5;
q.forceComponentsLocallySeparable=cellfun(@(d)d.locallySeparable,o.forceSensitivity);
q.forceSeparationApplicable=~strcmp(o.complementarityMode,'no-contact');
q.hasForceSeparationWarning=q.forceSeparationApplicable & ~q.forceComponentsLocallySeparable;
q.requiresReview=~q.numericalChecksPassed | q.hasOptimizationWarning | ...
    ~q.frictionKinematicsEnforced | q.hasForceSeparationWarning | q.hasRodPenetration | ...
    q.hasFrictionObservationWarning;
q.forceUncertaintyCoverageValidated=false;
q.forceAccuracyCertified=false;
% Noise calibration does not mean latent history is in the MAP posterior.
q.historyUncertaintyModeled=o.historyUncertaintyModeled;
q.historyStateOptimized=~cellfun(@isempty,o.historyEstimate);
q.historyModeResolutionModeled=o.historyCurvatureStdPerMm>0;
q.historyNoiseCalibrationAvailable=o.historyCurvatureStdPerMm>0;
q.contactModeOptimized=false;
if strcmp(output.config.forceSensor.complementaritySolver,'scholtes')
    q.contactModeOptimized=true;
    q.historyModeResolutionModeled=false;
end
q.environmentCovarianceUsed=o.environmentCovarianceUsed;
q.environmentCalibrationUncertaintyModeled=false;
q.uncertaintyScope=o.uncertaintyScope;
if q.contactModeOptimized && ~q.historyUncertaintyModeled
    q.uncertaintyScope=['Local first-order observation covariance conditional on reconstructed history. ', ...
        'Full contact mode optimized; history noise is not included in the MAP likelihood, ', ...
        'and history states and mode mixtures are not marginalized.'];
end
output.quality=q;
end
