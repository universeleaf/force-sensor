function estimator = sensor_estimator_config(cfg)
%SENSOR_ESTIMATOR_CONFIG Allowlist model/solver inputs; omit simulation loads.
estimator.forceSensor=cfg.forceSensor;
if ~isfield(estimator.forceSensor,'historyStateMode')
    estimator.forceSensor.historyStateMode='fixed';
end
if ~isfield(estimator.forceSensor,'historyPointInterpolation')
    estimator.forceSensor.historyPointInterpolation='linear';
end
if ~isfield(estimator.forceSensor,'historyCurvatureStdPerMm') || ...
        isempty(estimator.forceSensor.historyCurvatureStdPerMm)
    estimator.forceSensor.historyCurvatureStdPerMm=0;
    if isfield(cfg.sensing,'curvatureNoiseStd')
        estimator.forceSensor.historyCurvatureStdPerMm=cfg.sensing.curvatureNoiseStd;
    end
end
assert(isscalar(estimator.forceSensor.historyCurvatureStdPerMm) && ...
    isfinite(estimator.forceSensor.historyCurvatureStdPerMm) && ...
    estimator.forceSensor.historyCurvatureStdPerMm>=0,'Invalid history curvature standard deviation.');
if ~isfield(estimator.forceSensor,'slipConfidenceSigma')
    estimator.forceSensor.slipConfidenceSigma=3;
end
assert(isscalar(estimator.forceSensor.slipConfidenceSigma) && ...
    isfinite(estimator.forceSensor.slipConfidenceSigma) && ...
    estimator.forceSensor.slipConfidenceSigma>0,'Invalid slip confidence multiplier.');
% Saved pre-seeding packets must replay with their original initialization.
if ~isfield(estimator.forceSensor,'useShapeOnlySeed')
    estimator.forceSensor.useShapeOnlySeed=false;
end
estimator.sensing=struct('numFbgPoints',cfg.sensing.numFbgPoints, ...
    'curvatureInterpolation',cfg.sensing.curvatureInterpolation, ...
    'shapeSmoothing',cfg.sensing.shapeSmoothing);
end
