function result = evaluate_multi_contact_state(tube, x, cfg, contactCount, planeMode, options)
%EVALUATE_MULTI_CONTACT_STATE Forward mechanics and audit for a K-contact state.
% This is the mechanics kernel for the future multi-contact MAP. It evaluates
% one candidate state with the same nonlinear Cosserat shooting map used by
% the independent multi-contact truth generator. It deliberately does not
% optimize x, so callers cannot mistake a forward candidate for an estimate.

if nargin < 4 || isempty(contactCount), contactCount = 1; end
if nargin < 5 || isempty(planeMode), planeMode = 'independent'; end
if nargin < 6 || isempty(options), options = struct; end
decoded = decode_multi_contact_state(x, tube, cfg, contactCount, planeMode);
if any(diff(decoded.contactArcLength) <= 0)
    error('rod:InvalidContactOrdering', 'Contact arc lengths must be strictly increasing.');
end

if ~isfield(options,'relativeTolerance'), options.relativeTolerance = cfg.forceSensor.mechanicsRelativeTolerance; end
if ~isfield(options,'momentToleranceNmm'), options.momentToleranceNmm = cfg.forceSensor.mechanicsMomentToleranceNmm; end
if ~isfield(options,'collisionStepMm'), options.collisionStepMm = cfg.forceSensor.planeCollisionStepMm; end
if ~isfield(options,'maxRhsEvaluations'), options.maxRhsEvaluations = cfg.forceSensor.mechanicsMaxRhsEvaluations; end
mechanics = solve_cosserat_multi_contact_map(tube, decoded.contactArcLength, ...
    decoded.contactForce, decoded.tipForce, options);
[c, ceq, diagnostics] = multi_contact_constraints(decoded, mechanics, cfg);

fbgIdx = unique(round(linspace(1, numel(tube.s), cfg.sensing.numFbgPoints)));
axesObserved = [1 2];
if isfield(cfg.forceSensor,'curvatureObservedAxes')
    axesObserved = cfg.forceSensor.curvatureObservedAxes;
end
environment = zeros(6, contactCount);
for k = 1:contactCount
    environment(:,k) = [decoded.contacts(k).planePoint(:); decoded.contacts(k).normal(:)];
end
result = struct;
result.state = x(:);
result.decoded = decoded;
result.mechanics = mechanics;
result.predictedCurvature = mechanics.u(axesObserved, fbgIdx);
result.fbgIndices = fbgIdx;
result.environment = environment;
result.constraints = struct('c',c,'ceq',ceq,'diagnostics',diagnostics);
result.contactForce = decoded.contactForce;
result.tipForce = decoded.tipForce;
result.totalForce = sum(decoded.contactForce,2) + decoded.tipForce;
result.mechanicsResidualNmm = mechanics.tipMomentResidualNmm;
result.scope = 'Forward candidate evaluation; no inverse optimization or measurement fit performed.';
end
