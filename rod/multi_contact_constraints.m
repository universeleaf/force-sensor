function [c, ceq, diagnostics] = multi_contact_constraints(decoded, mechanics, cfg, minArcSeparationMm)
%MULTI_CONTACT_CONSTRAINTS Shared admissibility checks for K contacts.
% c<=0 and ceq=0 follow the fmincon/MPCC convention. The same routine can
% be called after an optimization to audit the returned state.

if nargin < 4 || isempty(minArcSeparationMm), minArcSeparationMm = 1e-3; end
K = decoded.contactCount;
if isfield(cfg.forceSensor,'frictionMu')
    mu = cfg.forceSensor.frictionMu;
elseif isfield(cfg,'frictionMu')
    mu = cfg.frictionMu;
else
    mu = 0;
end
if isscalar(mu), mu = repmat(mu, 1, K); end
if numel(mu) ~= K
    error('rod:InvalidFrictionSchedule', 'Need one friction coefficient per contact.');
end
if ~isstruct(mechanics) || ~isfield(mechanics,'contactPoints')
    error('rod:MissingMultiContactMechanics', 'mechanics.contactPoints is required.');
end
if size(mechanics.contactPoints,2) ~= K
    error('rod:ContactCountMismatch', 'Mechanics and state contact counts differ.');
end

c = [];
ceq = [];
gap = nan(1,K);
coneSlack = nan(1,K);
minW = nan(1,K);
contactPoint = mechanics.contactPoints;
sampledMinGap = nan(1,K);
for k = 1:K
    q = decoded.contacts(k);
    gap(k) = q.normal' * (contactPoint(:,k) - q.planePoint(:));
    coneSlack(k) = mu(k)*q.normalForce - sum(q.beta);
    if isfield(mechanics,'collisionP') && isfield(mechanics,'collisionS')
        sampled = q.normal' * (mechanics.collisionP - q.planePoint(:));
        sampledMinGap(k) = min([sampled, gap(k)]);
        c = [c; -sampled(:)]; %#ok<AGROW>
    else
        sampledMinGap(k) = gap(k);
    end
    % A static state has no measured tangential displacement. Keeping the
    % lambda term explicit makes this a valid static cone audit; a temporal
    % wrapper must add D'v before using friction complementarity.
    w = q.lambda * ones(size(q.beta));
    minW(k) = min(w);
    c = [c; -gap(k); -q.beta(:); -coneSlack(k); -q.normalForce]; %#ok<AGROW>
    ceq = [ceq; gap(k)*q.normalForce; w(:).*q.beta(:); ...
        coneSlack(k)*q.lambda]; %#ok<AGROW>
end

s = decoded.contactArcLength;
if K > 1
    c = [c; minArcSeparationMm - diff(s(:))];
end
diagnostics = struct('gap',gap,'coneSlack',coneSlack,'minFrictionW',minW, ...
    'contactPoint',contactPoint,'sampledMinGapMm',sampledMinGap, ...
    'orderedArcLength',all(diff(s)>minArcSeparationMm), ...
    'maxEqualityResidual',max([0;abs(ceq)]), ...
    'maxInequalityViolation',max([0;c]));
end
