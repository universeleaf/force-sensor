function spec = multi_contact_state_spec(cfg, contactCount, planeMode)
%MULTI_CONTACT_STATE_SPEC Canonical state layout for K contact reactions.
%
% The public estimator currently uses K=1. This helper makes the planned
% multi-contact extension explicit without changing that compatible layout.
% For independent planes, each contact contributes
% [planePoint(3); eta(2); s; fn; beta(m); lambda], followed by one common
% distal force fe(3). With sharedPlane, the first plane block is stored once.

if nargin < 1 || isempty(cfg)
    cfg = run_rod_plane_force_sensing_experiment('sensor-config');
end
if nargin < 2 || isempty(contactCount), contactCount = 1; end
if nargin < 3 || isempty(planeMode), planeMode = 'independent'; end
if ~isscalar(contactCount) || contactCount < 1 || contactCount ~= round(contactCount)
    error('rod:InvalidContactCount', 'contactCount must be a positive integer.');
end
planeMode = lower(char(planeMode));
if ~ismember(planeMode, {'independent','shared'})
    error('rod:InvalidPlaneMode', 'planeMode must be ''independent'' or ''shared''.');
end
if ~isfield(cfg, 'forceSensor') || ~isfield(cfg.forceSensor, 'numFrictionDirs')
    error('rod:InvalidEstimatorConfig', 'cfg.forceSensor.numFrictionDirs is required.');
end
m = cfg.forceSensor.numFrictionDirs;
if ~isscalar(m) || m < 1 || m ~= round(m)
    error('rod:InvalidFrictionDirections', 'numFrictionDirs must be a positive integer.');
end

spec = struct;
spec.contactCount = contactCount;
spec.frictionDirectionCount = m;
spec.planeMode = planeMode;
spec.blockLength = 8 + m;
spec.sharedPlaneLength = 5;
spec.tipForceLength = 3;
spec.contact = repmat(struct('planePoint', [], 'eta', [], 's', [], ...
    'normalForce', [], 'beta', [], 'lambda', [], 'all', []), 1, contactCount);

cursor = 0;
if strcmp(planeMode, 'shared')
    spec.planePoint = 1:3;
    spec.eta = 4:5;
    cursor = spec.sharedPlaneLength;
else
    spec.planePoint = [];
    spec.eta = [];
end

for k = 1:contactCount
    if strcmp(planeMode, 'independent')
        point = cursor + (1:3);
        eta = cursor + (4:5);
        sIdx = cursor + 6;
        fnIdx = cursor + 7;
        betaIdx = cursor + (8:7 + m);
        lambdaIdx = cursor + 8 + m;
        allIdx = cursor + (1:(8 + m));
        cursor = cursor + 8 + m;
    else
        point = spec.planePoint;
        eta = spec.eta;
        sIdx = cursor + 1;
        fnIdx = cursor + 2;
        betaIdx = cursor + (3:(2 + m));
        lambdaIdx = cursor + 3 + m;
        allIdx = cursor + (1:(3 + m));
        cursor = cursor + 3 + m;
    end
    spec.contact(k).planePoint = point;
    spec.contact(k).eta = eta;
    spec.contact(k).s = sIdx;
    spec.contact(k).normalForce = fnIdx;
    spec.contact(k).beta = betaIdx;
    spec.contact(k).lambda = lambdaIdx;
    spec.contact(k).all = allIdx;
end

spec.tipForce = cursor + (1:3);
spec.stateLength = cursor + 3;
spec.description = sprintf('%d-contact %s-plane state; m=%d friction directions.', ...
    contactCount, planeMode, m);

if contactCount == 1 && strcmp(planeMode, 'independent')
    expected = 11 + m;
    assert(spec.stateLength == expected && isequal(spec.contact(1).planePoint, 1:3) && ...
        isequal(spec.tipForce, 9 + m:11 + m), 'rod:StateLayoutRegression', ...
        'The single-contact state layout no longer matches the public API.');
end
end
