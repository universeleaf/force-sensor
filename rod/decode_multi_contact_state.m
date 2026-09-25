function decoded = decode_multi_contact_state(x, tube, cfg, contactCount, planeMode)
%DECODE_MULTI_CONTACT_STATE Decode the canonical K-contact force state.
% This is a geometry/force decoder only. It does not silently solve an
% inverse problem or project infeasible values; constraints are returned by
% multi_contact_constraints. A mechanics evaluation can use the decoded
% contact forces with solve_cosserat_multi_contact_map.

if nargin < 4 || isempty(contactCount), contactCount = 1; end
if nargin < 5 || isempty(planeMode), planeMode = 'independent'; end
spec = multi_contact_state_spec(cfg, contactCount, planeMode);
x = x(:);
if numel(x) ~= spec.stateLength
    error('rod:InvalidMultiContactState', 'Expected %d state entries, received %d.', ...
        spec.stateLength, numel(x));
end
if ~isstruct(tube) || ~isfield(tube,'s')
    error('rod:InvalidTube', 'tube.s is required.');
end

decoded = struct('state', x, 'spec', spec, 'contactCount', contactCount, ...
    'planeMode', planeMode, 'tipForce', x(spec.tipForce), 'contacts', []);
contacts = repmat(struct('planePoint', [], 'eta', [], 'normal', [], ...
    's', [], 'normalForce', [], 'beta', [], 'lambda', [], ...
    'directions', [], 'force', [], 'frictionForce', [], 'normalVectorForce', []), ...
    1, contactCount);

if strcmpi(planeMode, 'shared')
    commonPoint = x(spec.planePoint);
    commonEta = x(spec.eta);
end
for k = 1:contactCount
    idx = spec.contact(k);
    if strcmpi(planeMode, 'shared')
        point = commonPoint;
        eta = commonEta;
    else
        point = x(idx.planePoint);
        eta = x(idx.eta);
    end
    n = local_eta_to_normal(eta, cfg);
    beta = max(0, x(idx.beta));
    fn = max(0, x(idx.normalForce));
    D = local_friction_directions(n, cfg.forceSensor.numFrictionDirs);
    fNormal = n * fn;
    fFriction = D * beta;
    contacts(k).planePoint = point;
    contacts(k).eta = eta;
    contacts(k).normal = n;
    contacts(k).s = min(max(x(idx.s), tube.s(1)), tube.s(end));
    contacts(k).normalForce = fn;
    contacts(k).beta = beta;
    contacts(k).lambda = max(0, x(idx.lambda));
    contacts(k).directions = D;
    contacts(k).force = fNormal + fFriction;
    contacts(k).frictionForce = fFriction;
    contacts(k).normalVectorForce = fNormal;
end
decoded.contacts = contacts;
decoded.contactArcLength = reshape([contacts.s], 1, []);
decoded.contactForce = reshape([contacts.force], 3, []);
decoded.normalForce = [contacts.normalForce];
decoded.frictionForce = reshape([contacts.frictionForce], 3, []);
decoded.lambda = [contacts.lambda];
end

function n = local_eta_to_normal(eta, cfg)
if isfield(cfg.forceSensor,'normalReference')
    reference = cfg.forceSensor.normalReference;
elseif isfield(cfg,'planeNormal')
    reference = cfg.planeNormal;
else
    reference = [0;0;1];
end
n0 = local_unit_vector(reference, [0;0;1]);
B = local_stable_tangent_basis(n0);
t = B * eta(:);
theta = norm(t);
if theta < 1e-10
    n = n0;
else
    n = local_unit_vector(cos(theta)*n0 + sin(theta)*t/theta, n0);
end
end

function D = local_friction_directions(n, m)
n = local_unit_vector(n, [0;0;1]);
t1 = local_contact_tangent(n);
if m == 1
    D = t1;
elseif m == 2
    D = [t1, -t1];
else
    B = local_stable_tangent_basis(n);
    theta = linspace(0, 2*pi, m+1);
    theta = theta(1:end-1);
    D = B * [cos(theta); sin(theta)];
end
end

function B = local_stable_tangent_basis(n)
n = local_unit_vector(n, [0;0;1]);
t1 = local_contact_tangent(n);
t2 = local_unit_vector(cross(n,t1), [0;1;0]);
B = [t1,t2];
end

function t = local_contact_tangent(n)
ref = [1;0;0];
if abs(ref'*n) > 0.9, ref = [0;1;0]; end
t = local_unit_vector(ref - n*(n'*ref), [1;0;0]);
end

function v = local_unit_vector(v, fallback)
v = v(:);
if norm(v) < 1e-12, v = fallback(:); end
v = v / max(norm(v), eps);
end
