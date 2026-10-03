function [input, truth] = build_multi_contact_truth(kind, seed)
%BUILD_MULTI_CONTACT_TRUTH Independent fixed-environment mismatch cases.
% Unlike the historical imposed-load generator, all truth reactions/locations
% solve contact closure and tangency before an observation is withheld or a
% friction parameter is altered for the inverse only.
if nargin<1||isempty(kind), kind='two-contact'; end
if nargin<2||isempty(seed), seed=401; end
kind=char(kind);
if strcmpi(kind,'friction-mismatch')
    [input,truth]=build_spatial_friction_packet(2.5e-5,seed);
    truth.trueFrictionMu=input.packet.frictionMu;
    input.packet.frictionMu(:)=0.01;
    interpretation='Independent spatial sliding equilibrium with true mu=0.03, inverse mu=0.01; friction-model mismatch.';
else
    scenes=multi_contact_demo_scenes(); scene=scenes(1);
    if any(strcmpi(kind,{'curved-surface','nonparallel-planes'}))
        scene=scenes(2);
        interpretation='Independent tapered-channel equilibrium; inverse observes only one face. Two flat facets are not a smooth curved surface.';
    elseif strcmpi(kind,'two-contact')
        interpretation='Independent S-channel equilibrium; inverse observes only one of the two contact faces.';
    else
        error('rod:UnknownMismatchCase','Unknown mismatch kind: %s',kind);
    end
    scene.curvatureNoiseStd=2.5e-5; scene.seed=seed;
    [input,truth]=build_formulation_multi_packet(scene,[3 4]);
end
points=input.packet.planePointMm(:,:,1); normals=input.packet.planeNormal(:,:,1);
truth.planePoint=points(:,1); truth.planeNormal=normals(:,1);
truth.secondaryPlanePoint=points(:,2); truth.secondaryPlaneNormal=normals(:,2);
truth.nonplanarAngleDeg=acosd(min(1,abs(normals(:,1)'*normals(:,2))));
truth.contactForce=reshape(sum(truth.contactForces,2),3,[]);
truth.kind=kind; truth.scope=interpretation;
if ~strcmpi(kind,'friction-mismatch')
    % All unobserved environment fields are removed together. No true
    % contact count, order, location or mode is added to the inverse input.
    input.packet.planePointMm=input.packet.planePointMm(:,1,:);
    input.packet.planeNormal=input.packet.planeNormal(:,1,:);
    input.packet.planeCovariance=input.packet.planeCovariance(:,:,1,:);
    input.packet.frictionMu=input.packet.frictionMu(1,:);
end
end
