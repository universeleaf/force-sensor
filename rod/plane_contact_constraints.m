function [c,ceq,diagnostic] = plane_contact_constraints(d,cfg)
%PLANE_CONTACT_CONSTRAINTS Rigid-plane admissibility beyond a candidate gap.
% The PDF's point MPCC alone permits another part of the centerline to cross
% the plane. Check a fixed arclength grid of the continuous mechanics solution.
% A positive-force smooth INTERIOR contact must also be a stationary gap.
% The force/endpoint factors keep separation and endpoint contact admissible.
% This is a zero-radius, single-contact model; sampling is not a proof of
% continuous nonpenetration between every pair of samples.
c=[]; ceq=[]; diagnostic=struct;
enabled=isfield(cfg,'planeContactGeometry')&&strcmp(cfg.planeContactGeometry,'rod');
if ~enabled && nargout<3,return;end
if isfield(d.mechanics,'collisionP')
    gaps=d.n'*(d.mechanics.collisionP-d.p1);
    bounds=d.mechanics.rodArcBoundsMm;
    fraction=(d.s1-bounds(1))/diff(bounds);
    interiorWeight=4*fraction*(1-fraction);
    normalTangent=d.n'*d.mechanics.contactTangent;
    diagnostic=struct('enforced',enabled,'minimumSampledGapMm',min([gaps,d.gap]), ...
        'sampleSpacingMm',max(diff(d.mechanics.collisionS)), ...
        'contactNormalTangent',normalTangent, ...
        'weightedTangencyResidualN',abs(d.fn*interiorWeight*normalTangent));
elseif enabled
    error('rod:MissingPlaneGeometry','Rod contact requires continuous Cosserat geometry.');
else
    % Legacy outputs can still expose nodal penetration without claiming
    % that a continuous centerline was checked.
    diagnostic=struct('enforced',false,'minimumSampledGapMm',min([d.n'*(d.p-d.p1),d.gap]), ...
        'sampleSpacingMm',NaN,'contactNormalTangent',NaN,'weightedTangencyResidualN',NaN);
    return;
end
if enabled
    c=-gaps(:)/cfg.mpccLengthScaleMm;
    ceq=d.fn/cfg.mpccForceScaleN*interiorWeight*normalTangent;
end
end
