function q = subset_formulation_packet(p, indices)
%SUBSET_FORMULATION_PACKET Keep every time-dependent schema-2 field aligned.
assert(p.schemaVersion==2&&all(indices==round(indices))&&all(diff(indices)>0)&& ...
    all(indices>=1)&&all(indices<=numel(p.timeSeconds)),'rod:InvalidPacketSubset','Invalid frame selection.');
q=p; q.timeSeconds=p.timeSeconds(indices); q.basePose=p.basePose(:,:,indices);
q.curvaturePerMm=p.curvaturePerMm(:,:,indices);
q.planePointMm=p.planePointMm(:,:,indices); q.planeNormal=p.planeNormal(:,:,indices);
if ~isscalar(p.frictionMu), q.frictionMu=p.frictionMu(:,indices); end
if isfield(p,'planeCovariance'), q.planeCovariance=p.planeCovariance(:,:,:,indices); end
if isfield(p,'curvatureCovariance'), q.curvatureCovariance=p.curvatureCovariance(:,:,indices); end
if isfield(p,'curvatureStdPerMm')&&~isscalar(p.curvatureStdPerMm)
    stdv=reshape(p.curvatureStdPerMm,[],numel(p.timeSeconds)); q.curvatureStdPerMm=stdv(:,indices);
end
end
