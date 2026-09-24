function info = friction_mode_resolution(displacement,directions,covariance,toleranceMm,sigmaMultiplier)
%FRICTION_MODE_RESOLUTION Can one friction ray be selected from noisy slip?
% A radius-k covariance ellipsoid must lie inside the selected ray's
% minimum-work Voronoi region. Testing only displacement magnitude does
% not establish the direction needed by an exact active-set constraint.
[~,active]=min(directions'*displacement);
competitors=setdiff(1:size(directions,2),active);
contrast=directions(:,competitors)-directions(:,active);
margin=contrast'*displacement;
variance=sum(contrast.*(covariance*contrast),1)';
noiseBound=sigmaMultiplier*sqrt(max(0,variance));
stdMax=sqrt(max(0,max(eig((covariance+covariance')/2))));
info=struct('activeDirection',active,'observedSlipMm',norm(displacement), ...
    'pairedSlipStdMm',stdMax,'slipResolutionMm',toleranceMm+sigmaMultiplier*stdMax, ...
    'directionResolved',all(margin>noiseBound), ...
    'minimumDirectionMarginMm',min([inf;margin-noiseBound]));
info.slidingResolved=info.observedSlipMm>info.slipResolutionMm && info.directionResolved;
end
