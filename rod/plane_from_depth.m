function observation = plane_from_depth(depthMm,camera,options)
%PLANE_FROM_DEPTH Robust plane observation from a rectified metric depth image.
% depthMm is optical-axis Z (not ray range); K uses MATLAB 1-based pixels.
% camera: K, TWorldCamera (translation mm), freeSpacePointMm (world).
% options: depthStdMm, pointFloorMm, normalFloorRad, anchorWorldMm required.
% The anchor is fixed across frames: p1 is its orthogonal plane projection,
% not a changing ROI centroid. No true plane, shape or force is accepted.
% Covariance order [p1(3);n(3)] retains cross-correlation. Conditional on
% calibration, selected inliers and iid depth noise; floors are explicit
% model regularization, not estimated camera-calibration uncertainty.
required={'depthStdMm','pointFloorMm','normalFloorRad','anchorWorldMm'};
assert(nargin==3 && isstruct(options)&&all(isfield(options,required)), ...
    'rod:DepthCalibration','Supply depth noise, positive model floors and a fixed world anchor.');
defaults=struct('inlierThresholdMm',1,'minInlierRatio',0.6,'minPoints',40, ...
    'minSpreadMm',2,'maxPoints',4000,'ransacTrials',200,'seed',17, ...
    'minDepthMm',1,'maxDepthMm',10000,'maxTimeSkewSeconds',0.005);
names=fieldnames(defaults);
for k=1:numel(names)
    if ~isfield(options,names{k}), options.(names{k})=defaults.(names{k}); end
end
for name={'depthStdMm','pointFloorMm','normalFloorRad','inlierThresholdMm', ...
        'minSpreadMm','minDepthMm','maxDepthMm'}
    v=options.(name{1});
    assert(isscalar(v)&&isreal(v)&&isfinite(v)&&v>0,'rod:DepthCalibration', ...
        'Invalid positive calibration: %s.',name{1});
end
for name={'minPoints','maxPoints','ransacTrials','seed'}
    v=options.(name{1});
    assert(isscalar(v)&&isfinite(v)&&v==round(v)&&v>=0,'rod:DepthCalibration','Invalid integer option.');
end
assert(options.minPoints>=4 && options.maxPoints>=options.minPoints && options.ransacTrials>0 && ...
    isscalar(options.minInlierRatio)&&isfinite(options.minInlierRatio)&& ...
    options.minInlierRatio>0.5&&options.minInlierRatio<=1&&options.maxDepthMm>options.minDepthMm, ...
    'rod:DepthCalibration','Invalid plane support thresholds.');
assert(isnumeric(depthMm)&&isreal(depthMm)&&ismatrix(depthMm)&&~isempty(depthMm), ...
    'rod:DepthCalibration','Depth must be a nonempty real Z-depth image in mm.');
assert(isstruct(camera)&&all(isfield(camera,{'K','TWorldCamera','freeSpacePointMm'})), ...
    'rod:DepthCalibration','Camera intrinsics, pose and a known free-space point are required.');
K=camera.K; T=camera.TWorldCamera; a=options.anchorWorldMm(:); free=camera.freeSpacePointMm(:);
assert(isequal(size(K),[3 3])&&isreal(K)&&all(isfinite(K),'all')&& ...
    K(1,1)>0&&K(2,2)>0&&norm(K(3,:)-[0 0 1])<1e-12&&abs(K(2,1))<1e-12, ...
    'rod:DepthCalibration','K must be a pinhole intrinsic matrix in 1-based pixels.');
assert(isequal(size(T),[4 4])&&isreal(T)&&all(isfinite(T),'all')&& ...
    norm(T(4,:)-[0 0 0 1])<1e-10&&norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')<1e-8&& ...
    abs(det(T(1:3,1:3))-1)<1e-8,'rod:DepthCalibration','Camera-to-world pose must be a proper rigid transform.');
assert(numel(a)==3&&numel(free)==3&&isreal([a;free])&&all(isfinite([a;free])), ...
    'rod:DepthCalibration','Anchor and free-space point must be finite world 3-vectors.');
mask=isfinite(depthMm)&depthMm>=options.minDepthMm&depthMm<=options.maxDepthMm;
if isfield(options,'roiMask')
    assert(islogical(options.roiMask)&&isequal(size(options.roiMask),size(depthMm)), ...
        'rod:DepthCalibration','ROI must be a logical mask of the depth image size.');
    mask=mask&options.roiMask;
end
pixels=find(mask); validCount=numel(pixels);
assert(validCount>=options.minPoints,'rod:DepthPlaneUnavailable','Too few valid ROI depth samples.');
if validCount>options.maxPoints
    pixels=pixels(unique(round(linspace(1,validCount,options.maxPoints))));
end
[v,u]=ind2sub(size(depthMm),pixels);
rays=T(1:3,1:3)*(K\[u';v';ones(1,numel(u))]);
points=rays.*double(depthMm(pixels))'+T(1:3,4);
stream=RandStream('mt19937ar','Seed',options.seed);
bestCount=0; bestCost=inf; keep=false(1,numel(pixels));
for trial=1:options.ransacTrials
    ids=randperm(stream,numel(pixels),3);
    delta=points(:,ids(2:3))-points(:,ids(1)); n=cross(delta(:,1),delta(:,2));
    if norm(n)<1e-10, continue; end
    n=n/norm(n); residual=abs(n'*(points-points(:,ids(1))));
    inlier=residual<=options.inlierThresholdMm; count=sum(inlier);
    cost=sum(min(residual.^2,options.inlierThresholdMm^2));
    if count>bestCount || (count==bestCount && cost<bestCost)
        keep=inlier; bestCount=count; bestCost=cost;
    end
end
assert(bestCount>=options.minPoints && bestCount/numel(pixels)>=options.minInlierRatio, ...
    'rod:DepthPlaneUnavailable','No dominant plane supported by the ROI.');
% Weighted orthogonal refinement. Depth noise projects along each pixel ray.
weights=ones(1,sum(keep));
for iter=1:5
    P=points(:,keep); centroid=P*weights'/sum(weights);
    scatter=(P-centroid).*sqrt(weights);
    [U,S,~]=svd(scatter,'econ'); n=U(:,end);
    residual=abs(n'*(points-centroid));
    keep=residual<=options.inlierThresholdMm;
    assert(sum(keep)>=options.minPoints && mean(keep)>=options.minInlierRatio, ...
        'rod:DepthPlaneUnavailable','Plane support vanished during refinement.');
    variance=(options.depthStdMm*(n'*rays(:,keep))).^2;
    weights=1./max(variance,1e-12);
end
% Refit the final inlier set once so covariance and estimate use the same data.
P=points(:,keep); centroid=P*weights'/sum(weights);
[U,S,~]=svd((P-centroid).*sqrt(weights),'econ'); n=U(:,end);
spread=diag(S)/sqrt(sum(weights));
assert(spread(2)>=options.minSpreadMm,'rod:DepthPlaneUnavailable', ...
    'Plane support is too narrow or collinear to determine its normal.');
side=n'*(free-centroid);
assert(abs(side)>options.inlierThresholdMm,'rod:DepthNormalAmbiguous', ...
    'Known free-space point is too close to the fitted plane to orient its normal.');
if side<0, n=-n; end
B=null(n'); d=n'*(centroid-a); point=a+d*n;
J=[(P-a)'*B,-ones(sum(keep),1)];
variance=max((options.depthStdMm*(n'*rays(:,keep))).^2,1e-12);
weightedJ=J./sqrt(variance(:)); info=weightedJ'*weightedJ;
assert(rcond(info)>1e-12,'rod:DepthPlaneUnavailable','Plane information matrix is degenerate.');
residual=n'*(P-point);
inflation=max(1,sum(residual.^2./variance)/max(sum(keep)-3,1));
parameterCovariance=inflation*(info\eye(3));
G=[d*B,n;B,zeros(3,1)];
conditional=G*parameterCovariance*G';
covariance=conditional+diag([options.pointFloorMm^2*ones(1,3),options.normalFloorRad^2*ones(1,3)]);
inlierMask=false(size(depthMm)); inlierMask(pixels(keep))=true;
observation=struct('planePointMm',point,'planeNormal',n, ...
    'planeCovariance',(covariance+covariance')/2,'conditionalCovariance',conditional, ...
    'inlierMask',inlierMask,'inlierFraction',mean(keep),'inlierCount',sum(keep), ...
    'validPixelCount',validCount,'sampleCount',numel(pixels), ...
    'residualRmseMm',sqrt(mean(residual.^2)),'supportSpreadMm',spread(1:2), ...
    'pointFloorMm',options.pointFloorMm,'normalFloorRad',options.normalFloorRad, ...
    'covarianceScope','Local iid Z-depth fit, fixed intrinsics/extrinsics and selected inliers, plus explicit model floors; not calibrated total uncertainty.');
end
