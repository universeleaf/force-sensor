function report = test_depth_plane()
%TEST_DEPTH_PLANE Analytic ray/plane fixture independent of the plane fitter.
[u,v]=meshgrid(1:80,1:60); K=[100 0 40.5;0 105 30.5;0 0 1];
n=[0.12;-0.08;-1]; n=n/norm(n); plane=[0;0;160];
rays=K\[u(:)';v(:)';ones(1,numel(u))];
ideal=reshape((n'*plane)./(n'*rays),size(u));
camera=struct('K',K,'TWorldCamera',eye(4),'freeSpacePointMm',[0;0;0]);
options=struct('depthStdMm',0.35,'pointFloorMm',0.1,'normalFloorRad',0.001, ...
    'anchorWorldMm',[0;0;0],'maxPoints',5000,'inlierThresholdMm',1.5);
before=rng; clean=plane_from_depth(ideal,camera,options); after=rng;
assert(isequal(before,after),'Depth fitting changed global random state.');
assert(norm(clean.planeNormal-n)<1e-12&&abs(n'*(clean.planePointMm-plane))<1e-10, ...
    'Camera deprojection, plane gauge or free-space normal sign is wrong.');
stream=RandStream('mt19937ar','Seed',23);
depth=ideal+options.depthStdMm*randn(stream,size(ideal));
order=randperm(stream,numel(depth));
depth(order(1:round(0.2*numel(depth))))=depth(order(1:round(0.2*numel(depth))))+40;
depth(order(round(0.2*numel(depth))+1:round(0.3*numel(depth))))=NaN;
o=plane_from_depth(depth,camera,options);
angle=acosd(max(-1,min(1,n'*o.planeNormal)));
offset=abs(n'*(o.planePointMm-plane));
assert(offset<0.05&&angle<0.1&&o.inlierFraction>0.7, ...
    'Robust plane fit failed with depth outliers and missing pixels.');
assert(min(eig(o.planeCovariance))>0&& ...
    norm(o.planeCovariance(1:3,4:6),'fro')>1e-10,'Plane covariance lost rank or point/normal correlation.');
options2=options; options2.depthStdMm=2*options.depthStdMm;
twice=plane_from_depth(ideal,camera,options2);
assert(norm(twice.conditionalCovariance-4*clean.conditionalCovariance,'fro')<1e-10, ...
    'Plane variance does not scale with calibrated depth variance.');
% A fixed anchor eliminates artificial plane-point motion when ROI changes.
crop=options; crop.roiMask=u>40;
cropped=plane_from_depth(ideal,camera,crop);
assert(norm(cropped.planePointMm-clean.planePointMm)<1e-10,'ROI centroid drift leaked into plane state.');
theta=0.7; R=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)]; t=[30;-15;60];
rotated=camera; rotated.TWorldCamera=[R t;0 0 0 1]; rotated.freeSpacePointMm=t;
rotOptions=options; rotOptions.anchorWorldMm=t;
rot=plane_from_depth(depth,rotated,rotOptions); Q=blkdiag(R,R);
assert(norm(rot.planeNormal-R*o.planeNormal)<1e-10&& ...
    norm(rot.planePointMm-(R*o.planePointMm+t))<1e-8&& ...
    norm(rot.planeCovariance-Q*o.planeCovariance*Q','fro')<1e-8, ...
    'World transform or full covariance is not rotation equivariant.');
mustReject(@()plane_from_depth(nan(size(depth)),camera,options),'rod:DepthPlaneUnavailable');
narrow=options; narrow.roiMask=v==30;
mustReject(@()plane_from_depth(ideal,camera,narrow),'rod:DepthPlaneUnavailable');
ambiguous=camera; ambiguous.freeSpacePointMm=plane;
mustReject(@()plane_from_depth(ideal,ambiguous,options),'rod:DepthNormalAmbiguous');
bad=camera; bad.TWorldCamera(1,1)=-1;
mustReject(@()plane_from_depth(ideal,bad,options),'rod:DepthCalibration');
packet=struct('timeSeconds',[1 2]); camera.timeSeconds=[1 2];
[packet,~]=attach_depth_environment(packet,cat(3,depth,ideal),camera,options);
assert(size(packet.planeCovariance,3)==2&&all(packet.planeNormal(3,:)<0), ...
    'Depth adapter dropped covariance or reversed normal orientation.');
camera.timeSeconds=[0.9 2];
mustReject(@()attach_depth_environment(packet,cat(3,depth,ideal),camera,options),'rod:DepthSynchronization');
report=struct('offsetErrorMm',offset,'normalErrorDeg',angle,'inlierFraction',o.inlierFraction,'passed',true);
disp(report); disp('DEPTH_PLANE_TEST_PASSED');
end

function mustReject(f,id)
try, f(); catch err, assert(strcmp(err.identifier,id),'Unexpected error: %s',err.message); return; end
error('Expected rejection: %s',id);
end
