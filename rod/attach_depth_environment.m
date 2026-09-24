function [packet,observations] = attach_depth_environment(packet,depthMm,camera,options)
%ATTACH_DEPTH_ENVIRONMENT Replace plane observations using synchronized depth.
% depthMm H-by-W-by-N; camera.timeSeconds must align with packet times.
% K is fixed; TWorldCamera and freeSpacePointMm can be fixed or per frame.
% No stale-plane fallback: any missing/ambiguous frame rejects the batch.
nt=numel(packet.timeSeconds);
assert(isnumeric(depthMm)&&size(depthMm,3)==nt&&ndims(depthMm)<=3, ...
    'rod:DepthSynchronization','Provide one depth image for every packet frame.');
assert(isfield(camera,'timeSeconds')&&numel(camera.timeSeconds)==nt, ...
    'rod:DepthSynchronization','Explicit depth timestamps are required.');
skew=0.005;
if isfield(options,'maxTimeSkewSeconds'), skew=options.maxTimeSkewSeconds; end
times=camera.timeSeconds(:)';
assert(isscalar(skew)&&isfinite(skew)&&skew>=0&&all(isfinite(times))&& ...
    all(diff(times)>0)&&all(abs(times-packet.timeSeconds(:)')<=skew), ...
    'rod:DepthSynchronization','Depth and curvature timestamps do not match the allowed skew.');
assert(all(isfield(camera,{'K','TWorldCamera','freeSpacePointMm'}))&& ...
    (size(camera.TWorldCamera,3)==1||size(camera.TWorldCamera,3)==nt)&& ...
    size(camera.freeSpacePointMm,1)==3&&any(size(camera.freeSpacePointMm,2)==[1 nt]), ...
    'rod:DepthCalibration','Provide fixed or per-frame camera poses and free-space points.');
observations=cell(1,nt);
for k=1:nt
    frame=camera;
    frame.TWorldCamera=camera.TWorldCamera(:,:,min(k,size(camera.TWorldCamera,3)));
    frame.freeSpacePointMm=camera.freeSpacePointMm(:,min(k,size(camera.freeSpacePointMm,2)));
    observations{k}=plane_from_depth(depthMm(:,:,k),frame,options);
end
packet.planePointMm=zeros(3,nt); packet.planeNormal=zeros(3,nt);
packet.planeCovariance=zeros(6,6,nt);
for k=1:nt
    packet.planePointMm(:,k)=observations{k}.planePointMm;
    packet.planeNormal(:,k)=observations{k}.planeNormal;
    packet.planeCovariance(:,:,k)=observations{k}.planeCovariance;
end
packet.environmentTimeSeconds=times;
packet.environmentMaxTimeSkewSeconds=skew;
packet.environmentProvenance='Rectified metric Z-depth images; robust single-plane fit in an explicit ROI, oriented by a calibrated free-space point; camera calibration held fixed.';
end
