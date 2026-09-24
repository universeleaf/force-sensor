function m = measurements_from_sensor_packet(tube, packet, cfg)
%MEASUREMENTS_FROM_SENSOR_PACKET Reconstruct only from supplied observations.
% Calibrated intrinsic shape and stiffness are model inputs. This function
% neither takes nor accesses a forward result, contact truth or true force.
assert(isstruct(packet)&&isscalar(packet)&&isstruct(tube)&&isscalar(tube), ...
    'rod:InvalidSensorInput','Sensor packet and calibrated model must be scalar structs.');
assert(all(isfield(tube,{'s','uhat'}))&&isnumeric(tube.s)&&isreal(tube.s)&& ...
    isvector(tube.s)&&numel(tube.s)>=2&&all(isfinite(tube.s))&& ...
    tube.s(1)==0&&all(diff(tube.s)>0),'rod:InvalidSensorModel','Invalid calibrated arclength grid.');
tube.s=tube.s(:)';
assert(isnumeric(tube.uhat)&&isreal(tube.uhat)&&isequal(size(tube.uhat),[3 numel(tube.s)])&& ...
    all(isfinite(tube.uhat),'all'),'rod:InvalidSensorModel','Invalid intrinsic curvature calibration.');
K=getTubeK(tube);
assert(isreal(K)&&numel(K)==3*numel(tube.s)&&all(isfinite(K))&&all(K>0), ...
    'rod:InvalidSensorModel','Calibrated bending/torsion stiffness must be finite and positive.');
assert(any(strcmp(cfg.sensing.curvatureInterpolation,{'absolute','intrinsic-delta'})), ...
    'rod:InvalidSensorConfig','Unknown curvature interpolation method.');
assert(isnumeric(cfg.sensing.shapeSmoothing)&&isreal(cfg.sensing.shapeSmoothing)&& ...
    isscalar(cfg.sensing.shapeSmoothing)&&isfinite(cfg.sensing.shapeSmoothing)&&cfg.sensing.shapeSmoothing>=0, ...
    'rod:InvalidSensorConfig','Shape smoothing must be finite and nonnegative.');
required={'schemaVersion','fbgIdx','sFbgMm','curvaturePerMm', ...
 'previousCurvaturePerMm','basePose','previousBasePose','timeSeconds', ...
 'previousTimeSeconds','processReferencePeriodSeconds','frictionMu', ...
 'actuationMm','planePointMm','planeNormal'};
assert(all(isfield(packet,required)),'Incomplete sensor packet.');
assert(isnumeric(packet.schemaVersion)&&isscalar(packet.schemaVersion)&&packet.schemaVersion==1, ...
    'Unsupported sensor packet version.');
t=packet.timeSeconds(:)'; old=packet.previousTimeSeconds(:)'; nt=numel(t);
assert(nt>0&&isnumeric(t)&&isnumeric(old)&&isreal([t old])&&all(isfinite([t old]))&&numel(old)==nt&&all(diff(t)>0)&&all(t>old), ...
    'Sensor timestamps must be ordered and history must precede the current sample.');
dt=packet.processReferencePeriodSeconds;
assert(isnumeric(dt)&&isreal(dt)&&isscalar(dt)&&isfinite(dt)&&dt>0,'Invalid process reference period.');
idx=packet.fbgIdx(:)';
assert(isnumeric(idx)&&isreal(idx)&&numel(idx)>=2 && all(isfinite(idx)) && all(idx==round(idx)) && ...
    all(diff(idx)>0) && idx(1)==1 && idx(end)==numel(tube.s), ...
    'Sensor grid must be ordered and include both rod endpoints.');
assert(isnumeric(packet.sFbgMm)&&isreal(packet.sFbgMm)&&numel(packet.sFbgMm)==numel(idx)&& ...
    all(isfinite(packet.sFbgMm(:)))&&max(abs(tube.s(idx)-packet.sFbgMm(:)'))<1e-9, ...
    'Sensor arclength calibration mismatch.');
for name={'curvaturePerMm','previousCurvaturePerMm','basePose','previousBasePose','planePointMm','planeNormal','frictionMu','actuationMm'}
    v=packet.(name{1}); assert(isnumeric(v)&&isreal(v)&&all(isfinite(v(:))), ...
        'Sensor observation must be finite and real: %s',name{1});
end
assert(size(packet.curvaturePerMm,1)==3&&size(packet.curvaturePerMm,2)==numel(idx)&& ...
    size(packet.curvaturePerMm,3)==nt&&numel(packet.curvaturePerMm)==3*numel(idx)*nt, ...
    'Curvature packet dimension mismatch.');
assert(isequal(size(packet.curvaturePerMm),size(packet.previousCurvaturePerMm)), ...
    'History/current curvature dimensions differ.');
assert(size(packet.basePose,1)==4 && size(packet.basePose,2)==4 && ...
    size(packet.basePose,3)==nt && numel(packet.basePose)==16*nt&&isequal(size(packet.basePose),size(packet.previousBasePose)), ...
    'Base-pose packet dimension mismatch.');
assert(isequal(size(packet.planePointMm),[3 nt]) && isequal(size(packet.planeNormal),[3 nt]), ...
    'Plane observations must be 3-by-frame count.');
assert(all(abs(vecnorm(packet.planeNormal)-1)<1e-6),'Plane normals must be unit vectors.');
assert(numel(packet.frictionMu)==nt && all(packet.frictionMu>=0) && ...
    numel(packet.actuationMm)==nt,'Friction/actuation packet dimension or range mismatch.');
if isfield(packet,'environmentTimeSeconds')
    skew=0.005;
    if isfield(packet,'environmentMaxTimeSkewSeconds'), skew=packet.environmentMaxTimeSkewSeconds; end
    env=packet.environmentTimeSeconds(:)';
    assert(isnumeric(skew)&&isreal(skew)&&isscalar(skew)&&isfinite(skew)&&skew>=0&& ...
        isnumeric(env)&&isreal(env)&&numel(env)==nt&&all(isfinite(env))&& ...
        all(diff(env)>0)&&all(abs(env-t)<=skew), ...
        'rod:DepthSynchronization','Saved environment observations exceed the declared time skew.');
end
% A shared timestamp names one physical sample, including its base pose.
% Reject conflicting data rather than treating it as independent history.
allTimes=[t old]; allU=cat(3,packet.curvaturePerMm,packet.previousCurvaturePerMm);
allBase=cat(3,packet.basePose,packet.previousBasePose);
[ordered,order]=sort(allTimes);
for pair=find(abs(diff(ordered))<=1e-9)
    a=order(pair); b=order(pair+1);
    assert(max(abs(allU(:,:,a)-allU(:,:,b)),[],'all')<=1e-12&& ...
        max(abs(allBase(:,:,a)-allBase(:,:,b)),[],'all')<=1e-10, ...
        'rod:ConflictingSensorSample','The same timestamp has conflicting curvature or base pose.');
end
m=struct('fbgIdx',idx,'sFbg',tube.s(idx),'uSparse',packet.curvaturePerMm);
m.u=zeros(3,numel(tube.s),nt); m.p=m.u;
m.pSparse=zeros(3,numel(idx),nt); m.previousShape=cell(1,nt);
for k=1:nt
    current=reconstruct_sensor_curvature(tube,packet.curvaturePerMm(:,:,k),idx,packet.basePose(:,:,k),cfg);
    previous=reconstruct_sensor_curvature(tube,packet.previousCurvaturePerMm(:,:,k),idx,packet.previousBasePose(:,:,k),cfg);
    previous.timeSeconds=old(k);
    m.u(:,:,k)=current.u; m.p(:,:,k)=current.p;
    m.pSparse(:,:,k)=current.p(:,idx); % baseline receives the same FBG information
    m.previousShape{k}=previous;
end
m.planePointMeasured=packet.planePointMm;
m.planeZMeasured=packet.planePointMm(3,:);
m.planeNormalMeasured=packet.planeNormal;
if isfield(packet,'planeCovariance')
    C=packet.planeCovariance;
    assert(isnumeric(C)&&isreal(C)&&size(C,1)==6&&size(C,2)==6&& ...
        size(C,3)==nt&&numel(C)==36*nt&&all(isfinite(C),'all'), ...
        'rod:InvalidPlaneCovariance','Plane covariance must be finite real 6-by-6-by-frame.');
    m.environmentWhitening=zeros(6,6,nt);
    for k=1:nt
        one=C(:,:,k);
        assert(norm(one-one','fro')<=1e-10*max(norm(one,'fro'),eps), ...
            'rod:InvalidPlaneCovariance','Plane covariance must be symmetric.');
        [L,status]=chol((one+one')/2,'lower');
        assert(status==0,'rod:InvalidPlaneCovariance', ...
            'Plane covariance must be positive definite, including explicit model floors.');
        m.environmentWhitening(:,:,k)=L\eye(6);
    end
    m.planeCovariance=C;
end
if isfield(packet,'environmentProvenance'), m.environmentProvenance=packet.environmentProvenance; end
m.baseTraj=packet.basePose;
m.betaMm=packet.actuationMm(:)';
m.frictionMu=packet.frictionMu(:)';
m.timeSeconds=t;
m.historyIntervalSeconds=t-old;
m.predictionIntervalSeconds=[dt,diff(t)];
m.processStepCount=m.predictionIntervalSeconds/dt;
m.historySource='sparse-paired';
m.curvatureInterpolation=cfg.sensing.curvatureInterpolation;
m.description='Sparse current/prior bending measurements, calibrated unloaded profile, base poses and plane observations; no dense truth history.';
m.observedCurvatureAxes=1:3;
if isfield(cfg.forceSensor,'curvatureObservedAxes')
    m.observedCurvatureAxes=cfg.forceSensor.curvatureObservedAxes;
end

end
