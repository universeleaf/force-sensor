function obs = formulation_window_observations(input)
%FORMULATION_WINDOW_OBSERVATIONS Validate shape/environment packets without truth.
% Schema 2: arbitrary sparse arclengths; 2/3 measured curvature channels;
% P planes with a full 6x6 covariance each. Schema 1 is adapted without
% counting duplicate predecessor measurements twice.
assert(isstruct(input)&&isscalar(input)&&all(isfield(input,{'tube','packet','config'})), ...
    'rod:InvalidSensorInput','Expected calibrated tube, packet, config.');
assert(~any(isfield(input,{'truth','forward','results'}))&& ...
    ~any(isfield(input.packet,{'truth','contactForces','contactS','tipForce'})), ...
    'rod:TruthInEstimator','Force/location truth is not an estimator observation.');
cfg=input.config; p=input.packet;
if p.schemaVersion==1
    measurements_from_sensor_packet(input.tube,p,cfg); % existing strict contracts
    % Saved output frames can skip sensor ticks. Keep every supplied paired
    % predecessor, and merge a shared timestamp only once. The existing
    % validator above rejects conflicting observations at a shared time.
    nt=numel(p.timeSeconds); allTimes=[p.timeSeconds(:)' p.previousTimeSeconds(:)'];
    [ordered,order]=sort(allTimes); group=cumsum([1 abs(diff(ordered))>1e-9]);
    uid=zeros(1,2*nt); uid(order)=group; count=max(group);
    allU=cat(3,p.curvaturePerMm,p.previousCurvaturePerMm);
    allB=cat(3,p.basePose,p.previousBasePose);
    representatives=zeros(1,count); owners=representatives;
    environmentLikelihood=false(1,count); predecessorIndex=zeros(1,count);
    for kk=1:count
        current=find(uid(1:nt)==kk,1);
        if ~isempty(current)
            representatives(kk)=current; owners(kk)=current;
            environmentLikelihood(kk)=true; predecessorIndex(kk)=uid(nt+current);
        else
            history=find(uid(nt+1:end)==kk,1);
            representatives(kk)=nt+history; owners(kk)=history;
        end
    end
    q=struct('schemaVersion',2,'sFbgMm',p.sFbgMm,'observedCurvatureAxes',cfg.forceSensor.curvatureObservedAxes, ...
        'curvaturePerMm',allU(:,:,representatives),'basePose',allB(:,:,representatives), ...
        'timeSeconds',allTimes(representatives), ...
        'planePointMm',reshape(p.planePointMm(:,owners),3,1,count), ...
        'planeNormal',reshape(p.planeNormal(:,owners),3,1,count), ...
        'frictionMu',p.frictionMu(owners), ...
        'curvatureStdPerMm',cfg.forceSensor.measurementStd.curvature, ...
        'processReferencePeriodSeconds',p.processReferencePeriodSeconds);
    if isfield(p,'curvatureNoiseStdPerMm'), q.curvatureStdPerMm=max(p.curvatureNoiseStdPerMm,1e-7); end
    if isfield(p,'planeCovariance')
        q.planeCovariance=reshape(p.planeCovariance(:,:,owners),6,6,1,count);
    end
    outputIndices=uid(1:nt);
    % History-only times have no independent camera observation. Their
    % copied geometry initializes the latent state but adds no likelihood.
    p=q;
else
    assert(p.schemaVersion==2,'rod:InvalidSensorPacket','Unsupported packet schema.');
    outputIndices=1:numel(p.timeSeconds); environmentLikelihood=true(size(outputIndices));
    predecessorIndex=[0 outputIndices(1:end-1)];
end
required={'sFbgMm','observedCurvatureAxes','curvaturePerMm','basePose','timeSeconds', ...
    'planePointMm','planeNormal','frictionMu'};
assert(all(isfield(p,required)),'rod:InvalidSensorPacket','Incomplete multi-plane sensor packet.');
assert(isfield(p,'curvatureStdPerMm')||isfield(p,'curvatureCovariance'), ...
    'rod:InvalidCurvatureCovariance','Provide curvature standard deviations or a full covariance.');
t=p.timeSeconds(:)'; T=numel(t); P=size(p.planePointMm,2); arcs=p.sFbgMm(:)'; axes=p.observedCurvatureAxes(:)';
assert(T>=1&&all(isfinite(t))&&all(diff(t)>0),'rod:InvalidSensorTime','Timestamps must increase.');
assert(numel(arcs)>=2&&all(diff(arcs)>0)&&arcs(1)>=input.tube.s(1)&&arcs(end)<=input.tube.s(end), ...
    'rod:InvalidSensorGrid','Sparse calibrated arclengths must be ordered and inside rod.');
assert(ismember(numel(axes),[2 3])&&all(ismember(axes,1:3))&&numel(unique(axes))==numel(axes), ...
    'rod:InvalidSensorAxes','Declare the actual two/three curvature channels.');
u=p.curvaturePerMm;
if size(u,1)==3, u=u(axes,:,:); end
assert(size(u,1)==numel(axes)&&size(u,2)==numel(arcs)&&size(u,3)==T&& ...
    all(isfinite(u),'all'),'rod:InvalidCurvaturePacket','Curvature dimensions must match channels, sensors and times.');
assert(size(p.basePose,1)==4&&size(p.basePose,2)==4&&size(p.basePose,3)==T, ...
    'rod:InvalidBasePose','Base pose must be 4x4xT.');
for k=1:T
    B=p.basePose(:,:,k); R=B(1:3,1:3);
    assert(all(isfinite(B),'all')&&norm(R'*R-eye(3),'fro')<1e-6&&abs(det(R)-1)<1e-6&& ...
        norm(B(4,:)-[0 0 0 1])<1e-10,'rod:InvalidBasePose','Expected a rigid base transform.');
end
assert(size(p.planePointMm,1)==3&&size(p.planePointMm,3)==T&& ...
    isequal(size(p.planePointMm),size(p.planeNormal))&& ...
    all(isfinite(p.planePointMm),'all')&&all(isfinite(p.planeNormal),'all')&& ...
    all(abs(vecnorm(p.planeNormal,2,1)-1)<1e-6,'all'), ...
    'rod:InvalidPlanePacket','Planes must be 3xPxT with unit free-space normals.');
mu=p.frictionMu;
if isscalar(mu), mu=repmat(mu,P,T); end
assert(isequal(size(mu),[P T])&&all(isfinite(mu),'all')&&all(mu>=0,'all'), ...
    'rod:InvalidFrictionPacket','Friction coefficients must be nonnegative PxT.');
nu=numel(axes)*numel(arcs);
if isfield(p,'curvatureCovariance')
    assert(size(p.curvatureCovariance,1)==nu&&size(p.curvatureCovariance,2)==nu&& ...
        size(p.curvatureCovariance,3)==T,'rod:InvalidCurvatureCovariance', ...
        'Full curvature covariance must match channels, sensors and timestamps.');
else
    stdU=p.curvatureStdPerMm;
    assert(isscalar(stdU)||numel(stdU)==nu*T,'rod:InvalidCurvatureCovariance', ...
        'Use a scalar standard deviation or one value per observed channel and timestamp.');
    if isscalar(stdU), stdU=repmat(stdU,nu,T); else, stdU=reshape(stdU,nu,T); end
    assert(isreal(stdU)&&all(isfinite(stdU),'all')&&all(stdU>0,'all'), ...
        'rod:InvalidCurvatureCovariance','Positive curvature standard deviations required.');
end
WU=zeros(nu,nu,T);
for k=1:T
    if isfield(p,'curvatureCovariance'), WU(:,:,k)=whitening(p.curvatureCovariance(:,:,k),nu);
    else, WU(:,:,k)=diag(1./stdU(:,k)); end
end
WE=zeros(6,6,P,T);
for k=1:T
    for j=1:P
        if isfield(p,'planeCovariance')
            assert(size(p.planeCovariance,1)==6&&size(p.planeCovariance,2)==6&& ...
                size(p.planeCovariance,3)==P&&size(p.planeCovariance,4)==T, ...
                'rod:InvalidObservationCovariance','Plane covariance must be 6x6xPxT.');
            C=p.planeCovariance(:,:,j,k);
        else, C=diag([cfg.forceSensor.measurementStd.planePointMm(:);cfg.forceSensor.measurementStd.normalVector(:)].^2); end
        WE(:,:,j,k)=whitening(C,6);
    end
end
period=0.02;
if isfield(p,'processReferencePeriodSeconds'), period=p.processReferencePeriodSeconds; end
assert(isscalar(period)&&isfinite(period)&&period>0,'rod:InvalidProcessPeriod','Positive process reference period required.');
obs=struct('timeSeconds',t,'outputIndices',outputIndices,'frameCount',T,'planeCount',P, ...
    'arcs',arcs,'axes',axes,'u',u,'basePose',p.basePose,'point',p.planePointMm, ...
    'normal',p.planeNormal,'mu',mu,'curvatureWhitening',WU,'environmentWhitening',WE, ...
    'environmentLikelihood',environmentLikelihood,'predecessorIndex',predecessorIndex, ...
    'referencePeriodSeconds',period, ...
    'sourceSchemaVersion',input.packet.schemaVersion);
end
function W=whitening(C,n)
assert(isequal(size(C),[n n])&&isreal(C)&&all(isfinite(C),'all')&&norm(C-C','fro')<1e-10*max(norm(C,'fro'),eps), ...
    'rod:InvalidObservationCovariance','Covariance must be finite and symmetric.');
[L,flag]=chol((C+C')/2,'lower');
assert(flag==0,'rod:InvalidObservationCovariance','Covariance must be positive definite with explicit model floors.');
W=L\eye(n);
end
