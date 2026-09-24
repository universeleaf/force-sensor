function packet = simulate_sensor_packet(tube, forward, cfg)
%SIMULATE_SENSOR_PACKET Sensor simulation boundary; no dense shape/load export.
% Paired samples are one sensor tick apart. Filter outputs may skip ticks.
% Only selected curvature samples, commanded base poses and environment
% observations cross this boundary. Dense p/R and all loads are excluded.
ns=numel(tube.s); nt=numel(forward.betaMm);
idx=unique(round(linspace(1,ns,cfg.sensing.numFbgPoints)));
ticks=forward.internalSampleIndices(:)';
dt=cfg.sensing.samplePeriodSeconds;
assert(isscalar(dt)&&isfinite(dt)&&dt>0,'Sensor period must be positive.');
stream=RandStream('mt19937ar','Seed',cfg.randomSeed);
allTicks=unique([ticks,ticks-1]);
samples=zeros(3,numel(idx),numel(allTicks));
for j=1:numel(allTicks)
    k=find(ticks==allTicks(j),1);
    if ~isempty(k)
        u=forward.u(:,idx,k);
    else
        k=find(ticks-1==allTicks(j),1);
        u=forward.previousState{k}.u(:,idx);
    end
    samples(:,:,j)=u+cfg.sensing.curvatureNoiseStd*randn(stream,size(u));
    % Current FBG model measures two bending components, not torsion.
    samples(3,:,j)=tube.uhat(3,idx);
end
[~,currentIdx]=ismember(ticks,allTicks);
[~,previousIdx]=ismember(ticks-1,allTicks);
packet=struct;
packet.schemaVersion=1;
packet.fbgIdx=idx;
packet.sFbgMm=tube.s(idx);
packet.curvaturePerMm=samples(:,:,currentIdx);
packet.previousCurvaturePerMm=samples(:,:,previousIdx);
packet.basePose=forward.baseTraj;
packet.previousBasePose=zeros(4,4,nt);
for k=1:nt, packet.previousBasePose(:,:,k)=forward.previousState{k}.T_base; end
packet.timeSeconds=(ticks-1)*dt;
packet.previousTimeSeconds=(ticks-2)*dt;
packet.processReferencePeriodSeconds=dt;
packet.frictionMu=forward.frictionMu;
packet.actuationMm=forward.betaMm;
packet.planePointMm=zeros(3,nt);
packet.planeNormal=zeros(3,nt);
n=cfg.planeNormal(:)/norm(cfg.planeNormal);
angleStd=0;
if isfield(cfg.sensing,'planeNormalNoiseStdDeg')
    angleStd=deg2rad(cfg.sensing.planeNormalNoiseStdDeg);
end
for k=1:nt
    offset=cfg.sensing.planeOffsetBiasMm+cfg.sensing.planeOffsetNoiseStdMm*randn(stream);
    packet.planePointMm(:,k)=cfg.planePointMm(:)+n*offset;
    perturb=(eye(3)-n*n')*randn(stream,3,1)*angleStd;
    measuredNormal=n+perturb;
    packet.planeNormal(:,k)=measuredNormal/norm(measuredNormal);
end
packet.provenance='Synthetic sparse bending observations; paired prior sensor sample, no dense truth or true loads. Third curvature is calibrated torsion imposed as a planar-model assumption, not a measured channel.';
end
