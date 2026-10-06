function frames=read_input(input)
% This adapter deliberately accepts sensorInput only, never truth or scene.
assert(~any(isfield(input,{'truth','results','forward'})),'tsfs:TruthInput','Truth is not an observation.');
if isfield(input,'frames')
 frames=cell(size(input.frames));
 for k=1:numel(frames)
  a=input.frames{k}; m=a.model; s=m.sMm(:)'; u0=zeros(3,numel(s));
  v=m.intrinsicCurvaturePerMm(:)';u0(2,:)=[v v(end)];
  tube=CreatTube(s(end),s,u0);tube.kb=m.EINmm2;tube.kt=m.EINmm2/1.3;
  base=eye(4);q=m.baseAngleRad;base(1:3,1:3)=[cos(q) 0 sin(q);0 1 0;-sin(q) 0 cos(q)];base([1 3],4)=m.baseXZ(:);tube.T_base=base;
  p=zeros(3,size(m.planePointXZ,2));p([1 3],:)=m.planePointXZ;n=p;n([1 3],:)=m.planeNormalXZ;
  fr=struct('tube',tube,'arcs',a.fbgArcLengthMm(:)','axes',2,'u',a.curvaturePerMm(:)', ...
   'base',base,'point',p,'normal',n,'mu',zeros(1,size(p,2)), ...
   'CU',eye(numel(a.fbgArcLengthMm))*a.curvatureStdPerMm^2, ...
   'CE',repmat(diag([0.05 0.05 0.05 0.001 0.001 0.001].^2),1,1,size(p,2)), ...
   'time',(k-1)*0.02,'previous',[],'historySource','missing', ...
   'notes',{{'Planar fixture measures one bending channel; unobserved channels use calibrated intrinsic curvature.','Environment prior declared by adapter: 0.05 mm point and 0.001 normal component standard deviations.'}});
  if k>1,fr.previous=frames{k-1};fr.previous.previous=[];fr.historySource='preceding-fbg-reconstruction';end
  frames{k}=fr;
 end
 return
end
p=input.packet;
if p.schemaVersion==1
 T=numel(p.timeSeconds);frames=cell(1,T);axes=input.config.forceSensor.curvatureObservedAxes;
 for k=1:T
  sigma=input.config.forceSensor.measurementStd.curvature;
  if isfield(p,'curvatureNoiseStdPerMm'),sigma=max(p.curvatureNoiseStdPerMm,1e-7);end
  fr=struct('tube',input.tube,'arcs',p.sFbgMm(:)','axes',axes(:)', ...
   'u',p.curvaturePerMm(axes,:,k),'base',p.basePose(:,:,k),'point',p.planePointMm(:,k), ...
   'normal',p.planeNormal(:,k),'mu',p.frictionMu(k),'CU',eye(numel(axes)*numel(p.sFbgMm))*sigma^2, ...
   'CE',p.planeCovariance(:,:,k),'time',p.timeSeconds(k),'previous',[],'historySource','paired-fbg-reconstruction','notes',{{}});
  prev=fr;prev.u=p.previousCurvaturePerMm(axes,:,k);prev.base=p.previousBasePose(:,:,k);prev.time=p.previousTimeSeconds(k);prev.historySource='missing';
  fr.previous=prev;frames{k}=fr;
 end
else
 obs=formulation_window_observations(input);frames=cell(1,obs.frameCount);
 for k=1:obs.frameCount
  W=obs.curvatureWhitening(:,:,k); A=W\eye(size(W));CE=zeros(6,6,obs.planeCount);
  for j=1:obs.planeCount,E=obs.environmentWhitening(:,:,j,k)\eye(6);CE(:,:,j)=E*E';end
  fr=struct('tube',input.tube,'arcs',obs.arcs,'axes',obs.axes,'u',obs.u(:,:,k), ...
   'base',obs.basePose(:,:,k),'point',obs.point(:,:,k),'normal',obs.normal(:,:,k), ...
   'mu',obs.mu(:,k)','CU',A*A','CE',CE,'time',obs.timeSeconds(k), ...
   'previous',[],'historySource','missing','notes',{{}});
  if obs.predecessorIndex(k)>0
   prev=frames{obs.predecessorIndex(k)};prev.previous=[];fr.previous=prev;fr.historySource='preceding-fbg-reconstruction';
  end
  frames{k}=fr;
 end
end
end
