function build_varying_tip_truth(options)
% Copy of build_inclined_truth with a time-varying applied tip load (world X),
% used only to measure tracking lag of temporal force priors. Same read-only
% collaborator forward solver and friction-branch selection; only tipForceXZ changes.
% Default schedule (frame index k of the 0.4 mm trajectory): 1 N until k=60,
% linear ramp to 2 N at k=80, hold, step to 0.5 N at k=131, hold.
if nargin<1,options=struct;end
step=2;if isfield(options,'stepMm'),step=options.stepMm;end
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'inclined_wall');if isfield(options,'folder'),folder=options.folder;end;if ~isfolder(folder),mkdir(folder);end
seed=load(fullfile(root,'datasets','sliding_clean.mat'));input=seed.sensorInput;tube=input.tube;s=0:200;radius=120/(pi/2);tube.s=s;tube.uhat=zeros(3,numel(s));tube.uhat(2,s>=80)=1/radius;tube.T_base=eye(4);
n=[1;0;-1]/sqrt(2);tangent=[1;0;1]/sqrt(2);point=[0;0;sqrt(2)*80];tip=[1;0;0];mu=0.3;distances=[80:-step:40 40+step:step:80];turnFrame=numel(80:-step:40);T=numel(distances);p=zeros(3,numel(s),T);u=p;poses=repmat(eye(4),1,1,T);fc=zeros(3,T);contactS=NaN(1,T);rows=[];shapes=cell(1,T);previous=[];
model=struct('sMm',s,'intrinsicCurvaturePerMm',double((s(1:end-1)+s(2:end))/2>=80)/radius,'EINmm2',tube.kb,'baseXZ',[0;0],'baseAngleRad',0,'tipForceXZ',[1;0],'planeNormalXZ',n([1 3]),'planePointXZ',point([1 3]));
tipX=ones(1,T);for k=1:T,if k>60&&k<=80,tipX(k)=1+(k-60)/20;elseif k>80&&k<=130,tipX(k)=2;elseif k>130,tipX(k)=0.5;end,end
if isfield(options,'tipX'),tipX=options.tipX;end
for k=1:T
 clock=tic;model.tipForceXZ=[tipX(k);0];model.baseXZ=[0;sqrt(2)*(80-distances(k))];model.tangentialNormalRatio=0;zero=solve_planar_contact_shooting(model);calls=1;
 if zero.normalReactionN<1e-8,q=zero;ratio=0;mode='free';slip=0;
 else
  [minus,vm]=atRatio(-mu);[plus,vp]=atRatio(mu);
  if vm>=-1e-9,q=minus;ratio=-mu;slip=vm;mode='sliding';
  elseif vp<=1e-9,q=plus;ratio=mu;slip=vp;mode='sliding';
  else
   assert(vm<0&&vp>0,'No bracket for sticking branch');ratio=fzero(@slipAt,[-mu mu],optimset('TolX',1e-10,'Display','off'));[q,slip]=atRatio(ratio);mode='sticking';
  end
 end
 world=[q.pXZ(1,:);zeros(1,numel(s));q.pXZ(2,:)];force=[q.contactResultantXZ(1);0;q.contactResultantXZ(2)];lambda=q.normalReactionN;tau=tangent'*force;projection=tau-max(-mu*lambda,min(mu*lambda,tau-slip));work=tau*slip;
 if lambda>1e-8,tan=abs(n([1 3])'*[sin(interp1(s,q.thetaRad,q.contactArcLengthMm,'pchip'));cos(interp1(s,q.thetaRad,q.contactArcLengthMm,'pchip'))]);else,tan=0;end
 assert(abs(projection)<2e-6&&work<1e-8&&q.maxPenetrationMm<1e-6&&abs(tau)<=mu*lambda+1e-7,'Reference physics failed');
 p(:,:,k)=world;u(2,:,k)=q.curvaturePerMm;poses(3,4,k)=model.baseXZ(2);fc(:,k)=force;contactS(k)=q.contactArcLengthMm;shapes{k}=q;previous=q;
 direction='insert';if k>turnFrame,direction='withdraw';end
 rows=[rows;struct('frame',k,'distanceMm',distances(k),'baseZmm',model.baseXZ(2),'direction',direction,'mode',mode,'contactArcMm',q.contactArcLengthMm,'normalN',lambda,'tangentN',tau,'contactFx',force(1),'contactFy',force(2),'contactFz',force(3),'slipMm',slip,'workNmm',work,'projectionN',abs(projection),'penetrationMm',q.maxPenetrationMm,'complementarityNmm',q.complementarityNmm,'stationarityNmm',q.stationarityInfNmm,'tangencyInterpolated',tan,'mapCalls',calls,'seconds',toc(clock))];
 writetable(struct2table(rows),fullfile(folder,'truth_frames.csv'));fprintf('TRUTH %d/%d d %.2f %s s %.3f fn %.3f tau %.3f slip %.5f\n',k,T,distances(k),mode,contactS(k),lambda,tau,slip);
end
packet=input.packet;packet=rmfield(packet,{'curvaturePerMm','previousCurvaturePerMm'});idx=packet.fbgIdx;w=zeros(2,numel(s),T);w(1,:,:)=u(2,:,:);packet.observationType='bishop-curvature';packet.bishopCurvaturePerMm=w(:,idx,:);packet.previousBishopCurvaturePerMm=packet.bishopCurvaturePerMm(:,:,[1 1:T-1]);packet.basePose=poses;packet.previousBasePose=poses(:,:,[1 1:T-1]);packet.timeSeconds=(0:T-1)*0.02;packet.previousTimeSeconds=packet.timeSeconds-0.02;packet.frictionMu=mu*ones(1,T);packet.actuationMm=reshape(poses(3,4,:),1,[]);packet.planePointMm=repmat(point,1,T);packet.planeNormal=repmat(n,1,T);packet.planeCovariance=repmat(diag([0.03^2*ones(1,3) 0.001^2*ones(1,3)]),1,1,T);packet.provenance='Independent planar quasistatic reference; friction branch selected against actual previous material points; ideal Bishop bending channels; no injected noise.';
input.packet=packet;input.tube=tube;input.config.forceSensor.measurementStd.curvature=1e-4;
truth=struct('p',p,'u',u,'bishopW',w,'basePose',poses,'contactForce',fc,'tipForce',[tipX;zeros(2,T)],'contactS',contactS,'planePoint',point,'planeNormal',n,'sMm',s,'distanceMm',distances,'mu',mu,'shapes',{shapes});
configuration=struct('lengthMm',200,'straightMm',80,'curvedMm',120,'intrinsicAngleDeg',90,'intrinsicRadiusMm',radius,'wallNormal',n,'wallPointMm',point,'distanceConvention','Perpendicular base to plane','distanceSequenceMm',distances,'tipForceWorldN','time-varying: 1 N, ramp k61-80 to 2 N, step to 0.5 N at k131','tipForceXSchedule',tipX,'trueMu',mu,'solver','fmincon SQP','sensorStdPerMm',1e-4,'injectedNoiseStd',0,'forcePriors',false,'constraintTolerance',1e-6,'epsilon',1e-6,'stepDistanceMm',step,'samplingSeconds',0.02,'scope','Quasistatic; 0.02 s timestamps are history bookkeeping, not a validated dynamic insertion speed. Full exposed rod translates along world +Z and then reverses.');
save(fullfile(folder,'truth.mat'),'input','truth','configuration','rows','-v7.3');atomic_write_artifact(fullfile(folder,'configuration.json'),'json',configuration);
 function [q,v]=atRatio(r)
  mm=model;mm.tangentialNormalRatio=r;q=solve_planar_contact_shooting(mm);calls=calls+1;if isempty(previous),v=0;else,old=interp1(s,previous.pXZ',q.contactArcLengthMm,'pchip')';v=tangent([1 3])'*(q.contactPointXZ-old);end
 end
 function v=slipAt(r),[~,v]=atRatio(r);end
end
