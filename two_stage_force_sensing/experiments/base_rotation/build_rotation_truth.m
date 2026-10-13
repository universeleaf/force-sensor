function [input,truth]=build_rotation_truth(mu,angles)
% Independent contact wrapper around the collaborator's unmodified 3D map.
if nargin<1,mu=0.3;end
if nargin<2,angles=linspace(0,45,13);end
assert(isscalar(mu)&&isfinite(mu)&&mu>=0&&abs(1000*mu-round(1000*mu))<1e-9,'Coefficient names use 0.001 resolution.');
assert(angles(1)==0&&all(diff(angles)>0));
[root,paths]=setup_tsfs;assert(startsWith(getenv('TSFS_RUN_ID'),'base_rotation_'),'Use a separate base_rotation_ run ID.');
folder=fullfile(paths.results,'rotation');if ~isfolder(folder),mkdir(folder);end
name=sprintf('truth_mu_%03d',round(1000*mu));file=fullfile(folder,[name '.mat']);
if isfile(file),a=load(file);assert(isequal(a.truth.angleDeg,angles)&&a.truth.mu==mu&&a.truth.generatorVersion==2,'Cached reference settings differ; use a fresh run ID.');input=a.input;truth=a.truth;return,end
seed=load(fullfile(root,'datasets','sliding_clean.mat'));input=seed.sensorInput;tube=input.tube;
L=tube.s(end);base=eye(4);base(3,4)=18;tube.T_base=base;
point=[0;0;160];normal=[0;0;-1];B=[1 0;0 1;0 0];tip=[1;0;-1];
mid=(tube.s(1:end-1)+tube.s(2:end))/2;
model=struct('sMm',tube.s,'intrinsicCurvaturePerMm',double(mid>=120)/30,'EINmm2',tube.kb, ...
 'baseXZ',[0;18],'baseAngleRad',0,'tipForceXZ',[1;-1],'planePointXZ',[0;160],'planeNormalXZ',[0;-1],'tangentialNormalRatio',0);
initial=solve_planar_contact_shooting(model);
mapOptions=struct('relativeTolerance',2e-9,'momentToleranceNmm',1e-5,'collisionStepMm',0.1,'maxRhsEvaluations',1000000);
rootOptions=optimoptions('lsqnonlin','Display','off','MaxIterations',80,'MaxFunctionEvaluations',600, ...
 'FunctionTolerance',1e-13,'OptimalityTolerance',1e-10,'StepTolerance',1e-11,'FiniteDifferenceStepSize',1e-5,'FiniteDifferenceType','central');
T=numel(angles);ns=numel(tube.s);p=zeros(3,ns,T);u=p;R=zeros(3,3,ns,T);bases=repmat(base,1,1,T);fc=zeros(3,T);arcs=zeros(1,T);rows=[];shapes=cell(1,T);
v=[initial.normalReactionN;0;0;initial.contactArcLengthMm/L];previous=[];mapCalls=0;
for k=1:T
 clock=tic;a=deg2rad(angles(k));tube.T_base=base;tube.T_base(1:3,1:3)=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];bases(:,:,k)=tube.T_base;mapCalls=0;
 if k==1
  flag=1;iterations=0;
 else
  if mu==0
   [v,~,~,flag,out]=lsqnonlin(@zeroResidual,v,[0;-100;-100;1/L],[100;100;100;1-1/L],rootOptions);iterations=out.iterations;
  else
   [stick,~,~,flag,out]=lsqnonlin(@stickResidual,v,[0;-100;-100;1/L],[100;100;100;1-1/L],rootOptions);iterations=out.iterations;
   if norm(stick(2:3))<=mu*stick(1)+1e-8
    v=stick;
   else
    seedSlide=[stick(1);atan2(stick(3),stick(2));stick(4)];
    fprintf('STICK candidate fn %.6f tau %.6f arc %.6f cone excess %.6f\n',stick(1),norm(stick(2:3)),stick(4)*L,norm(stick(2:3))-mu*stick(1));validSlide=false;
    for normalFactor=[1 0.8 1.2 0.5]
    for shift=[0 0.2 -0.2 0.6 -0.6 1 -1 pi -2 2]
     trialSeed=seedSlide;trialSeed(1)=normalFactor*seedSlide(1);trialSeed(2)=trialSeed(2)+shift;
     [w,~,~,flag,out]=lsqnonlin(@slideResidual,trialSeed,[0;-Inf;1/L],[100;Inf;1-1/L],rootOptions);
     trial=[w(1);mu*w(1)*[cos(w(2));sin(w(2))];w(3)];iterations=iterations+out.iterations;
     [check,~,~,trialSlip]=residual(trial);fprintf('SLIDE seed %.3f fn %.6f angle %.6f gap %.3g tan %.3g proj %.3g work %.5g flag %d\n',shift,w(1),w(2),check(1),check(2),norm(check(3:4)),trial(2:3)'*trialSlip,flag);
     if flag>0&&norm(check,inf)<1e-6&&trial(2:3)'*trialSlip<=1e-8,v=trial;validSlide=true;break,end
    end
    if validSlide,break,end
    end
    if ~validSlide,save(fullfile(folder,[name '_branch_failed.mat']),'tube','previous','stick','v','angles','k','mapOptions','shapes','rows');end
    assert(validSlide,'rotation:SlidingRoot','No dissipative sliding root passed the projection check.');
   end
  end
 end
 [r,q,f,slip,proj]=residual(v);gap=normal'*(q.contactPoints-point);tangent=normal'*q.contactTangents;
 penetration=max(0,-min(normal'*(q.collisionP-point)));cone=max(0,norm(v(2:3))-mu*v(1));work=v(2:3)'*slip;
 audit=flag>0&&abs(gap)<2e-6&&abs(tangent)<1e-6&&penetration<2e-5&&max(abs(proj))<1e-6&&cone<1e-6&&work<=1e-8&&v(1)>0;
 row=struct('frame',k,'angleDeg',angles(k),'mu',mu,'seconds',toc(clock),'outerIterations',iterations,'mapCalls',mapCalls,'exitFlag',flag, ...
  'gapMm',gap,'tangency',tangent,'penetrationMm',penetration,'projectionN',norm(proj,inf),'coneExcessN',cone,'frictionWorkNmm',work, ...
  'slipMm',norm(slip),'terminalMomentNmm',q.tipMomentResidualNmm,'contactArcMm',v(4)*L,'normalN',v(1), ...
  'contactFx',f(1),'contactFy',f(2),'contactFz',f(3),'frictionN',norm(v(2:3)), ...
  'twistIntegralDeg',rad2deg(trapz(tube.s,q.u(3,:))),'maxTwistPerMm',max(abs(q.u(3,:))),'passed',audit);
 rows=[rows;row];writetable(struct2table(rows),fullfile(folder,[name '_audit.csv']));
 fprintf('TRUTH mu %.3f angle %.2f: %.2fs fn %.4f tau %.4f gap %.2g slip %.4g twist %.3f audit %d\n',mu,angles(k),row.seconds,v(1),norm(v(2:3)),gap,norm(slip),row.twistIntegralDeg,audit);
 if ~audit,save(fullfile(folder,[name '_failed.mat']),'v','q','r','row','angles');error('rotation:InvalidTruth','Forward solution failed physical audit at %.2f degrees.',angles(k));end
 p(:,:,k)=q.p;u(:,:,k)=q.u;R(:,:,:,k)=q.R;fc(:,k)=f;arcs(k)=v(4)*L;shapes{k}=q;previous=q;
end
packet=seed.sensorInput.packet;idx=packet.fbgIdx;
packet.curvaturePerMm=u(:,idx,:);packet.curvaturePerMm(3,:,:)=repmat(reshape(tube.uhat(3,idx),1,[],1),1,1,T);
packet.previousCurvaturePerMm=packet.curvaturePerMm(:,:,[1 1:T-1]);packet.basePose=bases;packet.previousBasePose=bases(:,:,[1 1:T-1]);
packet.timeSeconds=(0:T-1)*0.02;packet.previousTimeSeconds=packet.timeSeconds-0.02;packet.frictionMu=mu*ones(1,T);packet.actuationMm=18*ones(1,T);
packet.planePointMm=repmat(point,1,T);packet.planeNormal=repmat(normal,1,T);packet.planeCovariance=repmat(seed.sensorInput.packet.planeCovariance(:,:,1),1,1,T);
packet.provenance='Independent spatial contact solve; rest initialization with zero tangential preload; axial base rotation; two bending channels only, third entry remains intrinsic twist, no injected noise.';
input.packet=packet;input.tube.T_base=base;
truth=struct('p',p,'u',u,'R',R,'basePose',bases,'contactForce',fc,'tipForce',repmat(tip,1,T),'totalForce',fc+tip, ...
 'contactS',arcs,'planePoint',point,'planeNormal',normal,'sMm',tube.s,'angleDeg',angles,'mu',mu,'shapes',{shapes}, ...
 'generatorVersion',2,'initialization','Rest equilibrium, zero tangential preload; frame zero history is identical at the earlier timestamp.', ...
 'scope','Quasistatic centerline point-force Coulomb contact; no surface torque, inertia or tip contact. Independent upstream nested shooting map; forward friction uses actual preceding material positions.');
save(file,'input','truth','rows','mapOptions','rootOptions','-v7.3');
 function r=stickResidual(z)
  [r,~,~,slip]=residual(z);r(3:4)=slip;
 end
 function r=zeroResidual(z)
  r=residual(z);r(3:4)=z(2:3);
 end
 function r=slideResidual(w)
  z=[w(1);mu*w(1)*[cos(w(2));sin(w(2))];w(3)];[r,~,~,slip]=residual(z);
  r=[r(1:2);[-sin(w(2)) cos(w(2))]*slip];
 end
 function [res,q,f,slip,proj]=residual(z)
  f=normal*z(1)+B*z(2:3);q=solve_cosserat_multi_contact_map(tube,z(4)*L,f,tip,mapOptions);mapCalls=mapCalls+1;
  if isempty(previous),slip=zeros(2,1);else,old=interp1(previous.collisionS,previous.collisionP',z(4)*L,'pchip')';slip=B'*(q.contactPoints-old);end
  % Independent disk projection; do not call the inverse's contact routine.
  diskArgument=z(2:3)-slip;radius=mu*z(1);projected=diskArgument*min(1,radius/max(norm(diskArgument),realmin));proj=z(2:3)-projected;
  if isempty(previous),proj=z(2:3);end
  res=[normal'*(q.contactPoints-point);10*(normal'*q.contactTangents);proj];
 end
end
