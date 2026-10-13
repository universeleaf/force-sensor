function build_inclined_rotation_truth(maxFrames,options)
% Independent 3D reference: reuse the collaborator map read-only.
if nargin<1,maxFrames=Inf;end;if nargin<2,options=struct;end
step=3.75;if isfield(options,'stepDeg'),step=options.stepDeg;end
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs','inclined_wall_rotation_20261010');if isfield(options,'folder'),folder=options.folder;end;a=load(fullfile(folder,'maximum_insertion_seed.mat'));seed=a.seed;b=load(fullfile(folder,'seed_map_validation.mat'));tube=seed.input.tube;tube.T_base=seed.basePose;
angles=[0:step:45 45-step:-step:0];turnFrame=numel(0:step:45);T=numel(angles);L=tube.s(end);ns=numel(tube.s);n=seed.planeNormal;point=seed.planePoint;B=[[1;0;1]/sqrt(2) [0;1;0]];mu=seed.mu;tip=seed.tipForce;base=seed.basePose;
mapOptions=struct('relativeTolerance',2e-9,'momentToleranceNmm',1e-5,'collisionStepMm',0.1,'maxRhsEvaluations',1000000);
rootOptions=optimoptions('lsqnonlin','Display','off','MaxIterations',80,'MaxFunctionEvaluations',600,'FunctionTolerance',1e-13,'OptimalityTolerance',1e-10,'StepTolerance',1e-11,'FiniteDifferenceStepSize',1e-5,'FiniteDifferenceType','central');
p=zeros(3,ns,T);u=p;R=zeros(3,3,ns,T);w=zeros(2,ns,T);poses=repmat(base,1,1,T);fc=zeros(3,T);arcs=zeros(1,T);shapes=cell(1,T);rows=[];v=[n'*seed.contactForce;B'*seed.contactForce;seed.contactS/L];previous=b.q;mapCalls=0;cacheZ=[];cacheQ=[];
for k=1:min(T,maxFrames)
 clock=tic;ang=deg2rad(angles(k));tube.T_base=base;tube.T_base(1:3,1:3)=base(1:3,1:3)*[cos(ang) -sin(ang) 0;sin(ang) cos(ang) 0;0 0 1];poses(:,:,k)=tube.T_base;mapCalls=0;cacheZ=[];cacheQ=[];iterations=0;
 if k==1
  q=b.q;f=seed.contactForce;slip=zeros(2,1);proj=zeros(2,1);flag=1;mode='saved-insertion-state';
 else
  fprintf('TRUTH START frame %d/%d angle %.2f\n',k,T,angles(k));
  [stick,~,~,flag,out]=lsqnonlin(@stickResidual,v,[0;-100;-100;1/L],[100;100;100;1-1/L],rootOptions);iterations=out.iterations;
  [rr,qq,ff,vv,pp]=residual(stick);
  if flag>0&&norm(rr,inf)<1e-6&&norm(vv)<1e-6&&norm(stick(2:3))<=mu*stick(1)+1e-8
   v=stick;q=qq;f=ff;slip=vv;proj=pp;mode='sticking';
  else
   fprintf('STICK angle %.2f fn %.4f tau %.4f cone %.4g gap %.3g slip %.3g\n',angles(k),stick(1),norm(stick(2:3)),norm(stick(2:3))-mu*stick(1),rr(1),norm(vv));
   seedSlide=[stick(1);atan2(stick(3),stick(2));stick(4)];valid=false;
   for normalFactor=[1 .8 1.2 .5]
    for shift=[0 .2 -.2 .6 -.6 1 -1 pi -2 2]
     st=seedSlide;st(1)=normalFactor*st(1);st(2)=st(2)+shift;
     [z,~,~,flag,out]=lsqnonlin(@slideResidual,st,[0;-Inf;1/L],[100;Inf;1-1/L],rootOptions);iterations=iterations+out.iterations;
     trial=[z(1);mu*z(1)*[cos(z(2));sin(z(2))];z(3)];[rr,qq,ff,vv,pp]=residual(trial);
     fprintf('SLIDE angle %.2f seed %.2f gap %.2g tangent %.2g proj %.2g work %.4g\n',angles(k),shift,rr(1),rr(2)/10,norm(pp),trial(2:3)'*vv);
     if flag>0&&norm(rr,inf)<1e-6&&trial(2:3)'*vv<=1e-8,v=trial;q=qq;f=ff;slip=vv;proj=pp;valid=true;break,end
    end
    if valid,break,end
   end
   if ~valid,save(fullfile(folder,'forward_failed.mat'),'k','tube','previous','v','stick','rows');error('inclinedRotation:InvalidSlidingRoot','No dissipative sliding root at frame %d',k);end
   mode='sliding';
  end
 end
 gap=n'*(q.contactPoints-point);tangency=n'*q.contactTangents;penetration=max(0,-min(n'*(q.collisionP-point)));cone=max(0,norm(v(2:3))-mu*v(1));work=v(2:3)'*slip;
 passed=flag>0&&abs(gap)<2e-6&&abs(tangency)<1e-6&&penetration<2e-5&&norm(proj,inf)<1e-6&&cone<1e-6&&work<=1e-8&&v(1)>0;
 if ~passed,save(fullfile(folder,'forward_failed.mat'),'k','tube','v','q','rows','gap','tangency','penetration','proj');error('inclinedRotation:ReferenceAudit','Reference physics failed at frame %d',k);end
 % Dense independent mechanical output resolves the material-to-Bishop integral.
 if k==1
  ww=seed.bishopW;refineError=0;
 else
  fine=tube;fine.s=unique([tube.s 0:.1:L v(4)*L]);fine.uhat=interp1(tube.s,tube.uhat',fine.s,'previous')';opts=mapOptions;opts.relativeTolerance=2e-10;
  refined=solve_cosserat_multi_contact_map(fine,v(4)*L,f,tip,opts);mapCalls=mapCalls+1;[denseW,~]=tsfs.material_to_bishop(fine.s,refined.u);[found,ix]=ismember(tube.s,fine.s);assert(all(found));ww=denseW(:,ix);refineError=max(vecnorm(refined.p(:,ix)-q.p));assert(refineError<1e-3,'Sensor refinement does not match reference');
 end
 p(:,:,k)=q.p;u(:,:,k)=q.u;R(:,:,:,k)=q.R;w(:,:,k)=ww;fc(:,k)=f;arcs(k)=v(4)*L;shapes{k}=q;previous=q;
 if k==1,p(:,:,k)=seed.p;u(:,:,k)=seed.u;end
 direction='rotate-forward';if k>turnFrame,direction='rotate-back';end
 rows=[rows;struct('frame',k,'angleDeg',angles(k),'distanceMm',40,'direction',direction,'mode',mode,'seconds',toc(clock),'iterations',iterations,'mapCalls',mapCalls,'gapMm',gap,'tangency',tangency,'penetrationMm',penetration,'projectionN',norm(proj,inf),'coneExcessN',cone,'workNmm',work,'slipMm',norm(slip),'terminalMomentNmm',q.tipMomentResidualNmm,'contactArcMm',arcs(k),'normalN',v(1),'tangentN',v(2),'tangent2N',v(3),'contactFx',f(1),'contactFy',f(2),'contactFz',f(3),'twistIntegralDeg',rad2deg(trapz(tube.s,q.u(3,:))),'sensorRefineShapeMaxMm',refineError,'passed',passed)];
 writetable(struct2table(rows),fullfile(folder,'truth_frames.csv'));save(fullfile(folder,'forward_checkpoint.mat'),'seed','angles','p','u','R','w','poses','fc','arcs','shapes','rows','k','v','mapOptions','rootOptions','-v7.3');
 fprintf('TRUTH DONE %d/%d angle %.2f %s: %.2fs fn %.4f tau %.4f arc %.3f audit %d\n',k,T,angles(k),mode,rows(end).seconds,v(1),norm(v(2:3)),arcs(k),passed);
end
if maxFrames<T,return,end
input=seed.input;packet=input.packet;idx=packet.fbgIdx;packet.bishopCurvaturePerMm=w(:,idx,:);packet.previousBishopCurvaturePerMm=packet.bishopCurvaturePerMm(:,:,[1 1:T-1]);packet.basePose=poses;packet.previousBasePose=poses(:,:,[1 1:T-1]);if isfield(seed,'timeSeconds'),t0=seed.timeSeconds;else,t0=seed.estimates{1}.time;end;packet.timeSeconds=t0+(0:T-1)*.02;packet.previousTimeSeconds=packet.timeSeconds-.02;packet.frictionMu=mu*ones(1,T);packet.actuationMm=base(3,4)*ones(1,T);packet.planePointMm=repmat(point,1,T);packet.planeNormal=repmat(n,1,T);packet.planeCovariance=repmat(seed.input.packet.planeCovariance(:,:,seed.frame),1,1,T);packet.provenance='Saved maximum-insertion loaded state, then independent 3D axial base rotation 0 to 45 to 0 degrees; noiseless dense Bishop observations; centerline Coulomb force, no surface torque.';input.packet=packet;input.tube.T_base=base;
truth=struct('p',p,'u',u,'R',R,'bishopW',w,'basePose',poses,'contactForce',fc,'tipForce',repmat(tip,1,T),'contactS',arcs,'planePoint',point,'planeNormal',n,'sMm',tube.s,'angleDeg',angles,'distanceMm',40*ones(1,T),'mu',mu,'shapes',{shapes},'sourceSeedFrame',seed.frame);
configuration=struct('angleDeg',angles,'rotationAxis','local base z','stepDeg',step,'distanceMm',40,'tipForceWorldN',tip,'trueMu',mu,'noise',0,'initialState',sprintf('saved insertion frame %d with tangential preload and each estimator own history',seed.frame),'scope','same centerline point-force friction; no surface friction torque, tip or edge contact');
save(fullfile(folder,'truth.mat'),'input','truth','rows','configuration','mapOptions','rootOptions','-v7.3');atomic_write_artifact(fullfile(folder,'configuration.json'),'json',configuration);
 function r=stickResidual(z),[r,~,~,sl]=residual(z);r(3:4)=sl;end
 function r=slideResidual(z),vv=[z(1);mu*z(1)*[cos(z(2));sin(z(2))];z(3)];[r,~,~,sl]=residual(vv);r=[r(1:2);[-sin(z(2)) cos(z(2))]*sl];end
 function [r,q,f,sl,proj]=residual(z)
  f=n*z(1)+B*z(2:3);
  if isequal(z,cacheZ),q=cacheQ;else,q=solve_cosserat_multi_contact_map(tube,z(4)*L,f,tip,mapOptions);mapCalls=mapCalls+1;cacheZ=z;cacheQ=q;end
  old=interp1(previous.collisionS,previous.collisionP',z(4)*L,'pchip')';sl=B'*(q.contactPoints-old);arg=z(2:3)-sl;disk=arg*min(1,mu*z(1)/max(norm(arg),realmin));proj=z(2:3)-disk;r=[n'*(q.contactPoints-point);10*n'*q.contactTangents;proj];
 end
end
