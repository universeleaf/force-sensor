function audit_direction_sensitivity
% Offline local sensitivity only: no estimator settings or recorded states changed.
[root,paths]=setup_tsfs;o=tsfs.defaults;pub=fullfile(root,'results','published','base_rotation_direction_audit_20261007');if ~isfolder(pub),mkdir(pub);end
rows=[];axesRows=[];detail={};checks=[];actual=[];
for id={'planar','375deg','1deg'}
 tag=id{1};
 if strcmp(tag,'planar'),a=load(fullfile(root,'datasets','sliding_clean.mat'));input=a.sensorInput;t=a.truth;angles=NaN(1,size(t.p,3));
 else,a=load(fullfile(root,'results','runs',['base_rotation_relaxed_v2_' tag '_20261007'],'rotation','truth_mu_300.mat'));input=a.input;t=a.truth;angles=t.angleDeg;end
 frames=tsfs.read_input(input);
 for k=1:numel(frames)
  fr=frames{k};fr.observationType='bishop-curvature';fr.axes=[1 2];
  n=t.planeNormal;if size(n,2)>1,n=n(:,k);end;n=n/norm(n);B=null(n');A=[n B];
  g=struct('s',t.contactS(k),'normal',n,'plane',1,'B',B);
  fc=t.contactForce(:,k);ft=t.tipForce(:,k);pc=interp1(t.sMm,t.p(:,:,k)',g.s,'pchip')';
  if isfield(t,'contactPoint'),pc=t.contactPoint(:,k);elseif isfield(t,'shapes'),pc=t.shapes{k}.contactPoints;end
  m=cross(t.p(:,end,k)-t.p(:,1,k),ft)+cross(pc-t.p(:,1,k),fc);scale=[200;200;200;ones(6,1)];z=[m;ft;A'*fc]./scale;
  sh=tsfs.ivp(fr,g,z,scale,o,true);[G,P,Q,E]=reduced(sh,scale,A);
  if k>1
   if isfield(t,'shapes'),prev=t.shapes{k-1};old=interp1(prev.collisionS,prev.collisionP',g.s,'pchip')';
   else,old=interp1(t.sMm,t.p(:,:,k-1)',g.s,'pchip')';end
  else
   if strcmp(tag,'planar'),prev=tsfs.reconstruct(frames{k}.previous,false);old=interp1(prev.s,prev.p',g.s,'pchip')';else,old=pc;end
  end
  v=B'*(pc-old);tau=B'*fc;lambda=n'*fc;mu=fr.mu(1);margin=mu*lambda-norm(tau);
  if norm(v)>1e-4&&abs(margin)<1e-4
   mode='sliding';unit=v/norm(v);Cf=[n'*P;[zeros(2,3) B'+mu*unit*n']+mu*lambda/norm(v)*(eye(2)-unit*unit')*B'*P];
  else,mode='sticking';Cf=P;end
  [U,S,V]=svd(G,0);sv=diag(S);N=null(Cf);sf=svd(G*N);
  r=struct('caseName',tag,'frame',k,'angleDeg',angles(k),'mode',mode,'contactS',g.s,'normalN',lambda,'slipMm',norm(v),'coneMarginN',margin, ...
   'tipY',ft(2),'contactY',fc(2),'condition',sv(1)/sv(end),'smin',sv(end),'smax',sv(1), ...
   'weakYFraction',norm(V([2 5],end)),'contactTangentDim',size(N,2),'contactTangentCondition',sf(1)/sf(end), ...
   'momentEliminationCondition',cond(E(:,1:3)),'referenceShapeMaxMm',max(vecnorm(sh.p-t.p(:,:,k))),'terminalMomentNmm',norm(sh.tipMoment,inf));
  rows=[rows;r];
  detail{end+1}=struct('caseName',tag,'frame',k,'angleDeg',angles(k),'G',G,'contactPJacobian',P,'contactLawJacobian',Cf,'singularValues',sv,'weakForceDirection',V(:,end),'feasibleSingularValues',sf,'order',{{'tipX','tipY','tipZ','contactX','contactY','contactZ'}});
  for axis=1:3
   u=zeros(6,1);u(axis)=1;ex=u;ex(axis+3)=-1;
   [d0,res0]=profile(G,zeros(0,6),axis);[dg,resg]=profile(G,n'*P,axis);[dc,resc]=profile(G,Cf,axis);
   names='XYZ';axesRows=[axesRows;struct('caseName',tag,'frame',k,'angleDeg',angles(k),'axis',names(axis),'mode',mode, ...
    'tipOnlyCurvatureRmsPerN',rms(G*u),'contactOnlyCurvatureRmsPerN',rms(G(:,axis+3)), ...
    'tipContactCorrelation',dot(G(:,axis),G(:,axis+3))/(norm(G(:,axis))*norm(G(:,axis+3))), ...
    'exchangeCurvatureRmsPerN',rms(G*ex),'profileCurvatureRmsPerN',rms(G*d0),'gapProfileCurvatureRmsPerN',rms(G*dg),'lawProfileCurvatureRmsPerN',rms(G*dc), ...
    'lawProfileShapeRmsMmPerN',sqrt(mean(sum(reshape(Q*dc,3,[]).^2,1))),'profileNoiseStdN',1e-4/norm(G*d0),'lawProfileNoiseStdN',1e-4/norm(G*dc), ...
    'compensatingTipX',dc(1),'compensatingTipY',dc(2),'compensatingTipZ',dc(3),'compensatingContactX',dc(4),'compensatingContactY',dc(5),'compensatingContactZ',dc(6), ...
    'profileEqualityError',max([res0 resg resc]))];
  end
  if (strcmp(tag,'planar')&&k==1)||(strcmp(tag,'375deg')&&ismember(k,[1 7 11 13]))
   for d={[0;1;0;0;0;0],V(:,end)}
    dir=d{1};h=1e-3;pred=zeros(numel(sh.prediction),2);
    for signidx=1:2
     delta=(2*signidx-3)*h;xx=sh.x;xx(4:end)=xx(4:end)+delta*blkdiag(eye(3),A')*dir;
     for it=1:5
      test=tsfs.ivp(fr,g,xx./scale,scale,o,true);
      if norm(test.tipMoment,inf)<1e-9,break,end
      xx(1:3)=xx(1:3)-(test.JtipMoment(:,1:3)./scale(1:3)')\test.tipMoment;
     end
     pred(:,signidx)=test.prediction(:);
    end
    fd=(pred(:,2)-pred(:,1))/(2*h);checks=[checks;struct('caseName',tag,'frame',k,'angleDeg',angles(k),'directionY',dir(2),'relativeError',norm(fd-G*dir)/norm(G*dir))];
   end
  end
 end
 if ~strcmp(tag,'planar')
  for solver={'ipopt','fmincon'}
   d=load(fullfile(root,'results','runs',['base_rotation_relaxed_v2_' tag '_20261007'],'rotation',[solver{1} '.mat']));
   for k=1:numel(d.runs)
    out=d.runs{k};g=out.geometry;m=out.mechanics;A=[g.normal(:,g.plane(1)) g.B(:,:,1)];[G,~,~,~]=reduced(m.evaluation.shape,m.scale,A);sv=svd(G);err=out.tipForce-t.tipForce(:,k);ec=sum(out.contactForce,2)-t.contactForce(:,k);
    actual=[actual;struct('caseName',tag,'solver',solver{1},'frame',k,'angleDeg',angles(k),'reason',m.reason,'forceCondition',sv(1)/sv(end),'tipErrorX',err(1),'tipErrorY',err(2),'tipErrorZ',err(3),'contactErrorX',ec(1),'contactErrorY',ec(2),'contactErrorZ',ec(3),'objective',m.evaluation.objective)];
   end
  end
 end
end
writetable(struct2table(rows),fullfile(pub,'reference_sensitivity.csv'));writetable(struct2table(axesRows),fullfile(pub,'direction_sensitivity.csv'));writetable(struct2table(checks),fullfile(pub,'derivative_checks.csv'));writetable(struct2table(actual),fullfile(pub,'actual_error_components.csv'));atomic_write_artifact(fullfile(pub,'matrices.json'),'json',detail);
assert(max([checks.relativeError])<1e-3,'Derivative validation failed');assert(max([axesRows.profileEqualityError])<1e-7,'Profile equalities failed');
fprintf('Audited %d reference states, %d directional profiles and %d actual outputs. Max FD error %.3g; max reference shape discrepancy %.3g mm.\n',numel(rows),numel(axesRows),numel(actual),max([checks.relativeError]),max([rows.referenceShapeMaxMm]));
end
function [G,P,Q,E]=reduced(sh,scale,A)
J=sh.Jprediction./scale';E=sh.JtipMoment./scale';M=E(:,1:3)\E(:,4:end);T=blkdiag(eye(3),A);G=(J(:,4:end)-J(:,1:3)*M)/T;
J=reshape(sh.JcontactP(:,1,:),3,[])./scale';P=(J(:,4:end)-J(:,1:3)*M)/T;
J=reshape(sh.JcollisionP,[],numel(scale))./scale';Q=(J(:,4:end)-J(:,1:3)*M)/T;
end
function [d,err]=profile(G,C,axis)
u=zeros(1,6);u(axis)=1;D=[C;u];b=[zeros(size(C,1),1);1];d=pinv(D)*b;N=null(D);d=d+N*((G*N)\(-G*d));err=norm(D*d-b,inf);
end
