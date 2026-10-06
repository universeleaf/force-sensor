function audit_plane_sensitivity
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'plane_sensitivity');if ~isfolder(folder),mkdir(folder);end
names={'sliding_clean','s_channel_two_contact'};o=tsfs.defaults;o.odeRelativeTolerance=2e-10;o.odeAbsoluteTolerance=2e-12;o.collisionStepMm=0.1;
allRows=[];models=cell(2,12);records=cell(2,12);finiteRows=[];
for scenario=1:2
 a=load(fullfile(root,'datasets',[names{scenario} '.mat']));frames=tsfs.read_input(a.sensorInput);t=a.truth;
 for k=1:numel(frames)
  fr=frames{k};L=fr.tube.s(end);EI=mean(fr.tube.kb);Fref=EI/L^2;Mref=EI/L;
  C=numel(t.contactS(:,k));ss=t.contactS(:,k)';
  if isfield(t,'contactForces'),fc=t.contactForces(:,:,k);pc=t.contactPoints(:,:,k);else,fc=t.contactForce(:,k);pc=t.contactPoint(:,k);end
  n=fr.normal;fn=sum(n.*fc,1);mu=fr.mu;slide=zeros(3,C);
  for i=1:C,tan=fc(:,i)-fn(i)*n(:,i);if norm(tan)>1e-10,slide(:,i)=-tan/norm(tan);end,end
  m=cross(t.p(:,end,k)-fr.base(1:3,4),t.tipForce(:,k));for i=1:C,m=m+cross(pc(:,i)-fr.base(1:3,4),fc(:,i));end
  model=struct('fr',fr,'normal',n,'point',fr.point,'mu',mu,'slide',slide,'tip',t.tipForce(:,k),'C',C,'L',L,'Fref',Fref,'Mref',Mref,'o',o);
  q=[m/Mref;fn(:)/Fref;ss(:)/L];theta=zeros(2*C+1,1);
  [q,r,sh,y,exitflag]=solve(model,q,theta);assert(norm(r,inf)<1e-8,'Nominal forward root failed.');
  nq=numel(q);np=numel(theta);J=zeros(numel(r),nq);Yq=zeros(numel(y),nq);h=1e-5;
  for j=1:nq
   qp=q;qm=q;qp(j)=qp(j)+h;qm(j)=qm(j)-h;[rp,~,yp]=evaluate(model,qp,theta);[rm,~,ym]=evaluate(model,qm,theta);
   J(:,j)=(rp-rm)/(2*h);Yq(:,j)=(yp-ym)/(2*h);
  end
  Rpar=zeros(numel(r),np);Ypar=zeros(numel(y),np);
  for j=1:np
   h=1e-4;if j>C&&j<=2*C,h=1e-5;end
   tp=theta;tm=theta;tp(j)=tp(j)+h;tm(j)=tm(j)-h;[rp,~,yp]=evaluate(model,q,tp);[rm,~,ym]=evaluate(model,q,tm);
   Rpar(:,j)=(rp-rm)/(2*h);Ypar(:,j)=(yp-ym)/(2*h);
  end
  Qpar=-J\Rpar;D=Yq*Qpar+Ypar;sing=svd(J);
  ds=D(1:C,:);dp=reshape(D(C+1:4*C,:),3,C,np);dfn=D(4*C+1:5*C,:);df=reshape(D(5*C+1:8*C,:),3,C,np);du=D(8*C+1:end,:);
  rec=struct('scenario',names{scenario},'frame',k,'s',ss,'normalForces',fn,'contactP',pc,'ds',ds,'dp',dp,'dfn',dfn,'df',df,'du',du, ...
   'parameterOrder',{{'normal offsets mm, one per plane','normal tilts toward +y radians, one per plane','tip Fy N'}}, ...
   'rootCondition',sing(1)/sing(end),'rootSingularValues',sing,'forwardTruthShapeErrorMm',max(vecnorm(sh.p-t.p(:,:,k))), ...
   'nominalResidual',norm(r,inf),'Qpar',Qpar,'q',q);
  records{scenario,k}=rec;models{scenario,k}=model;
  for j=1:np
   kind='offset';wall=j;if j>C&&j<=2*C,kind='tilt_y';wall=j-C;elseif j==np,kind='tip_y';wall=0;end
   for i=1:C
    row=struct('scenario',names{scenario},'frame',k,'contact',i,'kind',kind,'perturbedWall',wall, ...
     'normalForceN',fn(i),'ds',ds(i,j),'dpx',dp(1,i,j),'dpy',dp(2,i,j),'dpz',dp(3,i,j), ...
     'dfn',dfn(i,j),'dFx',df(1,i,j),'dFy',df(2,i,j),'dFz',df(3,i,j),'rootCondition',rec.rootCondition);allRows=[allRows;row];
   end
  end
  if k==1
   % Independent +/- re-equilibrations validate derivatives and finite changes.
   for j=1:np
    if j<=C,steps=[0.01 0.005 1];elseif j<=2*C,steps=[1e-3 5e-4];else,steps=[0.01 0.005];end
    for h=steps
     for sign=[-1 1]
      th=theta;th(j)=sign*h;seed=q+Qpar(:,j)*sign*h;
      [qq,rr,s2,y2,flag]=solve(model,seed,th);pen=penetration(model,s2,th);delta=y2-y;linear=D(:,j)*sign*h;
      row=struct('scenario',names{scenario},'parameter',j,'step',sign*h,'exitflag',flag,'rootResidual',norm(rr,inf),'penetrationMm',pen, ...
       'ds',delta(1:C)','dp',reshape(delta(C+1:4*C),3,C),'dfn',delta(4*C+1:5*C)', ...
       'contactPositionLinearErrorMm',max(vecnorm(reshape(delta(C+1:4*C)-linear(C+1:4*C),3,C))), ...
       'arcLinearErrorMm',max(abs(delta(1:C)-linear(1:C))));finiteRows=[finiteRows;row];
     end
    end
   end
   % Translating any infinite plane in y leaves this planar environment identical.
   translated=model;translated.point(2,:)=translated.point(2,:)+1;[rt,st,yt]=evaluate(translated,q,theta);
   rec.yTranslationPositionDifference=max(abs(st.contactP-sh.contactP),[],'all');rec.yTranslationResidualDifference=norm(rt-r,inf);
   % Recompute observation sensitivity with both bending channels on the same mechanics.
   m2=model;m2.fr.axes=[1 2];m2.fr.u=zeros(2,numel(fr.arcs));du2=zeros(2*numel(fr.arcs),np);
   for j=1:np
    h=1e-4;if j>C&&j<=2*C,h=1e-5;end
    tp=theta;tm=theta;tp(j)=h;tm(j)=-h;
    [~,sp]=evaluate(m2,q+Qpar(:,j)*h,tp);[~,sm]=evaluate(m2,q-Qpar(:,j)*h,tm);
    du2(:,j)=(sp.prediction(:)-sm.prediction(:))/(2*h);
   end
   rec.duTwoBendingChannels=du2;rec.tipYCurvatureResponseNorm=norm(du2(:,end));rec.tipYObservedResponseNorm=norm(du(:,end));
   records{scenario,k}=rec;
  end
  fprintf('%s frame %d C=%d Fn=%s condition %.3g nominal %.3g\n',names{scenario},k,C,mat2str(fn,4),rec.rootCondition,rec.nominalResidual);
 end
end
writetable(struct2table(allRows),fullfile(folder,'local_derivatives.csv'));
save(fullfile(folder,'sensitivity.mat'),'records','models','finiteRows','allRows','-v7.3');
fid=fopen(fullfile(folder,'finite_perturbations.json'),'w');fprintf(fid,'%s',jsonencode(finiteRows,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(folder,'first_frames.json'),'w');fprintf(fid,'%s',jsonencode({records{1,1},records{2,1}},PrettyPrint=true));fclose(fid);
end
function [r,sh,y]=evaluate(m,q,theta)
C=m.C;n=m.normal;point=m.point;
for i=1:C,n(:,i)=cos(theta(C+i))*m.normal(:,i)+sin(theta(C+i))*[0;1;0];point(:,i)=point(:,i)+theta(i)*m.normal(:,i);end
s=q(3+C+1:end)'*m.L;fn=q(4:3+C)'*m.Fref;B=zeros(3,2,C);x=zeros(6+3*C,1);x(1:3)=q(1:3)*m.Mref;x(4:6)=m.tip+[0;theta(end);0];
for i=1:C
 B(:,:,i)=tsfs.basis(n(:,i));ft=zeros(3,1);
 if m.mu(i)>0,v=m.slide(:,i)-n(:,i)*(n(:,i)'*m.slide(:,i));ft=-m.mu(i)*fn(i)*v/norm(v);end
 x(6+3*i-2:6+3*i)=[fn(i);B(:,:,i)'*ft];
end
g=struct('s',s,'normal',n,'point',point,'offset',zeros(1,C),'plane',1:C,'B',B,'mu',m.mu);
scale=[repmat(m.Mref,3,1);repmat(m.Fref,3+3*C,1)];sh=tsfs.ivp(m.fr,g,x./scale,scale,m.o,false);
gap=sum(n.*(sh.contactP-point),1);tangent=sum(n.*sh.contactT,1);
r=[sh.tipMoment/m.Mref;gap(:)/m.L;tangent(:)];
y=[s(:);sh.contactP(:);fn(:);sh.force(:);sh.prediction(:)];
end
function [q,r,sh,y,flag]=solve(m,q0,theta)
C=m.C;ss=q0(4+C:end);edges=[0;(ss(1:end-1)+ss(2:end))/2;1];lb=[-Inf(3,1);zeros(C,1);edges(1:end-1)+1e-6];ub=[Inf(3+C,1);edges(2:end)-1e-6];
opts=optimoptions('lsqnonlin','Display','off','FunctionTolerance',1e-15,'StepTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxIterations',80,'MaxFunctionEvaluations',1500,'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-5);
[q,~,r,flag]=lsqnonlin(@(v)evaluate(m,v,theta),q0,lb,ub,opts);[r,sh,y]=evaluate(m,q,theta);
end
function pen=penetration(m,sh,theta)
pen=0;for i=1:m.C,n=cos(theta(m.C+i))*m.normal(:,i)+sin(theta(m.C+i))*[0;1;0];point=m.point(:,i)+theta(i)*m.normal(:,i);pen=max(pen,-min(n'*(sh.collisionP-point)));end
end
