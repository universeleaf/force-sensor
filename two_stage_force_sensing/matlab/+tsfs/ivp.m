function sh=ivp(fr,g,z,scale,o,derivatives)
% Analytic variational IVP in scaled optimization coordinates. No shooting.
x=z.*scale;C=numel(g.s);nx=numel(z);tube=fr.tube;s=tube.s(:)';K=reshape(getTubeK(tube),3,[]);
F=zeros(3,C);DF=zeros(3,nx,C);
for i=1:C
 A=[g.normal(:,g.plane(i)) g.B(:,:,i)];cols=6+3*i-2:6+3*i;F(:,i)=A*x(cols);DF(:,cols,i)=A*diag(scale(cols));
end
change=find(any(abs(diff(tube.uhat,1,2))>1e-14,1)|any(abs(diff(K,1,2))>1e-10,1))+1;
edges=unique([s(1) s(change) g.s s(end)]);pieces=cell(1,numel(edges)-1);rhsCount=0;
y=[fr.base(1:3,4);reshape(fr.base(1:3,1:3),9,1);x(1:3)];
if derivatives,S=zeros(15,nx);S(13:15,1:3)=diag(scale(1:3));y=[y;S(:)];end
stateScale=[repmat(s(end)-s(1),3,1);ones(9,1);repmat(max(scale(1:3)),3,1)];
absTol=o.odeAbsoluteTolerance*stateScale;
if derivatives,absTol=[absTol;repmat(absTol,nx,1)];end
odeOpts=odeset('RelTol',o.odeRelativeTolerance,'AbsTol',absTol);
for k=1:numel(pieces)
 mid=mean(edges(k:k+1));node=find(s<=mid,1,'last');u0=tube.uhat(:,node);kk=K(:,node);
 load=x(4:6)+sum(F(:,g.s>mid),2);DL=zeros(3,nx);DL(:,4:6)=diag(scale(4:6));DL=DL+sum(DF(:,:,g.s>mid),3);
 pieces{k}=ode45(@rhs,edges(k:k+1),y,odeOpts);y=pieces{k}.y(:,end);
end
collision=unique([s s(1):o.collisionStepMm:s(end) g.s]);
Y=sample(s);YC=sample(g.s);YG=sample(collision);R=reshape(Y(4:12,:),3,3,[]);u=tube.uhat;
Ju=zeros(3,numel(s),nx);
for k=1:numel(s)
 rot=R(:,:,k);m=Y(13:15,k);u(:,k)=u(:,k)+(rot'*m)./K(:,k);
 if derivatives,SS=reshape(Y(16:end,k),15,nx);Ju(:,k,:)=(kron(eye(3),m')*SS(4:12,:)+rot'*SS(13:15,:))./K(:,k);end
end
prediction=interp1(s,u(fr.axes,:)',fr.arcs(:),'linear')';Jpred=zeros(numel(prediction),nx);
if derivatives
 for j=1:nx,uu=interp1(s,Ju(fr.axes,:,j)',fr.arcs(:),'linear')';Jpred(:,j)=uu(:);end
end
sh=struct('s',s,'p',Y(1:3,:),'R',R,'u',u,'tipMoment',y(13:15),'contactP',YC(1:3,:), ...
 'contactT',YC(10:12,:),'collisionS',collision,'collisionP',YG(1:3,:),'prediction',prediction, ...
 'Jprediction',Jpred,'JtipMoment',zeros(3,nx),'JcontactP',zeros(3,C,nx),'JcollisionP',zeros(3,numel(collision),nx), ...
 'rhsEvaluations',rhsCount,'segments',numel(pieces),'stateDimension',numel(y),'pieces',{pieces},'edges',edges,'force',F,'x',x);
if derivatives
 SS=reshape(y(16:end),15,nx);sh.JtipMoment=SS(13:15,:);
 for k=1:C,SS=reshape(YC(16:end,k),15,nx);sh.JcontactP(:,k,:)=SS(1:3,:);end
 for k=1:numel(collision),SS=reshape(YG(16:end,k),15,nx);sh.JcollisionP(:,k,:)=SS(1:3,:);end
end
 function dy=rhs(~,q)
  rhsCount=rhsCount+1;if rhsCount>100000,error('tsfs:RHSBudget','IVP RHS budget exceeded.');end
  rot=reshape(q(4:12),3,3);m=q(13:15);t=rot(:,3);v=u0+(rot'*m)./kk;Hv=tsfs.hat(v);dR=rot*Hv;
  dy=[t;dR(:);tsfs.hat(load)*t];
  if derivatives
   SS=reshape(q(16:end),15,nx);SR=SS(4:12,:);Sm=SS(13:15,:);St=SS(10:12,:);
   du=(kron(eye(3),m')*SR+rot'*Sm)./kk;HR=zeros(9,3);
   for a=1:3,e=zeros(3,1);e(a)=1;tmp=rot*tsfs.hat(e);HR(:,a)=tmp(:);end
   dS=[St;kron(Hv',eye(3))*SR+HR*du;tsfs.hat(load)*St-tsfs.hat(t)*DL];dy=[dy;dS(:)];
  end
 end
 function Q=sample(arcs)
  Q=zeros(numel(y),numel(arcs));
  for a=1:numel(pieces),take=arcs>=edges(a)&arcs<=edges(a+1);if any(take),Q(:,take)=deval(pieces{a},arcs(take));end,end
 end
end
