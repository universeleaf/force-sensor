function [z,scale,prior]=initialize(fr,g,previous,o)
C=numel(g.s);nx=6+3*C;L=fr.tube.s(end)-fr.tube.s(1);Fscale=1;
scale=[repmat(L*Fscale,3,1);repmat(Fscale,nx-3,1)];
sh=g.kinematicShape;K=reshape(getTubeK(fr.tube),3,[]);p=sh.p;N=numel(sh.s);
% Linear inverse moment balance on observed kinematics supplies a start only.
A=zeros(3*N,3+3*C);b=zeros(3*N,1);
for k=1:N
 rows=3*k-2:3*k;A(rows,1:3)=tsfs.hat(p(:,end)-p(:,k));
 for i=1:C,if g.s(i)>sh.s(k),A(rows,3+3*i-2:3+3*i)=tsfs.hat(g.predictedPoint(:,i)-p(:,k))*[g.normal(:,g.plane(i)) g.B(:,:,i)];end,end
 b(rows)=sh.R(:,:,k)*(K(:,k).*(sh.u(:,k)-fr.tube.uhat(:,k)));
end
reg=1e-6*max(norm(A,'fro'),1);forces=[A;reg*eye(size(A,2))]\[b;zeros(size(A,2),1)];
x=zeros(nx,1);x(4:end)=forces;fc=zeros(3,C);
for i=1:C,cols=6+3*i-2:6+3*i;x(cols(1))=max(0,x(cols(1)));fc(:,i)=[g.normal(:,g.plane(i)) g.B(:,:,i)]*x(cols);end
x(1:3)=cross(p(:,end)-p(:,1),x(4:6));
for i=1:C,x(1:3)=x(1:3)+cross(g.predictedPoint(:,i)-p(:,1),fc(:,i));end
prior=struct('tip',zeros(3,1),'force',zeros(3,C),'tipStd',o.tipPriorStd,'forceStd',o.forcePriorStd,'matched',zeros(1,C));
if ~isempty(previous)&&o.useForceTracking&&previous.valid
 dt=max(fr.time-previous.time,o.referencePeriodSeconds);prior.tip=previous.tipForce;
 prior.tipStd=o.tipProcessStd*sqrt(dt/o.referencePeriodSeconds);prior.forceStd=o.forceProcessStd*sqrt(dt/o.referencePeriodSeconds);
 for i=1:C
  pool=find(previous.geometry.plane==g.plane(i));if isempty(pool),continue,end
  [d,j]=min(abs(previous.geometry.s(pool)-g.s(i)));j=pool(j);
  if d<10&&~ismember(j,prior.matched),prior.force(:,i)=previous.contactForce(:,j);prior.matched(i)=j;end
 end
end
z=x./scale;
end
