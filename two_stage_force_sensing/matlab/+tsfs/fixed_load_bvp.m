function [z,info]=fixed_load_bvp(fr,g,z,scale,seed,cfg)
% Nonlinear equilibrium correction with prescribed tip/contact forces.
% Contact points are material load locations, not enforced wall constraints.
clock=tic;bishop=tsfs.is_bishop(fr);ny=15+double(bishop);L=fr.tube.s(end)-fr.tube.s(1);s0=fr.tube.s(1);M=max(scale(1:3));x=z.*scale;
K=reshape(getTubeK(fr.tube),3,[]);edges=seed.edges;regions=numel(edges)-1;data=cell(1,regions);mesh=[];
for k=1:regions
 mid=mean(edges(k:k+1));node=find(fr.tube.s<=mid,1,'last');load=x(4:6)+sum(seed.force(:,g.s>mid),2);
 data{k}=struct('u0',fr.tube.uhat(:,node),'K',K(:,node),'load',load);
 nodes=linspace((edges(k)-s0)/L,(edges(k+1)-s0)/L,max(4,ceil(diff(edges(k:k+1))/cfg.meshStepMm)+1));mesh=[mesh nodes];
end
rhsCalls=0;jacCalls=0;bcCalls=0;lastwarn('');init=bvpinit(mesh,@guess);opts=bvpset('RelTol',cfg.relativeTolerance,'AbsTol',cfg.absoluteTolerance,'NMax',cfg.maxMeshNodes,'FJacobian',@jac,'Stats','off');
bvp=bvp4c(@rhs,@bc,init,opts);[warningText,warningId]=lastwarn;z(1:3)=bvp.y(13:15,1)*M./scale(1:3);
Y=deval(bvp,(seed.s-s0)/L);p=L*Y(1:3,:);R=reshape(Y(4:12,:),3,3,[]);rotationError=0;for i=1:size(R,3),rotationError=max(rotationError,norm(R(:,:,i)'*R(:,:,i)-eye(3),'fro'));end
boundary=bc(bvp.y(:,[1 find(diff(bvp.x)==0)+1]),bvp.y(:,[find(diff(bvp.x)==0) numel(bvp.x)]));
info=struct('solver','MATLAB bvp4c multipoint collocation','seconds',toc(clock),'rhsEvaluations',rhsCalls,'jacobianEvaluations',jacCalls,'boundaryEvaluations',bcCalls,'initialNodes',numel(mesh),'finalNodes',numel(bvp.x),'regions',regions,'stats',bvp.stats,'warning',warningText,'warningId',warningId,'boundaryResidual',norm(boundary,inf),'collocationTipMomentNmm',norm(bvp.y(13:15,end)*M,inf),'rotationOrthogonality',rotationError,'baseMomentBeforeNmm',x(1:3),'baseMomentAfterNmm',z(1:3).*scale(1:3),'shapeChangeRmsMm',sqrt(mean(sum((p-seed.p).^2,1))),'p',p,'configuration',cfg,'converged',isempty(warningId)&&norm(boundary,inf)<=1e-6,'forceChangeN',0);
 function q=guess(t,k)
  if nargin<2,k=1;end
  ss=s0+L*t;assert(ss>=edges(k)-1e-8&&ss<=edges(k+1)+1e-8,'tsfs:BvpRegion','Initial guess outside region.');ss=min(edges(k+1),max(edges(k),ss));q=deval(seed.pieces{k},ss);q=q(1:ny);q(1:3)=q(1:3)/L;q(13:15)=q(13:15)/M;
 end
 function dq=rhs(~,q,k)
  if nargin<3,k=1;end
  rhsCalls=rhsCalls+1;d=data{k};rot=reshape(q(4:12),3,3);m=q(13:15)*M;u=d.u0+(rot'*m)./d.K;rr=L*rot*tsfs.hat(u);dq=[rot(:,3);rr(:);L/M*tsfs.hat(d.load)*rot(:,3)];if bishop,dq=[dq;L*u(3)];end
 end
 function J=jac(~,q,k)
  if nargin<3,k=1;end
  jacCalls=jacCalls+1;d=data{k};rot=reshape(q(4:12),3,3);m=q(13:15)*M;u=d.u0+(rot'*m)./d.K;
  duR=kron(eye(3),m')./d.K;dum=M*rot'./d.K;H=zeros(9,3);for a=1:3,v=zeros(3,1);v(a)=1;tmp=rot*tsfs.hat(v);H(:,a)=tmp(:);end
  J=zeros(ny);J(1:3,10:12)=eye(3);J(4:12,4:12)=L*(kron(tsfs.hat(u)',eye(3))+H*duR);J(4:12,13:15)=L*H*dum;J(13:15,10:12)=L/M*tsfs.hat(d.load);
  if bishop,J(16,4:12)=L*duR(3,:);J(16,13:15)=L*dum(3,:);end
 end
 function rr=bc(yl,yr)
  bcCalls=bcCalls+1;rr=[yl(1:3,1)-fr.base(1:3,4)/L;yl(4:12,1)-reshape(fr.base(1:3,1:3),9,1)];if bishop,rr=[rr;yl(16,1)];end
  rr=[rr;yr(13:15,end)];for k=1:regions-1,rr=[rr;yr(:,k)-yl(:,k+1)];end
 end
end
