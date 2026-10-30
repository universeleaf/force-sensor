function sol=qp_branch(fr,g,previous,o,cfg,modes,z0,scale,prior)
% Drop-in replacement for tsfs.mode_branch on the shared affine rod model
% (cfg.mechanicalModel). Same branch constraints, objective and outputs; only
% the optimizer differs:
%   separation, sticking inside the cone -> one convex QP (quadprog);
%   sliding, sticking on the cone edge   -> exact profile over the friction
%                                           angle, each angle one convex QP.
% For one contact the branch minimum is global up to the angle grid and
% refinement tolerance; several angle contacts use coordinate descent.
% Two options, both off by default (then the problem equals mode_branch):
%   cfg.forcePriors=true  keeps the framework's existing force-tracking rows
%       (prior.tip/tipStd, prior.force/forceStd from tsfs.initialize);
%       cfg.forcePriorTangentRatio scales the tangential std (default 1).
%   cfg.slipStd='stage1'  weights the slip-history equations as residuals with
%       the Stage 1 position covariance J_p C_w J_p' (plus Stage 1 model floor)
%       instead of treating the noisy previous shape as exact.
clock=tic;model=cfg.mechanicalModel;pc=struct('gridSize',36,'refineCount',3,'angleTolerance',1e-7,'collisionMarginMm',1,'coordinateSweeps',3,'multiStarts',2,'jointLimit',2500,'constraintTolerance',cfg.constraintTolerance);
A=affine_problem(fr,g,model,z0,scale,pc);
if isfield(cfg,'trustRadius')&&isfinite(cfg.trustRadius),A.box=cfg.trustRadius;end
if isfield(cfg,'forcePriors')&&cfg.forcePriors
 ratio=1;if isfield(cfg,'forcePriorTangentRatio'),ratio=cfg.forcePriorTangentRatio;end
 A=add_force_prior(A,g,prior,ratio);
end
if isfield(cfg,'slipStd')&&strcmp(cfg.slipStd,'stage1')&&A.history,A=stage1_slip_weights(fr,A,g,o);end
b=solve_mode(A,modes,pc);z=b.u(1:A.nx);
e=tsfs.linearized_evaluate(fr,g,z,scale,model);N=numel(fr.u);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=.5*(e.r'*e.r);
publicCfg=rmfield(cfg,'mechanicalModel');
sol=struct('z',z,'scale',scale,'prior',prior,'evaluation',e,'converged',b.converged,'reason',b.reason,'iterations',b.qps,'trace',[], ...
 'counts',struct('augmentedIVPs',0,'rhsEvaluations',0,'odeSegments',0,'failedIVPs',0,'affineEvaluations',b.qps),'seconds',toc(clock), ...
 'stationarity',NaN,'objectiveScale',1,'historySource',fr.historySource,'solver','enumerative/profiled-QP','solverStats',struct,'exitflag',b.exitflag, ...
 'multipliers',[],'configuration',publicCfg,'initialZ',z0,'initialY',[],'mode',modes,'modeY',mode_y(A,b),'modeViolation',b.violation, ...
 'modeFeasible',b.feasible,'exception','','derivativeCheck',[],'initialViolation',NaN);
sol.selectionObjective=b.objective;sol.objectiveParts=b.parts;sol.angles=b.angles;sol.coneBoundary=b.boundary;sol.certificate=b.certificate;
end

function A=affine_problem(fr,g,model,z0,scale,pc)
% Affine rows in scaled coordinates z, exactly as in linearized_evaluate.
e=model.e;sh=e.shape;nx=numel(z0);C=numel(g.s);ng=numel(e.h)-C;ns=numel(sh.collisionS);
A=struct('nx',nx,'z0',z0,'zModel',model.z,'scale',scale,'C',C,'mu',g.mu,'history',~isempty(model.history),'box',Inf);
A.R=e.J;A.b=e.J*model.z-e.r;A.nData=size(e.J,1);A.nPrior=0;
A.Meq=e.Jc(1:3,:);A.meq=e.Jc(1:3,:)*model.z-e.c(1:3);
A.Hc=e.Jh(1:ng,:);A.hrhs=e.Jh(1:ng,:)*model.z-e.h(1:ng);A.near=e.h(1:ng)>-pc.collisionMarginMm;
A.cols=zeros(3,C);A.G=zeros(C,nx);A.grhs=zeros(C,1);A.V=zeros(2,nx,C);A.cv=zeros(2,C);A.drop=cell(1,C);
for i=1:C
 cols=6+3*i-2:6+3*i;A.cols(:,i)=cols;JP=reshape(sh.JcontactP(:,i,:),3,nx);n=g.normal(:,g.plane(i));
 A.G(i,:)=n'*JP;A.grhs(i)=A.G(i,:)*model.z-e.gap(i);
 if A.history,A.V(:,:,i)=g.B(:,:,i)'*JP;A.cv(:,i)=e.slip(:,i)-A.V(:,:,i)*model.z;end
 index=find(abs(sh.collisionS-g.s(i))<1e-10);A.drop{i}=(g.plane(i)-1)*ns+index;
end
A.W=repmat(eye(2),1,1,C);A.soft=false;
end

function A=add_force_prior(A,g,prior,ratio)
% Same rows as tsfs.evaluate: (F_tip-prior.tip)/tipStd, (F_i-prior.force_i)/forceStd,
% with the contact difference split into normal and tangent-plane parts.
% ratio scales the tangential std (1: original isotropic prior; Inf: normal
% only), because friction may flip within one frame at slip reversal.
E=zeros(3,A.nx);E(:,4:6)=diag(A.scale(4:6))/prior.tipStd;R=E;b=prior.tip/prior.tipStd;
for i=1:A.C
 cols=A.cols(:,i);n=g.normal(:,g.plane(i));B=g.B(:,:,i);E=zeros(3,A.nx);E(:,cols)=diag(A.scale(cols));
 w=[1;1/ratio;1/ratio]/prior.forceStd;target=[n B]'*prior.force(:,i); % components in [n B]
 keep=w>0;R=[R;w(keep).*E(keep,:)];b=[b;w(keep).*target(keep)];
end
A.R=[A.R;R];A.b=[A.b;b];A.nPrior=size(R,1);
end

function A=stage1_slip_weights(fr,A,g,o)
% Error of the previous contact point modeled like Stage 1 position residuals.
A.soft=true;sh=g.kinematicShape;
for i=1:A.C
 [~,~,Jp]=tsfs.shape_at(sh,g.s(i));Jp=reshape(Jp,3,[]);
 S=g.B(:,:,i)'*(Jp*fr.CU*Jp')*g.B(:,:,i)+o.geometryPositionModelStdMm^2*eye(2);
 A.W(:,:,i)=chol((S+S')/2,'lower')\eye(2);
end
end

function b=solve_mode(A,modes,pc)
C=numel(modes);angles=NaN(1,C);boundary=false(1,C);qps=0;certificate=true;
if ~any(modes==2)
 q=qp_at(A,modes,angles,boundary,pc);qps=1;over=cone_excess(A,modes,q.u,boundary)>1e-10;
 if ~any(over),b=finish(q,modes,angles,boundary,qps,certificate);return,end
 boundary=over; % convex problem: a violated cone is active at the optimum
end
for pass=1:3
 % A frictionless cone edge is tau=0 for every angle: fix it, do not profile.
 fixed=boundary&A.mu==0;angles(fixed)=0;
 K=find(modes==2|(boundary&~fixed));init=initial_angles(A,modes,K);angles(K)=init;
 if isempty(K)
  q=qp_at(A,modes,angles,boundary,pc);qps=qps+1;
 elseif numel(K)==1
  [angles(K),q,n]=profile1(@(t)qp_at(A,modes,setangle(angles,K,t),boundary,pc),init,pc);qps=qps+n;
 else
  % Several angles: coarse joint grid, then coordinate descent from the two
  % best grid points. Not certified, but not limited to one start.
  certificate=false;[starts,n]=joint_starts(A,modes,angles,boundary,K,init,pc);qps=qps+n;best=[];
  for st=1:numel(starts)
   a=starts{st};
   for sweep=1:pc.coordinateSweeps
    for k=K,[a(k),qk,n]=profile1(@(t)qp_at(A,modes,setangle(a,k,t),boundary,pc),a(k),pc);qps=qps+n;end
   end
   if isempty(best)||better(qk,best.q),best=struct('q',qk,'angles',a);end
  end
  q=best.q;angles=best.angles;
 end
 over=cone_excess(A,modes,q.u,boundary)>1e-10;
 if ~any(over),break,end
 % A frictional edge added after an angle profile is a heuristic step; a
 % frictionless edge (tau=0) is exact for every angle.
 if any(over&A.mu>0),certificate=false;end
 boundary=boundary|over;
end
b=finish(q,modes,angles,boundary,qps,certificate);
end

function a=setangle(a,k,t),a(k)=t;end

function [starts,count]=joint_starts(A,modes,angles,boundary,K,init,pc)
% All combinations of a coarse per-angle grid plus the initial direction.
m=12;if numel(K)==3,m=8;elseif numel(K)>3,m=4;end
grids=cell(1,numel(K));for j=1:numel(K),grids{j}=[2*pi*(0:m-1)/m init(j)];end
G=cell(1,numel(K));[G{:}]=ndgrid(grids{:});combos=reshape(cat(numel(K)+1,G{:}),[],numel(K));
combos=combos(1:min(size(combos,1),pc.jointLimit),:);vals=inf(size(combos,1),1);viol=vals;
for c=1:size(combos,1)
 a=angles;a(K)=combos(c,:);q=qp_at(A,modes,a,boundary,pc);viol(c)=q.violation;if q.feasible,vals(c)=q.objective;end
end
count=size(combos,1);
if all(~isfinite(vals)),[~,o]=sort(viol);else,[~,o]=sort(vals);end
starts={};for c=o(1:min(pc.multiStarts,numel(o)))',a=angles;a(K)=combos(c,:);starts{end+1}=a;end
end

function tf=better(q,ref)
% Feasible beats infeasible; then lower objective, or lower violation.
if q.feasible~=ref.feasible,tf=q.feasible;elseif q.feasible,tf=q.objective<ref.objective;else,tf=q.violation<ref.violation;end
end

function init=initial_angles(A,modes,K)
% Starting candidates only; the full angle grid is always evaluated.
init=zeros(1,numel(K));
for m=1:numel(K)
 i=K(m);x=A.z0(A.cols(:,i)).*A.scale(A.cols(:,i));v=A.cv(:,i)+A.V(:,:,i)*A.z0;
 if modes(i)==2&&norm(v)>1e-12,init(m)=atan2(v(2),v(1));elseif modes(i)==2,init(m)=atan2(-x(3),-x(2));else,init(m)=atan2(x(3),x(2));end
end
end

function [best,qbest,count]=profile1(fun,init,pc)
% Periodic angle profile: full grid, then bounded refinement of the best
% local grid minima. Infeasible angles are excluded.
grid=2*pi*(0:pc.gridSize-1)/pc.gridSize;cands=[grid init(:)'];vals=inf(1,numel(cands));Q=cell(1,numel(cands));count=0;
for k=1:numel(cands),Q{k}=fun(cands(k));count=count+1;if Q{k}.feasible,vals(k)=Q{k}.objective;end,end
gv=vals(1:numel(grid));isMin=isfinite(gv)&gv<=circshift(gv,1)&gv<=circshift(gv,-1);starts=grid(isMin);[~,o]=sort(gv(isMin));starts=starts(o(1:min(pc.refineCount,numel(o))));
if isfinite(vals(end)),starts=[starts init(:)'];end
h=2*pi/pc.gridSize;opts=optimset('TolX',pc.angleTolerance,'Display','off');
for s=starts
 [t,~,~,out]=fminbnd(@(t)value(fun(t)),s-h,s+h,opts);count=count+out.funcCount;
 Q{end+1}=fun(t);count=count+1;cands(end+1)=t;vals(end+1)=inf;if Q{end}.feasible,vals(end)=Q{end}.objective;end
end
if all(~isfinite(vals)),[~,k]=min(cellfun(@(q)q.violation,Q));else,[~,k]=min(vals);end
best=mod(cands(k)+pi,2*pi)-pi;qbest=Q{k};
end

function v=value(q),if q.feasible,v=q.objective;else,v=1e30*(1+q.violation);end,end

function excess=cone_excess(A,modes,u,boundary)
excess=zeros(1,numel(modes));
for i=find(modes==1&~boundary),x=u(A.cols(:,i)).*A.scale(A.cols(:,i));excess(i)=norm(x(2:3))-A.mu(i)*x(1);end
end

function q=qp_at(A,modes,angles,boundary,pc)
% Convex QP for fixed modes and friction angles; u=[z;a], a = slip amplitude
% of each sliding contact. Constraints follow mode_branch with the angle fixed.
persistent opts
if isempty(opts),opts=optimoptions('quadprog','Algorithm','active-set','Display','off','MaxIterations',1000);end
nx=A.nx;slide=find(modes==2);na=numel(slide);nu=nx+na;s=A.scale;
R=[A.R zeros(size(A.R,1),na)];rb=A.b;Aeq=[A.Meq zeros(3,na)];beq=A.meq;lb=-inf(nu,1);ub=inf(nu,1);lb(nx+1:end)=0;drop=false(size(A.Hc,1),1);
if isfinite(A.box),lb(1:nx)=A.zModel-A.box;ub(1:nx)=A.zModel+A.box;end
for i=1:A.C
 cols=A.cols(:,i);
 if modes(i)==0,E=zeros(3,nu);E(:,cols)=eye(3);Aeq=[Aeq;E];beq=[beq;0;0;0];continue,end
 row=zeros(1,nu);row(1:nx)=A.G(i,:);Aeq=[Aeq;row];beq=[beq;A.grhs(i)];drop(A.drop{i})=true;lb(cols(1))=max(lb(cols(1)),0);
 d=[cos(angles(i));sin(angles(i))];k=find(slide==i);
 if modes(i)==2||boundary(i)
  sgn=1;if modes(i)==1,sgn=-1;end % sliding tau=-mu*lambda*d; cone edge tau=+mu*lambda*d
  E=zeros(2,nu);E(:,cols(2:3))=diag(s(cols(2:3)));E(:,cols(1))=sgn*A.mu(i)*s(cols(1))*d;Aeq=[Aeq;E];beq=[beq;0;0];
 end
 if ~A.history
  if modes(i)==2,E=zeros(2,nu);E(:,nx+k)=d;Aeq=[Aeq;E];beq=[beq;0;0];end % no history: zero slip
  continue
 end
 V=zeros(2,nu);V(:,1:nx)=A.V(:,:,i);if modes(i)==2,V(:,nx+k)=-d;end
 if A.soft,W=A.W(:,:,i);R=[R;W*V];rb=[rb;-W*A.cv(:,i)];else,Aeq=[Aeq;V];beq=[beq;-A.cv(:,i)];end
end
keep=any(Aeq~=0,2)|beq~=0;Aeq=Aeq(keep,:);beq=beq(keep);
H=R'*R;H=(H+H')/2;H=H+1e-12*max(1,max(abs(diag(H))))*eye(nu);f=-R'*rb;
work=A.near&~drop;u0=[A.z0;zeros(na,1)];Hc=[A.Hc zeros(size(A.Hc,1),na)];
for loop=1:8 % constraint generation over sampled non-penetration rows
 [u,~,exitflag]=quadprog(H,f,Hc(work,:),A.hrhs(work),Aeq,beq,lb,ub,u0,opts);
 if isempty(u),u=u0;break,end
 h=Hc*u-A.hrhs;bad=h>1e-10&~drop&~work;if ~any(bad),break,end
 work=work|bad;u0=u;
end
h=Hc*u-A.hrhs;h(drop)=-Inf;viol=max([abs(Aeq*u-beq);h;lb-u;u-ub;0]);
r=R*u-rb;nd=A.nData;np=A.nPrior;parts=[.5*sum(r(1:nd).^2) .5*sum(r(nd+1:nd+np).^2) .5*sum(r(nd+np+1:end).^2)];
q=struct('u',u,'objective',sum(parts),'parts',parts,'violation',viol,'feasible',exitflag>0&&viol<=pc.constraintTolerance,'exitflag',exitflag);
end

function b=finish(q,modes,angles,boundary,qps,certificate)
reason=sprintf('quadprog-exit-%d',q.exitflag);if ~q.feasible,reason=[reason '-infeasible'];end
b=struct('u',q.u,'modes',modes,'angles',angles,'boundary',boundary,'objective',q.objective,'parts',q.parts,'violation',q.violation, ...
 'feasible',q.feasible,'converged',q.exitflag>0,'exitflag',q.exitflag,'qps',qps,'certificate',certificate,'reason',reason);
end

function y=mode_y(A,b)
% Same layout as mode_branch: [z(1:6); per contact (lambda,tau) | (lambda,theta,a)].
z=b.u(1:A.nx);y=z(1:6);slide=find(b.modes==2);
for i=1:A.C
 cols=A.cols(:,i);
 switch b.modes(i)
  case 1,y=[y;z(cols)];
  case 2,k=find(slide==i);y=[y;z(cols(1));b.angles(i);b.u(A.nx+k)];
 end
end
end
