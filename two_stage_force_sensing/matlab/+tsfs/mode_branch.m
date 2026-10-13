function sol=mode_branch(fr,g,previous,o,cfg,modes,z0,scale,prior)
% Fixed integer assignment with exact circular friction and full nonlinear IVP.
% 0 separation, 1 sticking, 2 sliding, 3 frictionless active contact.
clock=tic;affine=isfield(cfg,'mechanicalModel');C=numel(g.s);nx=numel(z0);history=[];historySource=fr.historySource;
if ~isempty(fr.previous)
 if ~isempty(previous)&&previous.valid&&abs(previous.time-fr.previous.time)<1e-9
  history=previous.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];historySource='previous-mechanical-posterior';
 else,history=tsfs.reconstruct(fr.previous,false);end
end
slots=cell(1,C);y0=z0(1:6);lb=-Inf(6,1);ub=Inf(6,1);
for i=1:C
 cols=6+3*i-2:6+3*i;start=numel(y0);
 switch modes(i)
 case 0,slots{i}=[];
 case 1
  lambda=max(0,z0(cols(1)));tau=z0(cols(2:3));R=g.mu(i)*lambda;if norm(tau)>R,tau=tau*R/max(norm(tau),realmin);end
  slots{i}=start+(1:3);y0=[y0;lambda;tau];lb=[lb;0;-Inf;-Inf];ub=[ub;Inf;Inf;Inf];
 case 2
  vv=zeros(2,1);if ~isempty(history),vv=g.B(:,:,i)'*(g.predictedPoint(:,i)-tsfs.shape_at(history,g.s(i)));end
  angle=atan2(vv(2),vv(1));if norm(vv)<1e-10&&norm(z0(cols(2:3)))>1e-10,angle=atan2(-z0(cols(3)),-z0(cols(2)));end
  slots{i}=start+(1:3);y0=[y0;max(0,z0(cols(1)));angle;norm(vv)];lb=[lb;0;-Inf;0];ub=[ub;Inf;Inf;Inf];
 case 3,slots{i}=start+1;y0=[y0;max(0,z0(cols(1)))];lb=[lb;0];ub=[ub;Inf];
 end
end
ny=numel(y0);counts=struct('augmentedIVPs',0,'rhsEvaluations',0,'odeSegments',0,'failedIVPs',0,'affineEvaluations',0);cacheY=[];cache=[];accepted=[];trace=[];
initial=values(y0);accepted=initial;derivativeCheck=[];
if isfield(cfg,'checkDerivatives')&&cfg.checkDerivatives
 base=initial;fd=zeros(1+numel(base.eq)+numel(base.ineq),ny);
 for j=1:ny
  step=1e-6*max(1,abs(y0(j)));yp=y0;ym=y0;yp(j)=yp(j)+step;ym(j)=ym(j)-step;vp=values(yp);vm=values(ym);fd(:,j)=([vp.f;vp.eq;vp.ineq]-[vm.f;vm.eq;vm.ineq])/(2*step);
 end
 exact=[base.df';base.Deq;base.Dineq];derivativeCheck=norm(fd-exact,'fro')/max(1,norm(exact,'fro'));assert(derivativeCheck<1e-4,'Mode branch derivative check failed');
end
maxInner=300;if isfield(cfg,'maxIterationsPerAssignment'),maxInner=cfg.maxIterationsPerAssignment;end
opts=optimoptions('fmincon','Algorithm','sqp','Display','off','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'MaxIterations',maxInner,'MaxFunctionEvaluations',5000,'OptimalityTolerance',1e-8,'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',1e-12,'ScaleProblem',false,'OutputFcn',@logstep);
exception='';
try
 [y,~,exitflag,stats,multipliers]=fmincon(@objective,y0,[],[],[],[],lb,ub,@constraints,opts);last=values(y);reason=sprintf('fmincon-exit-%d',exitflag);
catch failure
 last=accepted;y=last.y;exitflag=-99;exception=[failure.identifier ': ' failure.message];reason=['callback-exception-' failure.identifier];stats=struct('iterations',max([0;vertcat(trace.iteration)]),'firstorderopt',NaN);multipliers=[];
end
violation=max([abs(last.eq);last.ineq;lb-y;y-ub;0]);
publicCfg=cfg;if affine,publicCfg=rmfield(publicCfg,'mechanicalModel');end;if isfield(publicCfg,'sharedModel'),publicCfg=rmfield(publicCfg,'sharedModel');end
sol=struct('z',last.z,'scale',scale,'prior',prior,'evaluation',last.e,'converged',exitflag>0,'reason',reason,'iterations',stats.iterations,'trace',trace,'counts',counts,'seconds',toc(clock),'stationarity',stats.firstorderopt,'objectiveScale',1e-4,'historySource',historySource,'solver','enumerative-MINLP/fmincon-SQP','solverStats',stats,'exitflag',exitflag,'multipliers',multipliers,'configuration',publicCfg,'initialZ',z0,'initialY',y0,'mode',modes,'modeY',y,'modeViolation',violation,'modeFeasible',violation<=cfg.constraintTolerance,'exception',exception,'derivativeCheck',derivativeCheck,'initialViolation',max([abs(initial.eq);initial.ineq;0]));
 function [z,D]=decode(y)
  z=zeros(nx,1);z(1:6)=y(1:6);D=zeros(nx,ny);D(1:6,1:6)=eye(6);
  for ii=1:C
   cc=6+3*ii-2:6+3*ii;ss=slots{ii};
   switch modes(ii)
   case 1,z(cc)=y(ss);D(cc,ss)=eye(3);
   case 2
    lambda=y(ss(1))*scale(cc(1));angle=y(ss(2));unit=[cos(angle);sin(angle)];perp=[-sin(angle);cos(angle)];z(cc(1))=y(ss(1));z(cc(2:3))=-g.mu(ii)*lambda*unit./scale(cc(2:3));D(cc(1),ss(1))=1;D(cc(2:3),ss(1))=-g.mu(ii)*scale(cc(1))*unit./scale(cc(2:3));D(cc(2:3),ss(2))=-g.mu(ii)*lambda*perp./scale(cc(2:3));
   case 3,z(cc(1))=y(ss);D(cc(1),ss)=1;
   end
  end
 end
 function v=values(y)
  if isequal(y,cacheY),v=cache;return,end
  [z,D]=decode(y);
  if affine
   e=tsfs.linearized_evaluate(fr,g,z,scale,cfg.mechanicalModel);counts.affineEvaluations=counts.affineEvaluations+1;
  else
   counts.augmentedIVPs=counts.augmentedIVPs+1;
   try,e=tsfs.evaluate(fr,g,z,scale,prior,history,o,true);catch err,counts.failedIVPs=counts.failedIVPs+1;rethrow(err);end
   counts.rhsEvaluations=counts.rhsEvaluations+e.shape.rhsEvaluations;counts.odeSegments=counts.odeSegments+e.shape.segments;
  end
  N=numel(fr.u);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=.5*(e.r'*e.r);sh=e.shape;
  eq=e.c(1:3);Deq=e.Jc(1:3,:)*D;ng=numel(e.h)-C;keep=true(ng,1);ns=numel(sh.collisionS);
  for ii=1:C
   if modes(ii)>0
    % Exact gap equality replaces its identical sampled gap inequality.
    index=find(abs(sh.collisionS-g.s(ii))<1e-10);keep((g.plane(ii)-1)*ns+index)=false;
   end
  end
  ineq=e.h(1:ng);Dineq=e.Jh(1:ng,:)*D;ineq=ineq(keep);Dineq=Dineq(keep,:);
  for ii=1:C
   cc=6+3*ii-2:6+3*ii;ss=slots{ii};JP=reshape(sh.JcontactP(:,ii,:),3,nx)*D;
   if modes(ii)>0,eq=[eq;e.gap(ii)];Deq=[Deq;g.normal(:,g.plane(ii))'*JP];end
   if modes(ii)==1
    eq=[eq;e.slip(:,ii)];Deq=[Deq;g.B(:,:,ii)'*JP];tau=sh.x(cc(2:3));nt=norm(tau);row=zeros(1,ny);row(ss(1))=-g.mu(ii)*scale(cc(1));if nt>0,row(ss(2:3))=(tau'/nt).*scale(cc(2:3))';end
    ineq=[ineq;nt-g.mu(ii)*sh.x(cc(1))];Dineq=[Dineq;row];
   elseif modes(ii)==2
    angle=y(ss(2));amp=y(ss(3));unit=[cos(angle);sin(angle)];perp=[-sin(angle);cos(angle)];eq=[eq;e.slip(:,ii)-amp*unit];row=g.B(:,:,ii)'*JP;row(:,ss(2))=row(:,ss(2))-amp*perp;row(:,ss(3))=row(:,ss(3))-unit;Deq=[Deq;row];
   end
  end
  if affine&&isfinite(cfg.trustRadius)
   dz=z-cfg.mechanicalModel.z;ineq=[ineq;dz-cfg.trustRadius;-dz-cfg.trustRadius];Dineq=[Dineq;D;-D];
  end
  v=struct('y',y,'z',z,'e',e,'f',1e-4*e.objective,'df',1e-4*D'*(e.J'*e.r),'eq',eq,'Deq',Deq,'ineq',ineq,'Dineq',Dineq);cacheY=y;cache=v;
 end
 function [f,df]=objective(y),v=values(y);f=v.f;df=v.df;end
 function [h,c,Dh,Dc]=constraints(y),v=values(y);h=v.ineq;c=v.eq;Dh=v.Dineq';Dc=v.Deq';end
 function stop=logstep(y,vals,state)
  stop=false;if strcmp(state,'iter')||strcmp(state,'init'),v=values(y);accepted=v;trace=[trace;struct('iteration',vals.iteration,'objective',v.e.objective,'constraintViolation',max([abs(v.eq);v.ineq;0]),'firstorderopt',vals.firstorderopt,'funcCount',vals.funccount,'seconds',toc(clock),'y',y')];end
 end
end
