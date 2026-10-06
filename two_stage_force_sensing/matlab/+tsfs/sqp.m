function sol=sqp(fr,g,previous,o)
clock=tic;[z,scale,prior]=tsfs.initialize(fr,g,previous,o);nx=numel(z);C=numel(g.s);
history=[];historySource=fr.historySource;
if ~isempty(fr.previous)
 if ~isempty(previous)&&previous.valid&&abs(previous.time-fr.previous.time)<1e-9
  history=previous.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];historySource='previous-mechanical-posterior';
 else,history=tsfs.reconstruct(fr.previous,false);end
end
counts=struct('stateIVPs',0,'augmentedIVPs',0,'rhsEvaluations',0,'odeSegments',0,'trialRejections',0,'qpIterations',0,'maxStateDimension',15);
e=ev(z,true);objectiveScale=1/max(1,max(diag(e.J'*e.J)));nu=o.initialMeritPenalty;delta=o.initialTrustRadius;damping=o.initialDamping;
trace=[];converged=false;reason='iteration-limit';stationarity=Inf;trialCount=0;
qpOpts=optimoptions('quadprog','Algorithm','active-set','Display','off','MaxIterations',o.qpMaxIterations, ...
 'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-8);
normalCols=7:3:nx;
for iteration=1:o.maxIterations
 grad=objectiveScale*(e.J'*e.r);H=objectiveScale*(e.J'*e.J);H=(H+H')/2+damping*eye(nx);
 nc=numel(e.c);nh=numel(e.h);nz=nx+2*nc+1;
 HH=zeros(nz);HH(1:nx,1:nx)=H;HH(nx+1:end,nx+1:end)=1e-12*eye(2*nc+1);
 ff=[grad;nu*ones(2*nc,1);nu];
 AA=[e.Jh zeros(nh,2*nc) -ones(nh,1)];bb=-e.h;
 AE=[e.Jc -eye(nc) eye(nc) zeros(nc,1)];be=-e.c;
 lb=[-delta*ones(nx,1);zeros(2*nc+1,1)];ub=[delta*ones(nx,1);Inf(2*nc+1,1)];
 lb(normalCols)=max(lb(normalCols),-z(normalCols));
 x0=[zeros(nx,1);max(e.c,0);max(-e.c,0);max([0;e.h])];
 [q,~,qpflag,qpout,mult]=quadprog(HH,ff,AA,bb,AE,be,lb,ub,x0,qpOpts);
 counts.qpIterations=counts.qpIterations+qpout.iterations;
 if isempty(q)||any(~isfinite(q)),reason='qp-failure';break,end
 d=q(1:nx);elastic=max(q(nx+1:end));
 lambda=mult.eqlin;gamma=mult.ineqlin;
 % Bounds on a local trust box are NOT constraints of the original problem.
 lag=grad+e.Jc'*lambda+e.Jh'*gamma;
 for col=normalCols,if z(col)<=o.normalTolerance,lag(col)=min(0,lag(col));end,end
 dualComplementarity=max([0;abs(gamma.*e.h)]);
 stationarity=max(norm(lag,inf),dualComplementarity);phys=physical(e);
 row=struct('iteration',iteration,'objective',e.objective,'scaledObjective',objectiveScale*e.objective, ...
  'eqInf',norm(e.c,inf),'penetration',max([0;e.h(1:size(g.normal,2)*numel(e.shape.collisionS))]),'inequalityInf',max([0;e.h]),'stationarity',stationarity, ...
  'stepInf',norm(d,inf),'elasticInf',elastic,'trustRadius',delta,'damping',damping,'penalty',nu, ...
  'qpExit',qpflag,'qpIterations',qpout.iterations,'alpha',0,'accepted',false,'seconds',toc(clock));
 trace=[trace;row];
 if phys&&stationarity<=o.stationarityTolerance&&qpflag>0
  converged=true;reason='stationary-and-feasible';break
 end
 if iteration>3 && norm(d,inf)<1e-7 && stationarity<=o.stationarityTolerance && ~phys
  if nu<1e6
   nu=min(1e6,10*nu);delta=max(delta,0.25);damping=min(damping,o.initialDamping);continue
  end
  reason='stationary-infeasible';break
 end
 if trialCount>=o.maxTrials,reason='trial-limit';break,end
 violation=@(v)norm(v.c,1)+max([0;v.h]);
 current=objectiveScale*e.objective+nu*violation(e);
 linearViolation=norm(e.c+e.Jc*d,1)+max([0;e.h+e.Jh*d]);
 predicted=-(grad'*d+0.5*d'*H*d+nu*(linearViolation-violation(e)));
 accepted=false;alpha=1;
 for ls=1:12
  trialCount=trialCount+1;
  if trialCount>o.maxTrials,break,end
  try
   t=ev(z+alpha*d,false);merit=objectiveScale*t.objective+nu*violation(t);
   accepted=isfinite(merit)&&merit<=current-1e-4*alpha*max(predicted,1e-14);
  catch err
   if ~startsWith(err.identifier,'tsfs:')&&~startsWith(err.identifier,'MATLAB:ode'),rethrow(err),end
   accepted=false;
  end
  if accepted,break,end
  counts.trialRejections=counts.trialRejections+1;alpha=alpha/2;
 end
 trace(iteration).alpha=alpha;trace(iteration).accepted=accepted;
 if accepted
  z=z+alpha*d;e=ev(z,true);
  if alpha==1&&predicted>0
   ratio=(current-merit)/predicted;
   if ratio>0.75,delta=min(8,2*delta);damping=max(1e-12,damping/3);end
  else,delta=max(1e-7,delta/2);damping=min(1e2,3*damping);end
 else
  delta=delta/2;damping=min(1e4,10*damping);
  if elastic>1e-6&&nu<1e8,nu=min(1e8,10*nu);end
 end
 if norm(alpha*d,inf)<1e-7&&~accepted&&delta<1e-6,reason='stagnation';break,end
 if o.verbose,fprintf('SQP %d cost %.4g eq %.3g gap %.3g KKT %.3g step %.3g accepted %d\n',iteration,e.objective,norm(e.c,inf),max([0;e.h]),stationarity,norm(d,inf),accepted);end
end
sol=struct('z',z,'scale',scale,'prior',prior,'evaluation',e,'converged',converged,'reason',reason, ...
 'iterations',numel(trace),'trace',trace,'counts',counts,'seconds',toc(clock),'stationarity',stationarity, ...
 'objectiveScale',objectiveScale,'historySource',historySource,'trialEvaluations',trialCount);
 function v=ev(zz,derivatives)
  v=tsfs.evaluate(fr,g,zz,scale,prior,history,o,derivatives);
  if derivatives,counts.augmentedIVPs=counts.augmentedIVPs+1;else,counts.stateIVPs=counts.stateIVPs+1;end
  counts.rhsEvaluations=counts.rhsEvaluations+v.shape.rhsEvaluations;counts.odeSegments=counts.odeSegments+v.shape.segments;
  counts.maxStateDimension=max(counts.maxStateDimension,v.shape.stateDimension);
 end
 function ok=physical(v)
  if C==0,ok=norm(v.shape.tipMoment,inf)<=o.momentTolerance&&max([0;v.h])<=o.gapTolerance;return,end
  fn=v.shape.x(normalCols);cone=0;
  for a=1:C,cols=6+3*a-2:6+3*a;cone=max(cone,norm(v.shape.x(cols(2:3)))-g.mu(a)*fn(a));end
  ok=norm(v.shape.tipMoment,inf)<=o.momentTolerance&&max(abs(v.projection),[],'all')<=o.projectionTolerance&& ...
   max([0;v.h])<=o.gapTolerance&&all(fn>=-o.normalTolerance)&&cone<=o.coneTolerance&& ...
   all(abs(fn(:)'.*v.gap)<=o.productTolerance);
  if C==0,ok=norm(v.shape.tipMoment,inf)<=o.momentTolerance&&max([0;v.h])<=o.gapTolerance;end
 end
end
