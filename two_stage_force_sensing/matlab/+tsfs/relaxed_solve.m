function sol=relaxed_solve(fr,g,previous,o,solver,cfg)
% Direct epsilon=1e-6 solve. Same objective, derivatives, and constraints.
if nargin<6,cfg=struct;end
if ~isfield(cfg,'forcePriors'),cfg.forcePriors=true;end
if ~isfield(cfg,'constraintTolerance'),cfg.constraintTolerance=1e-9;end
if ~isfield(cfg,'deduplicateGap'),cfg.deduplicateGap=false;end
clock=tic;epsilon=1e-6;objectiveScale=1e-4;[z,scale,prior]=tsfs.initialize(fr,g,previous,o);nx=numel(z);C=numel(g.s);
history=[];historySource=fr.historySource;
if ~isempty(fr.previous)
 if ~isempty(previous)&&previous.valid&&abs(previous.time-fr.previous.time)<1e-9
  history=previous.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];historySource='previous-mechanical-posterior';
 else,history=tsfs.reconstruct(fr.previous,false);end
end
counts=struct('stateIVPs',0,'augmentedIVPs',0,'rhsEvaluations',0,'odeSegments',0,'trialRejections',NaN,'qpIterations',NaN,'maxStateDimension',0);
cacheZ=[];cacheE=[];trace=[];initial=mechanics(z);beta=vecnorm(initial.slip)'+1e-8;y0=[z;beta];lb=-Inf(nx+C,1);lb(7:3:nx)=0;lb(nx+1:end)=0;ub=Inf(size(lb));
[v0,~]=allvalues(y0);nc=numel(v0)-1;setupClock=tic;
if strcmp(solver,'fmincon')
 opts=optimoptions('fmincon','Algorithm','sqp','Display','off','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
  'MaxIterations',300,'MaxFunctionEvaluations',5000,'OptimalityTolerance',1e-8,'ConstraintTolerance',cfg.constraintTolerance,'StepTolerance',1e-12,'ScaleProblem',false,'OutputFcn',@logstep);
 setupSeconds=toc(setupClock);solveClock=tic;
 [y,~,exitflag,output,multipliers]=fmincon(@objective,y0,[],[],[],[],lb,ub,@constraints,opts);solveSeconds=toc(solveClock);
 converged=exitflag>0;reason=sprintf('fmincon-exit-%d',exitflag);iterations=output.iterations;stationarity=output.firstorderopt;stats=output;
else
 cb=tsfs.RelaxedCallback('rod_nlp',@allvalues,numel(y0),numel(v0),false);sym=casadi.MX.sym('x',numel(y0));values=cb(sym);
 opts=struct('print_time',false,'error_on_fail',false);
 opts.ipopt=struct('print_level',0,'sb','yes','max_iter',300,'tol',1e-8,'constr_viol_tol',cfg.constraintTolerance,'acceptable_iter',0, ...
  'hessian_approximation','limited-memory','nlp_scaling_method','none','bound_relax_factor',0,'honor_original_bounds','yes');
 nlp=struct('x',sym,'f',values(1),'g',values(2:end));S=casadi.nlpsol('rotation_ipopt','ipopt',nlp,opts);setupSeconds=toc(setupClock);solveClock=tic;
 result=S('x0',y0,'lbx',lb,'ubx',ub,'lbg',[zeros(3,1);-Inf(nc-3,1)],'ubg',zeros(nc,1));solveSeconds=toc(solveClock);
 y=full(result.x);stats=S.stats();converged=stats.success;reason=stats.return_status;iterations=stats.iter_count;multipliers=full(result.lam_g);exitflag=double(converged);
 stationarity=NaN;if isfield(stats,'iterations')&&isfield(stats.iterations,'inf_du'),stationarity=stats.iterations.inf_du(end);end
 trace=stats.iterations;
end
e=mechanics(y(1:nx));[~,~,meta]=tsfs.relaxed_constraints(e,g,scale,y(nx+1:end),epsilon,cfg.deduplicateGap);
sol=struct('z',y(1:nx),'auxiliaryBeta',y(nx+1:end),'scale',scale,'prior',prior,'evaluation',e,'converged',converged,'reason',reason, ...
 'iterations',iterations,'trace',trace,'counts',counts,'seconds',toc(clock),'stationarity',stationarity,'objectiveScale',objectiveScale, ...
 'historySource',historySource,'epsilon',epsilon,'solver',solver,'solverStats',stats,'exitflag',exitflag,'multipliers',multipliers,'relaxed',meta, ...
 'configuration',cfg,'setupSeconds',setupSeconds,'solveSeconds',solveSeconds,'initialZ',z);
 function e=mechanics(zz)
  if isequal(zz,cacheZ),e=cacheE;return,end
  e=tsfs.evaluate(fr,g,zz,scale,prior,history,o,true);e.relaxedHasHistory=~isempty(history);
  if ~cfg.forcePriors,N=numel(fr.u);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=0.5*(e.r'*e.r);end
  cacheZ=zz;cacheE=e;
  counts.augmentedIVPs=counts.augmentedIVPs+1;counts.rhsEvaluations=counts.rhsEvaluations+e.shape.rhsEvaluations;
  counts.odeSegments=counts.odeSegments+e.shape.segments;counts.maxStateDimension=max(counts.maxStateDimension,e.shape.stateDimension);
 end
 function [v,J]=allvalues(y)
  e=mechanics(y(1:nx));[con,Dcon]=tsfs.relaxed_constraints(e,g,scale,y(nx+1:end),epsilon,cfg.deduplicateGap);
  v=[objectiveScale*e.objective;con];J=[objectiveScale*(e.r'*e.J) zeros(1,C);Dcon];
 end
 function [f,df]=objective(y),[v,J]=allvalues(y);f=v(1);df=J(1,:)';end
 function [h,c,Dh,Dc]=constraints(y),[v,J]=allvalues(y);c=v(2:4);h=v(5:end);Dc=J(2:4,:)';Dh=J(5:end,:)';end
 function stop=logstep(y,values,state)
  stop=false;if strcmp(state,'iter')||strcmp(state,'init')
   [v,~]=allvalues(y);trace=[trace;struct('iteration',values.iteration,'objective',v(1)/objectiveScale,'constraintViolation',max([abs(v(2:4));v(5:end);0]), ...
    'firstorderopt',values.firstorderopt,'funcCount',values.funccount,'seconds',toc(clock),'z',y')];
  end
 end
end
