function sol=single_linearization_solve(fr,g,previous,o,cfg)
% One rod linearization per physical frame. Mode selection uses affine physics.
% Nonlinear IVP is a diagnostic only; it never refines or reselects the solution.
clock=tic;[z0,scale,prior]=tsfs.initialize(fr,g,previous,o);t0=tic;model=tsfs.linearized_model(fr,g,z0,scale,prior,previous,o);linearizeSeconds=toc(t0);
C=numel(g.s);polygon=strcmp(cfg.singlePassMethod,'polygon-qp');if polygon,nm=18;else,nm=3;end
assignments=zeros(nm^C,C);for k=1:size(assignments,1),v=k-1;for i=1:C,assignments(k,i)=mod(v,nm);v=floor(v/nm);end,end
preferred=zeros(1,C);for i=1:C,if ~isempty(previous)&&prior.matched(i)>0&&isfield(previous.mechanics,'enumeratedMode'),preferred(i)=previous.mechanics.enumeratedMode(prior.matched(i));elseif ~isempty(previous)&&prior.matched(i)>0,preferred(i)=previous.mechanics.mode(prior.matched(i));end,end
first=find(all(assignments==preferred,2),1);order=[first setdiff(1:size(assignments,1),first,'stable')];attempts={};summary=[];
innerCfg=cfg;innerCfg.mechanicalModel=model;innerCfg.trustRadius=cfg.linearization.initialRadius;
for j=1:numel(order)
 modes=assignments(order(j),:);
 if polygon,s=tsfs.polygon_qp_branch(fr,g,o,innerCfg,modes,z0,scale,prior);else,s=tsfs.mode_branch(fr,g,previous,o,innerCfg,modes,z0,scale,prior);end
 s.enumeratedMode=modes;s.affineViolation=s.modeViolation;s.affineFeasible=s.modeFeasible;s.affineFbgRms=s.evaluation.fbgRms;s.affineEvaluation=s.evaluation;
 s.affineEvaluation.shape.p=interp1(s.evaluation.shape.collisionS,s.evaluation.shape.collisionP',s.evaluation.shape.s,'linear')';
 s.trustStep=norm(s.z-z0,inf);s.atTrustBoundary=s.trustStep>=.99*innerCfg.trustRadius;
 attempts{j}=s;summary=[summary;struct('attempt',j,'modes',mat2str(modes),'reason',s.reason,'nativePositive',s.converged,'feasible',s.modeFeasible,'violation',s.modeViolation,'objective',s.evaluation.objective,'seconds',s.seconds,'iterations',s.iterations,'ivps',0,'failedIVPs',0,'initialViolation',s.initialViolation,'stationarity',s.stationarity,'fbgRms',s.evaluation.fbgRms,'acceptable',s.converged&&s.modeFeasible&&s.evaluation.fbgRms<=cfg.fbgRmsAcceptance)];
end
feasible=find([summary.feasible]);if ~isempty(feasible),[~,j]=min([summary(feasible).objective]);chosen=feasible(j);else,[~,chosen]=min([summary.violation]);end
sol=attempts{chosen};sol.affineConverged=sol.converged;sol.affineObjective=sol.evaluation.objective;t0=tic;exception='';
try
 e=tsfs.evaluate(fr,g,sol.z,scale,prior,model.history,o,true);N=numel(fr.u);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=.5*(e.r'*e.r);sol.evaluation=e;
 if polygon
  [v,cv]=tsfs.polygon_qp_violation(e,g,sol.enumeratedMode,sol.qpY,scale);sol.polygonTrueViolation=v;sol.circularTrueViolation=cv;sol.modeViolation=v;
 else,[sol.modeViolation,~]=tsfs.mode_residual(e,g,sol.mode,sol.modeY,scale);sol.circularTrueViolation=sol.modeViolation;end
 sol.nonlinearAuditFailed=false;sol.modeFeasible=sol.modeViolation<=cfg.constraintTolerance;rhs=e.shape.rhsEvaluations;segments=e.shape.segments;
catch err
 % Retain an explicitly marked affine shape if nonlinear validation fails.
 sol.evaluation=sol.affineEvaluation;sol.modeFeasible=false;sol.modeViolation=Inf;sol.circularTrueViolation=Inf;sol.nonlinearAuditFailed=true;exception=[err.identifier ': ' err.message];rhs=0;segments=0;
end
sol.auditSeconds=toc(t0);sol.auditException=exception;sol.linearizeSeconds=linearizeSeconds;sol.selectedIterations=sol.iterations;sol.selectedSeconds=sol.seconds;sol.selectedCounts=sol.counts;
sol.selectedAttempt=chosen;sol.previousMode=preferred;sol.attemptSummary=summary;sol.attempts=attempts;sol.iterations=sum([summary.iterations]);sol.seconds=toc(clock);sol.counts=struct('augmentedIVPs',2,'rhsEvaluations',model.e.shape.rhsEvaluations+rhs,'odeSegments',model.e.shape.segments+segments,'failedIVPs',double(sol.nonlinearAuditFailed),'affineEvaluations',sum(cellfun(@(a)a.counts.affineEvaluations,attempts)));
sol.integerAssignmentsTested=numel(order);sol.integerAssignmentsPossible=numel(order);sol.globalContinuousCertificate=false;sol.modeAccepted=sol.affineConverged&&sol.modeFeasible&&sol.evaluation.fbgRms<=cfg.fbgRmsAcceptance;sol.conditionalSearch=false;sol.stoppedEarly=false;sol.fbgRmsAcceptance=cfg.fbgRmsAcceptance;sol.outerIterations=0;
sol.configuration=cfg;sol.reason=['one-linearization/' sol.reason];sol.solver=['one-linearization/' cfg.singlePassMethod];sol.converged=sol.affineConverged;
end
