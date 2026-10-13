function sol=mode_solve(fr,g,previous,o,cfg)
% Integer contact modes, previous assignment first; local continuous solves.
% Default exhaustive search. Optional first-acceptable policy uses native
% convergence, branch feasibility, and whitened FBG RMS (not objective scale).
clock=tic;conditional=isfield(cfg,'modeSearch')&&strcmp(cfg.modeSearch,'first-acceptable');
fbgLimit=o.fbgRmsWarning;if isfield(cfg,'fbgRmsAcceptance'),fbgLimit=cfg.fbgRmsAcceptance;end
assert(isscalar(fbgLimit)&&isfinite(fbgLimit)&&fbgLimit>=0,'Invalid FBG RMS acceptance threshold');
accepted=0;[z0,scale,prior]=tsfs.initialize(fr,g,previous,o);linearized=isfield(cfg,'mechanicsApproximation')&&strcmp(cfg.mechanicsApproximation,'successive-linearization');sharedSeconds=0;
if linearized,t0=tic;cfg.sharedModel=tsfs.linearized_model(fr,g,z0,scale,prior,previous,o);sharedSeconds=toc(t0);end
C=numel(g.s);choices=cell(1,C);radix=zeros(1,C);preferred=zeros(1,C);
for i=1:C
 if g.mu(i)==0,choices{i}=[0 3];else,choices{i}=[0 1 2];end;radix(i)=numel(choices{i});
 if ~isempty(previous)&&isfield(previous.mechanics,'mode')&&prior.matched(i)>0,preferred(i)=previous.mechanics.mode(prior.matched(i));end
 if ~ismember(preferred(i),choices{i}),preferred(i)=0;end
end
assignments=zeros(prod(radix),C);if C==0,assignments=zeros(1,0);end
for k=1:size(assignments,1),v=k-1;for i=1:C,assignments(k,i)=choices{i}(mod(v,radix(i))+1);v=floor(v/radix(i));end,end
first=find(all(assignments==preferred,2),1);order=[first setdiff(1:size(assignments,1),first,'stable')];attempts=cell(1,numel(order));summary=[];
for j=1:numel(order)
 modes=assignments(order(j),:);
 if linearized
  s=tsfs.linearized_mode_branch(fr,g,previous,o,cfg,modes,z0,scale,prior);
  if j==1,s.seconds=s.seconds+sharedSeconds;s.counts.augmentedIVPs=s.counts.augmentedIVPs+1;s.counts.rhsEvaluations=s.counts.rhsEvaluations+cfg.sharedModel.e.shape.rhsEvaluations;s.counts.odeSegments=s.counts.odeSegments+cfg.sharedModel.e.shape.segments;end
 else,s=tsfs.mode_branch(fr,g,previous,o,cfg,modes,z0,scale,prior);end
 attempts{j}=s;
 summary=[summary;struct('attempt',j,'modes',mat2str(modes),'reason',s.reason,'nativePositive',s.converged,'feasible',s.modeFeasible,'violation',s.modeViolation,'objective',s.evaluation.objective,'seconds',s.seconds,'iterations',s.iterations,'ivps',s.counts.augmentedIVPs,'failedIVPs',s.counts.failedIVPs,'initialViolation',s.initialViolation,'stationarity',s.stationarity,'fbgRms',s.evaluation.fbgRms,'acceptable',s.converged&&s.modeFeasible&&isfinite(s.evaluation.fbgRms)&&s.evaluation.fbgRms<=fbgLimit)];
 if conditional&&summary(end).acceptable,accepted=j;break,end
end
attempts=attempts(1:numel(summary));
feasible=find([summary.feasible]);if accepted>0,chosen=accepted;elseif ~isempty(feasible),[~,ii]=min([summary(feasible).objective]);chosen=feasible(ii);else,[~,chosen]=min([summary.violation]);end
sol=attempts{chosen};sol.selectedIterations=sol.iterations;sol.selectedCounts=sol.counts;sol.selectedSeconds=sol.seconds;sol.selectedAttempt=chosen;sol.previousMode=preferred;sol.attemptSummary=summary;sol.attempts=attempts;
sol.iterations=sum([summary.iterations]);sol.counts.augmentedIVPs=sum([summary.ivps]);sol.counts.failedIVPs=sum([summary.failedIVPs]);sol.counts.rhsEvaluations=sum(cellfun(@(x)x.counts.rhsEvaluations,attempts));sol.counts.odeSegments=sum(cellfun(@(x)x.counts.odeSegments,attempts));sol.seconds=toc(clock);sol.integerAssignmentsTested=numel(summary);sol.integerAssignmentsPossible=numel(order);sol.globalContinuousCertificate=false;
sol.modeAccepted=summary(chosen).acceptable;sol.fbgRmsAcceptance=fbgLimit;sol.conditionalSearch=conditional;sol.stoppedEarly=accepted>0&&numel(summary)<numel(order);
if conditional,sol.solver='conditional-modes/fmincon-SQP';end
if linearized,sol.solver='conditional-modes/successive-rod-linearization';sol.outerIterations=sum(cellfun(@(x)x.outerIterations,attempts));sol.counts.affineEvaluations=sum(cellfun(@(x)x.counts.affineEvaluations,attempts));end
end
