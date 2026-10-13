function sol=linearized_mode_branch(fr,g,previous,o,cfg,modes,z0,scale,prior)
% Successive rod linearization; exact circular friction in every inner solve.
% All final states/diagnostics are nonlinear IVP results, including failures.
clock=tic;model=cfg.sharedModel;settings=cfg.linearization;radius=settings.initialRadius;
counts=struct('augmentedIVPs',0,'rhsEvaluations',0,'odeSegments',0,'failedIVPs',0,'affineEvaluations',0);
iterations=0;outerTrace=[];best=[];currentY=[];converged=false;reason='outer-iteration-limit';
innerCfg=rmfield(cfg,'sharedModel');innerCfg=rmfield(innerCfg,'linearization');
for k=1:settings.maxOuterIterations
 innerCfg.mechanicalModel=model;innerCfg.trustRadius=radius;
 inner=tsfs.mode_branch(fr,g,previous,o,innerCfg,modes,model.z,scale,prior);
 iterations=iterations+inner.iterations;counts.affineEvaluations=counts.affineEvaluations+inner.counts.affineEvaluations;
 if isempty(currentY),currentY=inner.initialY;end
 [~,baseL1]=tsfs.mode_residual(model.e,g,modes,currentY,scale);
 [~,predL1]=tsfs.mode_residual(inner.evaluation,g,modes,inner.modeY,scale);
 penalty=1e-3;
 if isstruct(inner.multipliers)
  fields=fieldnames(inner.multipliers);for f=1:numel(fields),penalty=max(penalty,1.5*max([0;abs(inner.multipliers.(fields{f})(:))]));end
 end
 step=norm(inner.z-model.z,inf);predicted=1e-4*(model.e.objective-inner.evaluation.objective)+penalty*(baseL1-predL1);
 failed=false;exception='';trueObjective=NaN;trueFbg=NaN;counts.augmentedIVPs=counts.augmentedIVPs+1;
 try
  actual=tsfs.evaluate(fr,g,inner.z,scale,prior,model.history,o,true);N=numel(fr.u);actual.r=actual.r(1:N);actual.J=actual.J(1:N,:);actual.objective=.5*(actual.r'*actual.r);
  trueObjective=actual.objective;trueFbg=actual.fbgRms;counts.rhsEvaluations=counts.rhsEvaluations+actual.shape.rhsEvaluations;counts.odeSegments=counts.odeSegments+actual.shape.segments;
  [violation,actualL1]=tsfs.mode_residual(actual,g,modes,inner.modeY,scale);
  actualReduction=1e-4*(model.e.objective-actual.objective)+penalty*(baseL1-actualL1);
  ratio=actualReduction/max(predicted,1e-16);modelError=sqrt(mean((actual.r-inner.evaluation.r).^2));
 catch err
  failed=true;exception=[err.identifier ': ' err.message];counts.failedIVPs=counts.failedIVPs+1;
  violation=Inf;actualReduction=-Inf;ratio=-Inf;modelError=Inf;
 end
 accepted=~failed&&step<=radius+1e-8&&((predicted>1e-14&&ratio>=.1)||(violation<=cfg.constraintTolerance&&step<=settings.stepTolerance));
 if ~failed
  candidate=inner;candidate.evaluation=actual;candidate.modeViolation=violation;candidate.modeFeasible=violation<=cfg.constraintTolerance;
  if isempty(best)||(candidate.modeFeasible&&~best.modeFeasible)||(candidate.modeFeasible&&best.modeFeasible&&actual.objective<best.evaluation.objective)||(~candidate.modeFeasible&&~best.modeFeasible&&violation<best.modeViolation),best=candidate;end
 end
 outerTrace=[outerTrace;struct('trueObjective',trueObjective,'trueFbgRms',trueFbg,'predictedObjective',inner.evaluation.objective,'z',inner.z,'iteration',k,'radius',radius,'step',step,'innerIterations',inner.iterations,'innerExitflag',inner.exitflag,'innerViolation',inner.modeViolation,'trueViolation',violation,'predictedReduction',predicted,'actualReduction',actualReduction,'ratio',ratio,'penalty',penalty,'modelFbgRmsError',modelError,'accepted',accepted,'seconds',toc(clock),'exception',exception)];
 if ~failed&&inner.converged&&violation<=cfg.constraintTolerance&&step<=settings.stepTolerance
  best=candidate;converged=true;reason='linearized-step-converged';break
 end
 if accepted
  model.z=inner.z;model.e=actual;currentY=inner.modeY;
  if ratio>.75&&step>.8*radius,radius=min(settings.maxRadius,2*radius);end
 else
  if inner.modeViolation>cfg.constraintTolerance&&step>=.8*radius&&radius<settings.maxRadius
   radius=min(settings.maxRadius,2*radius);
  else,radius=radius/2;end
  if radius<settings.minRadius,reason='linearized-trust-radius-limit';break,end
 end
end
if isempty(best)
 % Keep the initial nonlinear state and expose failure; never export affine p.
 best=inner;best.z=model.z;best.evaluation=model.e;[best.modeViolation,~]=tsfs.mode_residual(model.e,g,modes,inner.initialY,scale);best.modeFeasible=best.modeViolation<=cfg.constraintTolerance;
end
sol=best;sol.converged=converged;sol.reason=reason;sol.exitflag=double(converged);sol.iterations=iterations;sol.counts=counts;sol.seconds=toc(clock);sol.trace=outerTrace;sol.outerIterations=numel(outerTrace);sol.outerTrace=outerTrace;sol.solver='successive-rod-linearization/full-circular-friction/fmincon-SQP';
sol.configuration=rmfield(cfg,'sharedModel');sol.initialZ=z0;
end
