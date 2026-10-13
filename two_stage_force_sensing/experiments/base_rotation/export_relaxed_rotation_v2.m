function export_relaxed_rotation_v2
% Export every frame/trace and audit likelihood/constraint derivatives.
[root,~]=setup_tsfs;allAudit=[];allConstraint=[];
for tag={'375deg','1deg'}
 runId=['base_rotation_relaxed_v2_' tag{1} '_20261007'];setenv('TSFS_RUN_ID',runId);[~,paths]=setup_tsfs;folder=fullfile(paths.results,'rotation');pub=fullfile(paths.published,runId);if ~isfolder(pub),mkdir(pub);end
 data=load(fullfile(folder,'truth_mu_300.mat'));frames=tsfs.read_input(data.input);
 for solver={'ipopt','fmincon'}
  name=solver{1};a=load(fullfile(folder,[name '.mat']));cfg=a.cfg;copyfile(fullfile(folder,[name '_frames.csv']),pub);traces=[];blocks=[];
  for k=1:numel(a.runs)
   fr=frames{k};fr.CU=eye(numel(fr.u))*cfg.sensorStd^2;fr.previous.CU=fr.CU;g=a.runs{k}.geometry;s=a.runs{k}.mechanics;e=s.evaluation;N=numel(fr.u);assert(numel(e.r)==N,'Force priors unexpectedly present');
   blocks=[blocks;struct('frame',k,'sensorObjective',0.5*sum(e.r.^2),'tipPriorObjective',0,'contactPriorObjective',0,'auxiliaryBetaMm',s.auxiliaryBeta,'seedDistance',norm(s.z-s.initialZ),'stationarity',s.stationarity)];
   if strcmp(name,'ipopt')
    t=s.trace;for j=1:numel(t.obj),traces=[traces;struct('frame',k,'iteration',j-1,'objective',t.obj(j)/s.objectiveScale,'primalViolation',t.inf_pr(j),'stationarity',t.inf_du(j))];end
   else
    t=s.trace;for j=1:numel(t),traces=[traces;struct('frame',k,'iteration',t(j).iteration,'objective',t(j).objective,'primalViolation',t(j).constraintViolation,'stationarity',t(j).firstorderopt)];end
   end
   history=tsfs.reconstruct(fr.previous,false);
   if k>1,history=a.runs{k-1}.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];end
   for phase={'initial','final'}
    if strcmp(phase{1},'initial')
     e=tsfs.evaluate(fr,g,s.initialZ,s.scale,s.prior,history,a.o,true);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=0.5*(e.r'*e.r);beta=vecnorm(e.slip)'+1e-8;
    else,e=s.evaluation;beta=s.auxiliaryBeta;end
    [v,A]=tsfs.relaxed_constraints(e,g,s.scale,beta,s.epsilon,true);J=e.J;G=s.objectiveScale*(J'*J);grad=s.objectiveScale*(J'*e.r);sv=svd(J,0);
    allAudit=[allAudit;struct('step',tag{1},'solver',name,'frame',k,'angleDeg',data.truth.angleDeg(k),'phase',phase{1}, ...
     'fbgObjective',e.objective,'solverObjective',s.objectiveScale*e.objective,'curvatureRmsPerMm',sqrt(mean((e.shape.prediction(:)-fr.u(:)).^2)), ...
     'scaledGradientInf',norm(grad,inf),'GNdiagMin',min(diag(G)),'GNdiagMax',max(diag(G)),'whitenedJcondition',sv(1)/sv(end), ...
     'constraintRows',numel(v),'constraintViolation',max([abs(v(1:3));v(4:end);0]),'normalForceN',e.shape.x(7),'slipMm',norm(e.slip(:,1)),'betaMm',beta(1))];
    C=numel(g.s);assert(C==1,'Audit labels describe this single-contact demo');ng=numel(e.h)-C;
    inds={1:3,3+(1:ng),3+ng+1,3+ng+2,3+ng+3,3+ng+4};labels={'moment','sampledGap','normalProduct','forceConeSquared','slipConeSquared','frictionDualityGap'};
    if strcmp(name,'ipopt'),mult=s.multipliers;else,mult=[s.multipliers.eqnonlin;s.multipliers.ineqnonlin];end
    for j=1:numel(inds)
     q=inds{j};rn=vecnorm(A(q,:),2,2);lambda=NaN;if strcmp(phase{1},'final'),lambda=max(abs(mult(q)));end
     allConstraint=[allConstraint;struct('step',tag{1},'solver',name,'frame',k,'angleDeg',data.truth.angleDeg(k),'phase',phase{1},'block',labels{j}, ...
      'residualMin',min(v(q)),'residualMax',max(v(q)),'JacobianRowNormMin',min(rn),'JacobianRowNormMax',max(rn),'multiplierAbsMax',lambda)];
    end
   end
  end
  writetable(struct2table(traces,'AsArray',true),fullfile(pub,[name '_iterations.csv']));writetable(struct2table(blocks,'AsArray',true),fullfile(pub,[name '_objective_blocks.csv']));
 end
 meta=struct('configuration',cfg,'matlabVersion',version,'epsilon',1e-6,'objectiveScale',1e-4,'maxIterations',300,'optimalityTolerance',1e-8,'stepTolerance',1e-12, ...
 'gapDiagnosticMm',1e-6,'terminalMomentDiagnosticNmm',2e-4,'squaredConeTolerance',1e-6,'productAndDualityUpperNmm',2e-6, ...
 'exactLawWarnings','Original diagnostic thresholds retained in exactLawFlags; flags use revised relaxed-feasibility criteria.','samplingSeconds',0.02,'injectedNoiseStd',0);
 atomic_write_artifact(fullfile(pub,'settings.json'),'json',meta);
end
pub=fullfile(root,'results','published','base_rotation_relaxed_v2_comparison_20261007');if ~isfolder(pub),mkdir(pub);end
writetable(struct2table(allAudit),fullfile(pub,'objective_scales.csv'));writetable(struct2table(allConstraint),fullfile(pub,'constraint_scales.csv'));
fprintf('Exported all revised frames, zero force-prior objective blocks, and %d state audits.\n',numel(allAudit));
end
