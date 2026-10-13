function audit_relaxed_scaling
% Read-only reevaluation of saved solver seeds and terminal states. No solves.
[root,paths]=setup_tsfs;assert(strcmp(getenv('TSFS_RUN_ID'),'base_rotation_relaxed_scaling_20261007'));rows=[];blocks=[];vars=[];
for tag={'375deg','1deg'}
 source=fullfile(root,'results','runs',['base_rotation_relaxed_' tag{1} '_20261007'],'rotation');data=load(fullfile(source,'truth_mu_300.mat'));frames=tsfs.read_input(data.input);
 for solver={'ipopt','fmincon'}
  a=load(fullfile(source,[solver{1} '.mat']));
  for k=1:numel(frames)
   fr=frames{k};out=a.runs{k};s=out.mechanics;g=out.geometry;nx=numel(s.z);C=numel(g.s);N=numel(fr.u);
   history=tsfs.reconstruct(fr.previous,false);
   if k>1,history=a.runs{k-1}.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];end
   for phase={'initial','final'}
    if strcmp(phase{1},'initial')
     e=tsfs.evaluate(fr,g,s.initialZ,s.scale,s.prior,history,a.o,true);beta=vecnorm(e.slip)'+1e-8;z=s.initialZ;
    else,e=s.evaluation;beta=s.auxiliaryBeta;z=s.z;end
    [v,A]=tsfs.relaxed_constraints(e,g,s.scale,beta,s.epsilon);alpha=s.objectiveScale;J=e.J;G=alpha*(J'*J);grad=alpha*(J'*e.r);sv=svd(J,0);
    row=struct('step',tag{1},'solver',solver{1},'frame',k,'angleDeg',data.truth.angleDeg(k),'phase',phase{1},'nObservations',N, ...
     'sigmaCurvaturePerMm',sqrt(fr.CU(1,1)),'curvatureResidualRmsPerMm',sqrt(mean((e.shape.prediction(:)-fr.u(:)).^2)), ...
     'fbgObjective',0.5*sum(e.r(1:N).^2),'tipPriorObjective',0.5*sum(e.r(N+(1:3)).^2),'contactPriorObjective',0.5*sum(e.r(N+4:end).^2), ...
     'unscaledObjective',e.objective,'solverObjective',alpha*e.objective,'scaledGradientInf',norm(grad,inf), ...
     'scaledGNdiagMin',min(diag(G)),'scaledGNdiagMax',max(diag(G)),'whitenedJcondition',sv(1)/sv(end), ...
     'betaMm',beta(1),'slipMm',norm(e.slip(:,1)),'normalForceN',e.shape.x(7),'frictionN',norm(e.shape.x(8:9)), ...
     'normalGapMm',e.gap(1),'stage1ResidualNorm',norm(g.residual),'tipPriorStdN',s.prior.tipStd,'contactPriorStdN',s.prior.forceStd, ...
     'constraintRows',numel(v),'constraintViolation',max([abs(v(1:3));v(4:end);0]));
    rows=[rows;row];
    ng=numel(e.h)-C;inds={1:3,3+(1:ng),3+ng+1,3+ng+2,3+ng+3,3+ng+4,3+ng+5};labels={'moment','sampledGap','contactGap','normalProduct','forceConeSquared','slipConeSquared','frictionDualityGap'};
    assert(C==1,'Audit labels currently select the single-contact experiment.');
    if strcmp(solver{1},'ipopt'),multipliers=s.multipliers;else,multipliers=[s.multipliers.eqnonlin;s.multipliers.ineqnonlin];end
    for j=1:numel(inds)
     q=inds{j};norms=vecnorm(A(q,:),2,2);lam=NaN;if strcmp(phase{1},'final'),lam=max(abs(multipliers(q)));end
     blocks=[blocks;struct('step',tag{1},'solver',solver{1},'frame',k,'angleDeg',data.truth.angleDeg(k),'phase',phase{1},'block',labels{j}, ...
      'residualMin',min(v(q)),'residualMax',max(v(q)),'residualAbsMax',max(abs(v(q))),'JacobianRowNormMin',min(norms),'JacobianRowNormMax',max(norms), ...
      'multiplierAbsMax',lam)];
    end
    for j=1:nx
     vars=[vars;struct('step',tag{1},'solver',solver{1},'frame',k,'phase',phase{1},'variable',j,'physicalScale',s.scale(j),'z',z(j),'gradient',grad(j),'GNdiagonal',G(j,j),'constraintColumnNorm',norm(A(:,j)))];
    end
   end
  end
 end
end
pub=fullfile(paths.published,getenv('TSFS_RUN_ID'));if ~isfolder(pub),mkdir(pub);end
writetable(struct2table(rows),fullfile(pub,'objective_scales.csv'));writetable(struct2table(blocks),fullfile(pub,'constraint_scales.csv'));writetable(struct2table(vars),fullfile(pub,'variable_scales.csv'));
save(fullfile(paths.results,'scaling.mat'),'rows','blocks','vars','-v7.3');fprintf('Audited %d initial/final states; no optimization rerun.\n',numel(rows));
end
