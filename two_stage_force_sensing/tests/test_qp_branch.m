function report=test_qp_branch
% tsfs.qp_branch against tsfs.mode_branch on the same rod linearization.
% Checks: interface compatibility, objective no worse than fmincon for every
% feasible fixed-mode branch, default objective = data objective, and that
% the optional prior rows equal the existing rows of tsfs.evaluate.
[root,paths]=setup_tsfs;o=tsfs.defaults;checks={};
a=load(fullfile(root,'datasets','sliding_clean.mat'));frames=tsfs.read_input(a.sensorInput);
bvp=struct('relativeTolerance',1e-7,'absoluteTolerance',1e-9,'maxMeshNodes',4000,'meshStepMm',5);
cfg=struct('constraintTolerance',1e-6,'maxIterationsPerAssignment',300,'fbgRmsAcceptance',Inf,'singlePassMethod','profiled-qp','linearization',struct('initialRadius',Inf),'forcePriors',false,'bvp',bvp);
previous=[];worst=-Inf;branches=0;
for k=1:3
 fr=frames{k};g=tsfs.geometry(fr,o);[z0,scale,prior]=tsfs.initialize(fr,g,previous,o);model=tsfs.linearized_model(fr,g,z0,scale,prior,previous,o);
 inner=cfg;inner.mechanicalModel=model;inner.trustRadius=Inf;
 for mode=0:2
  modes=mode*ones(1,numel(g.s));
  s=tsfs.mode_branch(fr,g,previous,o,inner,modes,z0,scale,prior);q=tsfs.qp_branch(fr,g,previous,o,inner,modes,z0,scale,prior);
  assert(all(isfield(q,fieldnames(s))),'qp_branch must return every mode_branch field.');
  assert(isequal(size(q.modeY),size(s.modeY)),'modeY layout differs.');
  assert(abs(q.selectionObjective-q.evaluation.objective)<=1e-9*max(1,q.evaluation.objective),'Default selection objective must equal the data objective.');
  if s.modeFeasible
   assert(q.modeFeasible,'QP infeasible where fmincon is feasible (frame %d mode %d).',k,mode);
   worst=max(worst,(q.evaluation.objective-s.evaluation.objective)/max(1,s.evaluation.objective));branches=branches+1;
  end
 end
 sol=tsfs.single_pass_bvp_solve(fr,g,previous,o,cfg);
 previous=struct('valid',true,'time',fr.time,'geometry',g,'mechanics',sol,'diagnostics',[],'contactForce',sol.evaluation.shape.force,'tipForce',sol.evaluation.shape.x(4:6),'seconds',0);
end
assert(branches>0&&worst<1e-6,'QP branch objective above fmincon by %.3g (relative).',worst);
checks{end+1}='interface, feasibility and objective no worse than fmincon';checks{end+1}='default objective equals data objective';
% Optional prior rows must reproduce the existing force-tracking rows of tsfs.evaluate.
fr=frames{4};g=tsfs.geometry(fr,o);[z0,scale,prior]=tsfs.initialize(fr,g,previous,o);model=tsfs.linearized_model(fr,g,z0,scale,prior,previous,o);
inner=cfg;inner.mechanicalModel=model;inner.trustRadius=Inf;inner.forcePriors=true;
q=tsfs.qp_branch(fr,g,previous,o,inner,2*ones(1,numel(g.s)),z0,scale,prior);
e=tsfs.evaluate(fr,g,q.z,scale,prior,model.history,o,false);N=numel(fr.u);priorObjective=.5*sum(e.r(N+1:end).^2);
priorError=abs(q.objectiveParts(2)-priorObjective)/max(1,priorObjective);assert(priorError<1e-8,'Prior rows differ from tsfs.evaluate.');
checks{end+1}='prior rows equal tsfs.evaluate force-tracking rows';
report=struct('passed',true,'checks',{checks},'worstRelativeObjectiveExcess',worst,'comparedBranches',branches,'priorRowError',priorError);
fid=fopen(fullfile(paths.results,'test_qp_branch.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end
