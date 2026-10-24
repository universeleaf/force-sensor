function cfg=robust_configurations(name,sensorStd)
% Named configurations for the paired comparison. 'reference' reproduces
% run_fine_bvp_experiment exactly (fmincon SQP per branch). Every 'qp...'
% name uses the same framework with tsfs.qp_branch as branch optimizer and
% adds switches given as '_'-separated tokens:
%   trust      bvpTrust: unconverged/fit-degrading BVP state not propagated
%   soft       slipStd='stage1': slip history weighted by Stage 1 covariance
%   tip<v>     forcePriors=true with o.tipProcessStd=v (framework's existing prior)
%   f<v>       o.forceProcessStd=v (contact force prior std per frame)
%   r<v>       forcePriorTangentRatio=v (tangential std = v*forceStd; Inf = normal only)
%   fast       o.batchedReconstruction=true (same Stage 1 differences, one pass)
% Any prior or soft-history token also enables trust.
% Examples: 'qp', 'qp_trust', 'qp_soft', 'qp_soft_tip0.1_f1'.
cfg=struct('constraintTolerance',1e-6,'sensorStd',sensorStd,'forcePriors',false,'solver','enumerative-MINLP/fmincon-SQP','exactCircularFriction',true, ...
 'allAssignments',true,'globalContinuousCertificate',false,'maxIterationsPerAssignment',100,'maxEvaluationsPerAssignment',5000,'optimalityTolerance',1e-8,'stepTolerance',1e-12,'objectiveScale',1e-4);
s=linearized_mode_settings;f=fieldnames(s);for k=1:numel(f),cfg.(f{k})=s.(f{k});end
cfg.modeSearch='exhaustive';cfg.fbgRmsAcceptance=1.24;cfg.estimatorMus=.3;cfg.solverFunction='tsfs.single_pass_bvp_solve';cfg.linearization.initialRadius=Inf;
cfg.singlePassMethod='full-cone-sqp';cfg.bvp=struct('relativeTolerance',1e-7,'absoluteTolerance',1e-9,'maxMeshNodes',4000,'meshStepMm',5,'loadPolicy','all forces fixed, base moment free; contact diagnostics only');
cfg.name=name;cfg.optionOverrides=struct;
if strcmp(name,'reference'),return,end
assert(strncmp(name,'qp',2),'Unknown configuration %s',name);
cfg.singlePassMethod='profiled-qp';cfg.solver='enumerative/profiled-QP';
tokens=strsplit(name,'_');trust=false;
for k=2:numel(tokens)
 t=tokens{k};v=str2double(regexprep(t,'^[a-z]+',''));
 if strcmp(t,'trust'),trust=true;
 elseif strcmp(t,'fast'),cfg.optionOverrides.batchedReconstruction=true; % numerically equivalent Stage 1 sensitivities
 elseif strcmp(t,'soft'),cfg.slipStd='stage1';trust=true;
 elseif startsWith(t,'tip'),cfg.forcePriors=true;cfg.optionOverrides.tipProcessStd=v;trust=true;
 elseif startsWith(t,'f'),cfg.optionOverrides.forceProcessStd=v;
 elseif startsWith(t,'r'),cfg.forcePriorTangentRatio=v;
 else,error('Unknown token %s in %s',t,name);end
end
if trust,cfg.bvpTrust=struct('policy','reject-unconverged-or-fit-degrading','fbgRatio',1.5,'fbgSlack',0.25);end
end
