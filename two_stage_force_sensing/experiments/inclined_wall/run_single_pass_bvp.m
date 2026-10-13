function run_single_pass_bvp(stamp,targets,maxFrames)
% Separate corrected trajectories; reuse exactly the prior benchmark packets.
if nargin<1,stamp='20261012_fixedload_v2';end;if nargin<2,targets=[0 .5 1 2];end;if nargin<3,maxFrames=Inf;end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));inputFolder=fullfile(root,'results','runs','single_pass_inputs_20261012_unbounded');
for target=targets
 level=strrep(num2str(target),'.','p');a=load(fullfile(inputFolder,['insertion_' level 'mm.mat']),'sensorStd');cfg=linearized_mode_settings;cfg.modeSearch='exhaustive';cfg.allAssignments=true;cfg.fbgRmsAcceptance=1.24;cfg.sensorStd=a.sensorStd;cfg.exactCircularFriction=true;cfg.estimatorMus=.3;cfg.reportFrameStart=true;cfg.targetNoiseTipRmsMm=target;cfg.noiseSeed=33001;cfg.maxIterationsPerAssignment=100;
 cfg.solverFunction='tsfs.single_pass_bvp_solve';cfg.linearization.initialRadius=Inf;cfg.singlePassMethod='full-cone-sqp';cfg.selectionPolicy='lowest affine-feasible objective, then fixed-load BVP; no reselection';cfg.bvp=struct('relativeTolerance',1e-7,'absoluteTolerance',1e-9,'maxMeshNodes',4000,'meshStepMm',5,'loadPolicy','all tip/contact forces fixed; base moment free; wall constraints diagnostic only');
 for scenario={'insertion','rotation'}
  name=scenario{1};if strcmp(name,'rotation')&&maxFrames<21,continue,end
  tag=sprintf('singlepass_bvp_%s_%smm_%s',name,level,stamp);path=fullfile(root,'results','runs',tag,'comparison.mat');T=41;if strcmp(name,'rotation'),T=25;end;T=min(T,maxFrames);
  if isfile(path),saved=load(path,'runs');if size(saved.runs,2)==T&&all(~cellfun(@isempty,saved.runs),'all'),fprintf('SKIP COMPLETE %s\n',tag);continue,end;error('Partial result exists; use fresh stamp.');end
  replay=struct('source',fullfile(inputFolder,[name '_' level 'mm.mat']));if strcmp(name,'rotation'),saved=load(fullfile(root,'results','runs',sprintf('singlepass_bvp_insertion_%smm_%s',level,stamp),'comparison.mat'),'runs');replay.initialEstimates=saved.runs(:,21)';end
  fprintf('START FIXED LOAD BVP %s\n',tag);run_inclined_modes(maxFrames,tag,cfg,replay);fprintf('DONE FIXED LOAD BVP %s\n',tag);
 end
end
end
