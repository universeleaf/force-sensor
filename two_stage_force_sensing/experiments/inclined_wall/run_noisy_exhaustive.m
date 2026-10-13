function run_noisy_exhaustive(stamp)
% Exhaust every mode on the same noisy packets, with independent posterior history.
% No iteration/tolerance change; no truth in branch selection.
if nargin<1,stamp='20261011';end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));
sourceFolder=fullfile(root,'results','runs','noisy_linearized_20261011');noise=load(fullfile(sourceFolder,'noise_payload.mat'));
for target=[2 5]
 cfg=linearized_mode_settings;cfg.modeSearch='exhaustive';cfg.allAssignments=true;cfg.fbgRmsAcceptance=1.24;cfg.sensorStd=noise.(['sigma' num2str(target)]);cfg.estimatorMus=.3;cfg.reportFrameStart=true;cfg.targetNoiseTipRmsMm=target;cfg.noiseSeed=33001;cfg.muUncertainty=false;cfg.selectionPolicy='Existing lowest-objective branch-feasible candidate, including flagged nonconverged candidates';
 for scenario={'insertion','rotation'}
  name=scenario{1};tag=sprintf('inclined_%s_noisy%dmm_exhaustive_%s',name,target,stamp);replay=struct('source',fullfile(sourceFolder,sprintf('%s_%dmm_truth.mat',name,target)));
  if strcmp(name,'rotation')
   a=load(fullfile(root,'results','runs',sprintf('inclined_insertion_noisy%dmm_exhaustive_%s',target,stamp),'comparison.mat'),'runs');replay.initialEstimates=a.runs(:,21)';
  end
  path=fullfile(root,'results','runs',tag,'comparison.mat');
  if isfile(path),a=load(path,'runs');T=41;if strcmp(name,'rotation'),T=25;end;if size(a.runs,2)==T&&all(~cellfun(@isempty,a.runs),'all'),fprintf('SKIP COMPLETE %s\n',tag);continue,end;error('Partial result exists; preserve and choose a fresh stamp.');end
  fprintf('START EXHAUSTIVE %s\n',tag);run_inclined_modes(Inf,tag,cfg,replay);fprintf('DONE EXHAUSTIVE %s\n',tag);
 end
end
end
