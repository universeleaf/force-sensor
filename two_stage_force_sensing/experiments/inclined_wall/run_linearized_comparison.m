function run_linearized_comparison(stamp)
% Matched input/initialization/physics, fresh sequential timing of both solvers.
if nargin<1,stamp='20261011';end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));base=linearized_mode_settings;
for scenario={'insertion','rotation'}
 name=scenario{1};replay=struct;
 if strcmp(name,'rotation')
  folder=fullfile(root,'results','runs','inclined_wall_rotation_20261010');a=load(fullfile(folder,'maximum_insertion_seed.mat'),'seed');replay=struct('source',fullfile(folder,'truth.mat'),'initialEstimates',{a.seed.estimates});
 end
 for variant={'full','linearized'}
  label=variant{1};cfg=base;
  if strcmp(label,'full'),cfg=rmfield(cfg,{'mechanicsApproximation','linearization'});end
  cfg.comparisonScenario=name;cfg.comparisonVariant=label;cfg.allAssignments=false;tag=['inclined_' name '_' label '_' stamp];
  fprintf('START COMPARISON %s\n',tag);run_inclined_modes(Inf,tag,cfg,replay);fprintf('DONE COMPARISON %s\n',tag);
 end
end
end
