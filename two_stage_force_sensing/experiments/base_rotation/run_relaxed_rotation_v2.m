function run_relaxed_rotation_v2(action)
% Explicit revised configuration; original benchmark paths never overwritten.
if nargin<1,action='all';end
[root,~]=setup_tsfs;cfg=struct('sensorStd',1e-4,'forcePriors',false,'constraintTolerance',1e-6,'deduplicateGap',true);
for tag={'375deg','1deg'}
 setenv('TSFS_RUN_ID',['base_rotation_relaxed_v2_' tag{1} '_20261007']);[~,paths]=setup_tsfs;folder=fullfile(paths.results,'rotation');if ~isfolder(folder),mkdir(folder);end
 source=fullfile(root,'results','runs',['base_rotation_relaxed_' tag{1} '_20261007'],'rotation','truth_mu_300.mat');
 if ~isfile(fullfile(folder,'truth_mu_300.mat')),copyfile(source,folder);end
 atomic_write_artifact(fullfile(folder,'configuration.json'),'json',cfg);
 for solver={'ipopt','fmincon'}
  if strcmp(action,'smoke'),run_relaxed_rotation(solver{1},1,cfg);else,run_relaxed_rotation(solver{1},Inf,cfg);end
 end
 if strcmp(action,'smoke'),break,end
end
end
