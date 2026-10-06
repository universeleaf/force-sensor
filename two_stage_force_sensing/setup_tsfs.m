function [root,paths]=setup_tsfs
% Resolve our package and the repository's existing, unmodified dependencies.
root=fileparts(mfilename('fullpath'));repo=fileparts(root);baseline=repo;
localFile=fullfile(root,'.local.json');
if isfile(localFile),local=jsondecode(fileread(localFile));if isfield(local,'baselineRoot'),baseline=local.baselineRoot;end,end
if ~isempty(getenv('TSFS_BASELINE_ROOT')),baseline=getenv('TSFS_BASELINE_ROOT');end
assert(isfolder(fullfile(baseline,'rod')),'tsfs:MissingBaseline','Expected rod/ beside two_stage_force_sensing/.');
assert(isfolder(fullfile(baseline,'LCP-Continuum')),'tsfs:MissingDependency', ...
    'Provide the existing LCP-Continuum dependency at the repository root; see README.');
addpath(fullfile(baseline,'rod'));
lcp=validate_lcp_dependency(baseline);
addpath(root,fullfile(root,'matlab'),fullfile(root,'tests'),fullfile(root,'baseline'));
folders={'benchmarks','bottleneck_audit','s_channel','plane_sensitivity','fbg_reconstruction','single_contact'};
for k=1:numel(folders),addpath(fullfile(root,'experiments',folders{k}));end
names={'integrate_curvature_field','formulation_window_observations','estimate_sensor_forces','estimate_planar_multi_contact','estimate_formulation_window'};
for k=1:numel(names)
 actual=which(names{k});expected=fullfile(baseline,'rod',[names{k} '.m']);
 assert(strcmpi(strrep(actual,'\','/'),strrep(expected,'\','/')),'tsfs:ShadowedDependency','Unexpected source for %s: %s',names{k},actual);
end
runId=getenv('TSFS_RUN_ID');if isempty(runId),runId='current';end
assert(~isempty(regexp(runId,'^[A-Za-z0-9][A-Za-z0-9_-]*$','once')),'tsfs:InvalidRunId','TSFS_RUN_ID must be a simple directory name.');
paths=struct('repo',repo,'baseline',baseline,'datasets',fullfile(root,'datasets'),'results',fullfile(root,'results','runs',runId), ...
 'published',fullfile(root,'results','published'),'lcp',lcp.path,'lcpRevision',lcp.gitRevision);
if ~isfolder(paths.results),mkdir(paths.results);end
end
