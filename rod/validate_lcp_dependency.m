function report = validate_lcp_dependency(rootDir)
%VALIDATE_LCP_DEPENDENCY Check and register the pinned LCP-Continuum tree.
% The experiment has two distinct model layers: LCP-Continuum supplies the
% rod kinematics/stiffness and forward contact primitives, while the inverse
% estimator lives in this repository.  Failing early here prevents a
% different MATLAB installation or a shadowed function from silently
% changing the forward model.

if nargin < 1 || isempty(rootDir)
    thisDir = fileparts(mfilename('fullpath'));
    rootDir = fileparts(thisDir);
end
if ~(ischar(rootDir) || (isstring(rootDir) && isscalar(rootDir)))
    error('rod:InvalidRepositoryRoot', 'Repository root must be a path string.');
end
rootDir = char(rootDir);
lcpDir = fullfile(rootDir, 'LCP-Continuum');
if ~isfolder(lcpDir)
    error('rod:MissingLcpDependency', ...
        'LCP-Continuum is missing. Expected dependency at: %s', lcpDir);
end

required = { ...
    fullfile('tubeKinematics', 'solveShape.m'), ...
    fullfile('tubeKinematics', 'computeJacobian.m'), ...
    fullfile('tubeKinematics', 'getTubeK.m'), ...
    fullfile('LCP', 'LCPSolve.m'), ...
    fullfile('contactMechanics', 'getContactShape3.m'), ...
    fullfile('contactMechanics', 'getFrictionalContactShape3.m')};
missing = false(size(required));
for k = 1:numel(required)
    missing(k) = ~isfile(fullfile(lcpDir, required{k}));
end
if any(missing)
    error('rod:IncompleteLcpDependency', ...
        'LCP-Continuum is incomplete; missing: %s', ...
        strjoin(required(missing), ', '));
end

% Put the checked tree first.  This is intentional: a user-level MATLAB
% path may contain another solveShape/LCPSolve with the same function name.
addpath(genpath(lcpDir), '-begin');
names = {'solveShape', 'computeJacobian', 'getTubeK', 'LCPSolve', ...
    'getContactShape3', 'getFrictionalContactShape3'};
paths = cell(size(names));
for k = 1:numel(names)
    paths{k} = which(names{k});
    expected = fullfile(lcpDir, required{k});
    if isempty(paths{k}) || ~samePath(paths{k}, expected)
        error('rod:LcpFunctionShadowed', ...
            ['LCP-Continuum function %s resolved to ''%s'', but the ', ...
             'checked dependency is ''%s''.'], names{k}, paths{k}, expected);
    end
end

[gitRevision, gitAvailable] = readGitRevision(lcpDir);
if gitAvailable
    sourceRevision = ['Jia0Shen/LCP-Continuum@', gitRevision];
else
    sourceRevision = 'Jia0Shen/LCP-Continuum@local-unversioned';
end

report = struct;
report.rootDir = rootDir;
report.path = lcpDir;
report.requiredFiles = required;
report.functionNames = names;
report.functionPaths = paths;
report.gitRevision = gitRevision;
report.gitAvailable = gitAvailable;
report.sourceRevision = sourceRevision;
report.pathVerified = true;
end


function tf = samePath(a, b)
a = char(a);
b = char(b);
if ispc
    tf = strcmpi(strrep(a, '/', '\'), strrep(b, '/', '\'));
else
    tf = strcmp(a, b);
end
end


function [revision, available] = readGitRevision(repoDir)
revision = '';
available = false;
if ~isfolder(fullfile(repoDir, '.git'))
    return;
end
% The dependency is local and already present; this is a read-only provenance
% query.  If Git is unavailable, the run remains usable but is labelled
% unversioned instead of reporting a fabricated commit.
quoted = ['"', strrep(repoDir, '"', '""'), '"'];
[status, output] = system(['git -C ', quoted, ' rev-parse HEAD']);
if status == 0
    output = strtrim(output);
    if ~isempty(regexp(output, '^[0-9a-fA-F]{7,40}$', 'once'))
        revision = output;
        available = true;
    end
end
end
