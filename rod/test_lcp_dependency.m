function test_lcp_dependency()
%TEST_LCP_DEPENDENCY Verify dependency presence and function resolution.
root = fileparts(fileparts(mfilename('fullpath')));
report = validate_lcp_dependency(root);
assert(report.pathVerified && report.gitAvailable, ...
    'The checked LCP-Continuum dependency did not report a Git revision.');
assert(startsWith(report.sourceRevision, 'Jia0Shen/LCP-Continuum@'), ...
    'Unexpected LCP-Continuum provenance string.');
for k = 1:numel(report.functionPaths)
    assert(isfile(report.functionPaths{k}), ...
        'Resolved LCP function is not a file: %s', report.functionPaths{k});
end
fprintf('LCP dependency check passed: %s\n', report.sourceRevision);
end
