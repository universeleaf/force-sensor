function test_result_integrity()
%TEST_RESULT_INTEGRITY Reject corrupted arrays even if cached metrics look good.
root = fileparts(fileparts(mfilename('fullpath')));
d = load(fullfile(root,'out','wall','results.mat'),'results');
r = d.results;
% Alter a body point far from contact, leaving cached shapeRmseMm untouched.
r.ours.p(1,20,1) = r.ours.p(1,20,1) + 100;
rejected = false;
try
    validate_rod_plane_displacement_results(r);
catch
    rejected = true;
end
assert(rejected, 'Validation trusted a stale shape metric and accepted corrupted geometry.');
r = d.results;
r.ours.tipForce(1,1) = NaN;
rejected = false;
try
    validate_rod_plane_displacement_results(r);
catch
    rejected = true;
end
assert(rejected, 'Validation accepted a nonfinite independent tip force.');
r=d.results;
r.measurements.historySource='sparse-paired';
r.truthConsistency.maxInequalityViolation(:)=NaN;
r.truthConsistency.maxEqualityResidual(:)=NaN;
report=validate_rod_plane_displacement_results(r);
assert(~report.truthConsistencyEvaluated && isnan(report.maxTruthInequalityViolation) && ...
    isnan(report.maxTruthEqualityResidual), ...
    'Missing diagnostics must not be reported as zero residuals.');
r.measurements.historySource='oracle';
rejected=false;
try
    validate_rod_plane_displacement_results(r);
catch
    rejected=true;
end
assert(rejected,'Oracle validation accepted missing truth-consistency diagnostics.');
disp('Result integrity checks passed.');
end
