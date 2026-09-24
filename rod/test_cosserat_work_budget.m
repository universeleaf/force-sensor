function test_cosserat_work_budget()
% A pathological optimizer trial must terminate without returning partial
% mechanics; the ordinary loaded solution remains independent of its budget.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
s=0:5:90;tube=CreatTube(90,s,repmat([0;.006;0],1,numel(s)));
fc=[.1;0;-.4];fe=[.05;0;-.1];
a=solve_cosserat_force_map(tube,65,fc,fe);
assert(a.rhsEvaluations>0&&a.rhsEvaluations<50000);
b=solve_cosserat_force_map(tube,65,fc,fe,struct('maxRhsEvaluations',100000));
assert(norm(a.p-b.p,'fro')<1e-10&&a.tipMomentResidualNmm<2e-4);
try
    solve_cosserat_force_map(tube,65,[1e8;0;0],zeros(3,1),struct('maxRhsEvaluations',1000));
    error('test:MissedBudget','Pathological mechanics trial returned a shape.');
catch err
    assert(strcmp(err.identifier,'rod:CosseratEquilibrium')&&contains(err.message,'budget exceeded'), ...
        'Expected controlled mechanics rejection, got %s: %s',err.identifier,err.message);
end
% A rejected solve must not poison the exact memoization cache.
c=solve_cosserat_force_map(tube,65,fc,fe);
assert(norm(a.p-c.p,'fro')<1e-10);
try
    solve_cosserat_force_map(tube, tube.s(end)+1, fc, fe);
    error('test:AcceptedInvalidArc','Out-of-domain contact arc length was accepted.');
catch err
    assert(strcmp(err.identifier,'rod:InvalidContactArcLength'), ...
        'Expected an explicit arc-length rejection, got %s: %s',err.identifier,err.message);
end
bad=tube; bad.uhat(:,1)=NaN;
try
    solve_cosserat_force_map(bad,65,fc,fe);
    error('test:AcceptedInvalidCurvature','Invalid intrinsic curvature was accepted.');
catch err
    assert(strcmp(err.identifier,'rod:InvalidTubeCurvature'), ...
        'Expected an explicit curvature rejection, got %s: %s',err.identifier,err.message);
end
try
    solve_cosserat_force_map(tube,65,fc(1:2),fe);
    error('test:AcceptedInvalidForce','Invalid force vector was accepted.');
catch err
    assert(strcmp(err.identifier,'rod:InvalidForceVector'), ...
        'Expected an explicit force rejection, got %s: %s',err.identifier,err.message);
end
fprintf('Cosserat work budget passed; ordinary equilibrium used %d RHS calls.\n',a.rhsEvaluations);
end
