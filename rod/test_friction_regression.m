function test_friction_regression()
%TEST_FRICTION_REGRESSION Saved trajectories plus adversarial constraint test.
test_friction_basis();
root=fileparts(fileparts(mfilename('fullpath')));
for scenario={'wall','senior'}
    saved=load(fullfile(root,'out',scenario{1},'results.mat'));
    validate_rod_plane_displacement_results(saved.results);
end
r=saved.results;
assert(strcmp(r.ours.complementarityMode{end},'sliding-contact'));
assert(r.ours.frictionForceResultant(1,end)<0, 'Senior friction must oppose +x slip.');
% Opposing beta coefficients cancel in F_t: a circular-cone-only test misses
% this violation of the actual polyhedral constraint sum(beta) <= mu*f_n.
bad=r; m=r.config.forceSensor.numFrictionDirs; k=size(r.ours.state,2);
extra=max(1,r.forward.frictionMu(k)*r.ours.state(7,k));
bad.ours.state(8,k)=bad.ours.state(8,k)+extra;
bad.ours.state(8+m/2,k)=bad.ours.state(8+m/2,k)+extra;
rejected=false;
try
    validate_friction(bad);
catch err
    rejected=contains(err.message,'Polyhedral friction cone');
end
assert(rejected, 'Audit failed to reject canceling beta coefficients outside the polyhedral cone.');
% A reversed force can remain inside both cones. Direction needs its own
% observed-displacement check, independent of the predicted shape.
bad=r;
bad.ours.state(8:7+m,k)=circshift(r.ours.state(8:7+m,k),m/2);
normal=r.ours.planeNormal(:,k)*r.ours.state(7,k);
bad.ours.contactForceResultant(:,k)=2*normal-r.ours.contactForceResultant(:,k);
rejected=false;
try
    validate_friction(bad);
catch err
    rejected=contains(err.message,'assists observed sliding');
end
assert(rejected, 'Audit failed to reject friction that assists observed sliding.');
bad=r; bad.ours.state(8+m,k)=0;
rejected=false;
try
    validate_friction(bad);
catch err
    rejected=contains(err.message,'w >= 0');
end
assert(rejected, 'Audit failed to reject a negative tangential complementarity slack.');
disp('Saved regressions and deliberate cone/direction/w violations passed.');
end
