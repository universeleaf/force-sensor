function test_multi_contact_state_spec()
%TEST_MULTI_CONTACT_STATE_SPEC Regression for the shared multi-contact API.
cfg = run_rod_plane_force_sensing_experiment('sensor-config');
cfg = formulation_solver_config(cfg);
m = cfg.forceSensor.numFrictionDirs;
s1 = multi_contact_state_spec(cfg, 1, 'independent');
assert(s1.stateLength == 11+m && isequal(s1.contact(1).planePoint,1:3), ...
    'Single-contact state compatibility failed.');
s2 = multi_contact_state_spec(cfg, 2, 'independent');
assert(s2.stateLength == 19+2*m && numel(s2.tipForce)==3, ...
    'Independent two-contact state length is invalid.');
ss = multi_contact_state_spec(cfg, 2, 'shared');
assert(ss.stateLength == 14+2*m && isequal(ss.contact(1).planePoint,ss.planePoint), ...
    'Shared-plane two-contact state length is invalid.');

tube = make_experiment_tube(struct('exposedLengthMm',100));
x = zeros(s2.stateLength,1);
x(s2.contact(1).planePoint) = [0;0;0];
x(s2.contact(2).planePoint) = [0;0;0];
x(s2.contact(1).s) = 20;
x(s2.contact(2).s) = 70;
x(s2.contact(1).normalForce) = 0.3;
x(s2.contact(2).normalForce) = 0.2;
d = decode_multi_contact_state(x,tube,cfg,2,'independent');
assert(isequal(size(d.contactForce),[3 2]) && all(isfinite(d.contactForce),'all'), ...
    'Decoded contact force matrix is invalid.');
fprintf('test_multi_contact_state_spec: PASS (m=%d, K=2 state=%d)\n',m,s2.stateLength);
end
