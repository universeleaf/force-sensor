function test_force_sensitivity()
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(root,'out','stage1','wall_tip_seeded','sensor_input.mat'),'sensorInput');
input=d.sensorInput; tube=input.tube;
m=measurements_from_sensor_packet(tube,input.packet,input.config);
previous=m.previousShape{end}; J=computeJacobian(previous.R,previous.p);
tip=force_sensitivity_diagnostic(tube,J,tube.s(end),m.fbgIdx,5e-5);
assert(tip.localRank<=3&&~tip.locallySeparable&&isinf(tip.conditionNumber), ...
    'Coincident contact/tip forces were reported separable.');
assert(norm(tip.forceToBendingMatrix*[1;-2;0.5;-1;2;-0.5])<1e-12);
body=force_sensitivity_diagnostic(tube,J,0.7*tube.s(end),m.fbgIdx,5e-5);
assert(body.localRank==6&&body.locallySeparable,'Separated reference load geometry lost rank.');
assert(size(body.forceToBendingMatrix,1)==2*numel(m.fbgIdx)&&~body.unmeasuredTorsionUsed);
scaled=force_sensitivity_diagnostic(tube,J,0.7*tube.s(end),m.fbgIdx,1e-4);
assert(max(abs(scaled.conditionalNoiseStdN-2*body.conditionalNoiseStdN))<1e-12);
disp('Measured-channel force sensitivity: tip degeneracy and noise scaling passed.');
end
