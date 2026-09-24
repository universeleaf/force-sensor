function test_cosserat_collision_geometry()
% An analytic circular arc penetrates BETWEEN rod nodes. The continuous ODE
% sampler must catch it, independently of the synthetic optimization fixture.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
s=0:10:90; kappa=pi/(2*47.3);
tube=CreatTube(90,s,repmat([0;kappa;0],1,numel(s)));
options=struct('relativeTolerance',2e-9,'collisionStepMm',.5);
shape=solve_cosserat_force_map(tube,47.3,zeros(3,1),zeros(3,1),options);
expected=[(1-cos(kappa*shape.collisionS))/kappa;zeros(size(shape.collisionS)); ...
    sin(kappa*shape.collisionS)/kappa];
assert(max(vecnorm(shape.collisionP-expected))<1e-6,'ODE collision geometry does not match the analytic circle.');
ceiling=1/kappa-.02;
assert(min(ceiling-shape.p(3,:))>0 && min(ceiling-shape.collisionP(3,:))<-.019, ...
    'Collision check missed penetration between coarse rod nodes.');
assert(max(diff(shape.collisionS))<=.5+1e-12 && abs(shape.contactTangent(3))<1e-7);
Q=[0 0 1;1 0 0;0 1 0]; shift=[2;-7;9];
tube.T_base=[Q shift;0 0 0 1];
rotated=solve_cosserat_force_map(tube,47.3,zeros(3,1),zeros(3,1),options);
assert(max(vecnorm(rotated.collisionP-(Q*shape.collisionP+shift)))<1e-6 && ...
    norm(rotated.contactTangent-Q*shape.contactTangent)<1e-7, ...
    'Continuous collision geometry lost rigid-transform equivariance.');
disp('Continuous Cosserat collision sampling catches between-node penetration.');
end
