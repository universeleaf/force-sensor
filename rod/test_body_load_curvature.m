function test_body_load_curvature()
% Check body-frame integration against independent world-frame mechanics.
[input,truth]=build_formulation_multi_packet([],3);
tube=input.tube;tube.T_base=input.packet.basePose(:,:,1);K=size(truth.contactForces,2);
stiffness=reshape(getTubeK(tube),3,[]);
m0=truth.R(:,:,1,1)*(stiffness(:,1).*(truth.u(:,1,1)-tube.uhat(:,1)));
reference=integrate_cosserat_load_state(tube,m0,truth.contactS(:,1),truth.contactForces(:,:,1),truth.tipForce(:,1));
[~,contactR]=cosserat_state_at_arc(reference,truth.contactS(:,1));
local=zeros(3,K);
for j=1:K
    local(:,j)=contactR(:,:,j)'*truth.contactForces(:,j,1);
end
ft=reference.R(:,:,end)'*truth.tipForce(:,1);
sh=integrate_body_load_curvature(tube,truth.contactS(:,1),local,ft);
assert(max(vecnorm(sh.p-truth.p(:,:,1)))<3e-4&&max(abs(sh.u-truth.u(:,:,1)),[],'all')<2e-8, ...
    'rod:BodyCurvatureRegression','Body-frame curvature must agree with independent planar truth.');
assert(norm(sh.tipForce-truth.tipForce(:,1))<1e-6&& ...
    max(vecnorm(sh.contactForce-truth.contactForces(:,:,1)))<1e-4,'rod:BodyForceFrameRegression', ...
    'Recovered force resultants must be in the world frame.');
% A finite-width Gaussian is distributed loading, not a renamed point load.
local(3,:)=0;
g=integrate_body_load_curvature(tube,truth.contactS(:,1),local,ft,struct('gaussianSigmaMm',[1 2]));
world=solve_cosserat_multi_contact_map(tube,[],zeros(3,0),g.tipForce,struct('maxRhsEvaluations',1000000));
assert(norm(g.p-world.p,'fro')>0.01&&all(isfinite(g.contactForce),'all'), ...
    'rod:GaussianLoadRegression','A Gaussian must affect equilibrium and have finite world resultants.');
fprintf('test_body_load_curvature: PASS\n');
end
