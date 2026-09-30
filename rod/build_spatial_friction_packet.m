function [input, truth] = build_spatial_friction_packet(noiseStd, seed)
%BUILD_SPATIAL_FRICTION_PACKET Spatial two-contact sliding equilibrium.
% An independent nested shooting solver determines contact positions and
% normal reactions from fixed walls. Sliding force acts out of the unloaded
% rod's x-z plane. Two translated equilibria give a known +y slip, but the
% inverse receives neither this mode nor the solved forces/locations.
if nargin<1, noiseStd=0; end
if nargin<2, seed=811; end
[input,planar]=build_formulation_multi_packet([],3);
rod=input.tube; rod.T_base=input.packet.basePose(:,:,1);
points=input.packet.planePointMm(:,:,1); normals=input.packet.planeNormal(:,:,1);
mu=0.03; tip=[0.1;0.3;-0.08]; K=2; L=rod.s(end);
% The independent shooting map retains a coarser calibration grid with the
% same exact profile discontinuities, rather than sharing lifted integration.
selection=unique([1:5:numel(rod.s),find(abs(diff(rod.uhat(2,:)))>1e-12)+1,numel(rod.s)]);
forwardRod=rod; forwardRod.s=rod.s(selection); forwardRod.uhat=rod.uhat(:,selection);
fullK=reshape(getTubeK(rod),3,[]); forwardRod.K=fullK(:,selection);
for name={'kb','kt','k'}, if isfield(forwardRod,name{1})&&numel(forwardRod.(name{1}))==numel(rod.s), forwardRod.(name{1})=forwardRod.(name{1})(selection); end; end
shootOpts=struct('relativeTolerance',2e-8,'momentToleranceNmm',1e-4,'collisionStepMm',0.5,'maxRhsEvaluations',1000000);
v0=[vecnorm(planar.contactForces(:,:,1))';planar.contactS(:,1)/L];
rootOpts=optimoptions('fsolve','Display','off','MaxIterations',60,'MaxFunctionEvaluations',800, ...
    'FunctionTolerance',1e-12,'StepTolerance',1e-10,'OptimalityTolerance',1e-10,'FiniteDifferenceStepSize',1e-5);
[v,res,flag]=fsolve(@contactResidual,v0,rootOpts);
[~,shape,force]=contactResidual(v);
assert(flag>0&&max(abs(res))<1e-6&&all(v(1:K)>0), ...
    'rod:InvalidSpatialTruth','Independent spatial contact solve did not converge.');
for j=1:K
    assert(min(normals(:,j)'*(shape.collisionP-points(:,j)))>-1e-3,'rod:InvalidSpatialTruth','Spatial truth penetrates a wall.');
end
T=2; translation=[0;0.5;0]; base=repmat(rod.T_base,1,1,T); base(1:3,4,2)=base(1:3,4,2)+translation;
idx=unique(round(linspace(2,numel(rod.s)-1,24))); rngStream=RandStream('mt19937ar','Seed',seed);
% Query mechanics on the full grid for scoring/FBG output only. No inverse
% state or observations are used by the independently generated equilibrium.
fullShape=solve_cosserat_multi_contact_map(rod,v(K+1:end)'*L,force,tip,shootOpts);
assert(max(abs(sum(normals.*(fullShape.contactPoints-points),1)))<1e-4&& ...
    max(abs(sum(normals.*fullShape.contactTangents,1)))<1e-5, ...
    'rod:SpatialTruthMeshMismatch','Full sensor grid does not preserve spatial contact equilibrium.');
p=fullShape.p; u=fullShape.u; curvature=repmat(u(1:2,idx),1,1,T)+noiseStd*randn(rngStream,2,numel(idx),T);
input.packet=struct('schemaVersion',2,'sFbgMm',rod.s(idx),'observedCurvatureAxes',[1 2], ...
    'curvaturePerMm',curvature,'curvatureStdPerMm',max(noiseStd,1e-7), ...
    'basePose',base,'timeSeconds',[0 0.02],'processReferencePeriodSeconds',0.02, ...
    'planePointMm',repmat(points,1,1,T),'planeNormal',repmat(normals,1,1,T), ...
    'planeCovariance',repmat(diag([0.05^2*ones(1,3),0.001^2*ones(1,3)]),1,1,K,T), ...
    'frictionMu',mu*ones(K,T));
truth=struct('p',cat(3,p,p+translation),'u',repmat(u,1,1,T),'basePose',base, ...
    'contactS',repmat(v(K+1:end)*L,1,T),'contactForces',repmat(force,1,1,T), ...
    'tipForce',repmat(tip,1,T),'totalForce',repmat(sum(force,2)+tip,1,T), ...
    'contactPoints',cat(3,fullShape.contactPoints,fullShape.contactPoints+translation), ...
    'rootResidual',max(abs(res)),'terminalMomentNmm',fullShape.tipMomentResidualNmm, ...
    'slipMm',translation,'seed',seed,'noiseStdPerMm',noiseStd, ...
    'scope','Independent spatial shooting/contact root solve; prescribed constant sliding branch; not stick-slip transition truth.');
    function [r,q,f]=contactResidual(z)
        arcs=z(K+1:end)'*L;
        assert(all(diff(arcs)>0)&&all(arcs>0)&&all(arcs<L),'rod:InvalidSpatialTruthTrial','Contact trial outside ordered rod.');
        f=normals.*z(1:K)'-[0;1;0]*(mu*z(1:K)');
        q=solve_cosserat_multi_contact_map(forwardRod,arcs,f,tip,shootOpts);
        r=[sum(normals.*(q.contactPoints-points),1)'/L;sum(normals.*q.contactTangents,1)'];
    end
end
