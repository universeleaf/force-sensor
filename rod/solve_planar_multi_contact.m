function result = solve_planar_multi_contact(model, seed)
% Frictionless smooth body contacts on fixed planes, solved simultaneously.
% Energy minimization supplies an initial branch only. Final equilibrium,
% contact closure and tangency use independent continuous ODE shooting.
K=numel(model.contactPlaneIndex); L=model.sMm(end); scale=model.EINmm2/L;
normals=model.planeNormalXZ; normals=normals./vecnorm(normals);
model.planeNormalXZ=normals;
if nargin<2 || isempty(seed)
    coarse=solve_planar_energy_rod(model);
    seed.normalForceN=zeros(1,K); seed.contactS=zeros(1,K);
    bounds=linspace(0,L,K+1);
    for j=1:K
        plane=model.contactPlaneIndex(j);
        mask=model.sMm(2:end)>bounds(j)&model.sMm(2:end)<bounds(j+1);
        f=coarse.planeNormalReactionN(plane,mask);
        ss=model.sMm(find(mask)+1);
        assert(sum(f)>1e-5,'rod:MissingContactBranch','Energy seed has no reaction for contact %d.',j);
        seed.normalForceN(j)=sum(f);
        seed.contactS(j)=sum(f.*ss)/sum(f);
    end
    seed.baseMomentNmm=model.EINmm2*(coarse.segmentCurvaturePerMm(1)-model.intrinsicCurvaturePerMm(1));
end
x0=[seed.baseMomentNmm/scale;seed.normalForceN(:);seed.contactS(:)/L];
% Order bounds select disjoint smooth-contact branches, not contact points.
edges=[0 (seed.contactS(1:end-1)+seed.contactS(2:end))/2 L]/L;
lb=[-inf;zeros(K,1);edges(1:end-1)'+1e-6];
ub=[inf;inf(K,1);edges(2:end)'-1e-6];
options=optimoptions('lsqnonlin','Display','off','FunctionTolerance',1e-15, ...
    'OptimalityTolerance',1e-11,'StepTolerance',1e-12,'MaxIterations',100, ...
    'MaxFunctionEvaluations',2000,'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-5);
[x,~,res,flag,output]=lsqnonlin(@residual,x0,lb,ub,options);
contactS=x(K+2:end)'*L;
query=unique([model.sMm linspace(0,L,2001) contactS]);
result=integrate_planar_multi_contact(model,x(1)*scale,contactS,x(2:K+1),model.tipForceXZ,query);
result.audit=audit_planar_multi_contact(model,result);
result.exitflag=flag; result.iterations=output.iterations; result.rootResidual=max(abs(res));
assert(result.audit.passed,'rod:InvalidMultiContactTruth', ...
    'Contact truth fails audit: penetration %.3g mm; gap %.3g mm; moment %.3g N mm; tangency %.3g.', ...
    result.audit.maxPenetrationMm,result.audit.maxAbsGapMm, ...
    result.audit.tipMomentResidualNmm,result.audit.maxTangencyResidual);
    function r=residual(v)
        q=integrate_planar_multi_contact(model,v(1)*scale,v(K+2:end)*L,v(2:K+1),model.tipForceXZ);
        n=normals(:,model.contactPlaneIndex); points=model.planePointXZ(:,model.contactPlaneIndex);
        r=[q.tipMomentResidualNmm/scale; ...
            sum(n.*(q.contactPointsXZ-points),1)'/L;sum(n.*q.contactTangentXZ,1)'];
    end
end
