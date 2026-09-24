function diagnostic=nonlinear_force_sensitivity(tube,d,idx,sigma,cfg)
% Conditional derivative of the EQUILIBRATED map, not the frozen lever arm.
opts=struct('relativeTolerance',cfg.mechanicsRelativeTolerance, ...
    'momentToleranceNmm',cfg.mechanicsMomentToleranceNmm);
if isfield(cfg,'mechanicsMaxRhsEvaluations'),opts.maxRhsEvaluations=cfg.mechanicsMaxRhsEvaluations;end
f=[d.contactForce;d.tipForce]; G=zeros(2*numel(idx),6); step=1e-4;
for j=1:6
    fp=f; fm=f; fp(j)=fp(j)+step; fm(j)=fm(j)-step;
    a=solve_cosserat_force_map(tube,d.s1,fp(1:3),fp(4:6),opts);
    b=solve_cosserat_force_map(tube,d.s1,fm(1:3),fm(4:6),opts);
    du=(a.u(1:2,idx)-b.u(1:2,idx))/(2*step); G(:,j)=du(:);
end
[~,S,V]=svd(G,'econ'); sv=diag(S); sv(end+1:6)=0;
rankValue=sum(sv>max(sv)*1e-8); condition=Inf; noise=nan(6,1);
if rankValue==6
    condition=sv(1)/sv(6);
    if sigma>0,noise=sigma*sqrt(sum((V./sv').^2,2));end
end
diagnostic=struct('scope','Conditional nonlinear-equilibrium derivative; fixed contact arclength/rod/base; two observed bending channels. Not augmented-state observability or coverage.', ...
    'singularValuesPerMmPerN',sv,'rankRelativeTolerance',1e-8,'localRank',rankValue, ...
    'conditionNumber',condition,'locallySeparable',rankValue==6, ...
    'contactTipSeparationMm',tube.s(end)-d.s1,'conditionalNoiseStdN',noise, ...
    'conditionalNoiseStdAvailable',rankValue==6&&sigma>0, ...
    'unmeasuredTorsionUsed',false,'forceToBendingMatrix',G);
end
