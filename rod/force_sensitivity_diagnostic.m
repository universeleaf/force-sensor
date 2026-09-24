function diagnostic = force_sensitivity_diagnostic(tube,J,arcMm,fbgIdx,sigma)
% Conditional six-force sensitivity using the TWO measured bending channels.
% Known predecessor geometry, stiffness and estimated contact arclength.
% Excludes environment, force priors and the unmeasured calibrated torsion.
% Not observability of the nonlinear augmented state or calibrated coverage.
s=tube.s(:)';
assert(isscalar(arcMm)&&isreal(arcMm)&&isfinite(arcMm)&&arcMm>=s(1)&&arcMm<=s(end));
j=find(s<=arcMm,1,'last'); j=min(j,numel(s)-1);
a=(arcMm-s(j))/(s(j+1)-s(j));
Jc=(1-a)*J(3*j-2:3*j,:)+a*J(3*j+1:3*j+3,:);
G=bsxfun(@rdivide,[Jc',J(end-2:end,:)'],getTubeK(tube));
rows=reshape(3*fbgIdx(:)'+[-2;-1],[],1); G=G(rows,:);
[~,S,V]=svd(G,'econ'); singular=diag(S); singular(end+1:6)=0;
cutoff=max(singular)*1e-8; localRank=sum(singular>cutoff);
condition=Inf; conditionalStd=nan(6,1);
if localRank==6
    condition=singular(1)/singular(6);
    if sigma>0, conditionalStd=sigma*sqrt(sum((V./singular').^2,2)); end
end
diagnostic=struct('scope','Two measured bending channels; known predecessor mechanics and estimated contact arclength; no global observability or force-interval coverage claim.', ...
    'singularValuesPerMmPerN',singular,'rankRelativeTolerance',1e-8,'localRank',localRank, ...
    'conditionNumber',condition,'locallySeparable',localRank==6, ...
    'contactTipSeparationMm',s(end)-arcMm,'conditionalNoiseStdN',conditionalStd, ...
    'conditionalNoiseStdAvailable',localRank==6&&sigma>0, ...
    'unmeasuredTorsionUsed',false,'forceToBendingMatrix',G);
end
