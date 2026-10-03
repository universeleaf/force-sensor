function [accepted, initialMerit, candidateMerit] = formulation_restoration_merit(initialResidual, candidateResidual)
%FORMULATION_RESTORATION_MERIT Safeguard a numerical feasibility initializer.
% Compare the SAME fixed residual: physics, proximity, and weak original MAP.
% A stopped least-squares solve is not evidence that its returned point has
% improved this merit. Never let a worse returned initializer replace its
% starting point. This does not change the subsequent MAP objective.
assert(isvector(initialResidual)&&isvector(candidateResidual)&& ...
    numel(initialResidual)==numel(candidateResidual)&&isreal(initialResidual)&& ...
    isreal(candidateResidual)&&all(isfinite(initialResidual)), ...
    'rod:InvalidRestorationResidual','Restoration residuals must have matching real vector dimensions.');
initialMerit=sum(initialResidual(:).^2);
candidateMerit=sum(candidateResidual(:).^2);
assert(isfinite(initialMerit),'rod:InvalidRestorationResidual','Initial restoration merit must be finite.');
accepted=isfinite(candidateMerit)&&candidateMerit<=initialMerit+1e-10*max(1,initialMerit);
end
