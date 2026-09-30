function fit = audit_formulation_window_fit(estimate, familyTailProbability)
%AUDIT_FORMULATION_WINDOW_FIT Noise-scaled observation/model tension diagnostic.
% Uses observations only. The chi-square reference keeps the FULL observation
% dimension rather than asserting fitted-parameter degrees of freedom. This
% is a conservative fixed-model reference, not a calibrated force interval,
% contact-mode test, or guarantee that every missing obstacle is detectable.
if nargin<2,familyTailProbability=1e-3;end
assert(isnumeric(familyTailProbability)&&isscalar(familyTailProbability)&& ...
    isreal(familyTailProbability)&&isfinite(familyTailProbability)&& ...
    familyTailProbability>0&&familyTailProbability<1,'rod:InvalidFitTailProbability', ...
    'Family tail probability must lie strictly between zero and one.');
obs=estimate.observations; T=numel(estimate.frames); nu=size(obs.curvatureWhitening,1);
perFrameTail=familyTailProbability/max(1,T);
% gammaincinv is in base MATLAB; no Statistics Toolbox dependency.
threshold=2*gammaincinv(1-perFrameTail,nu/2);
energy=zeros(1,T);
for k=1:T
    kk=obs.outputIndices(k);
    r=obs.curvatureWhitening(:,:,kk)*(estimate.frames{k}.predictedCurvature(:)- ...
        reshape(obs.u(:,:,kk),[],1));
    energy(k)=sum(r.^2);
end
fit=struct('curvatureResidualSquaredNorm',energy,'observationDimension',nu, ...
    'thresholdSquaredNorm',threshold,'thresholdRms',sqrt(threshold/nu), ...
    'familyTailProbability',familyTailProbability,'perFrameTailProbability',perFrameTail, ...
    'requiresReview',~isfinite(energy)|energy>threshold,'forceAccuracyCertified',false, ...
    'scope','Conservative Gaussian observation reference with Bonferroni across output frames; fitted degrees of freedom, candidate selection, priors and nonlinear modes are not calibrated. High residual signals model/observation tension, not its unique cause.');
end
