function score=score_formulation_coverage(estimate,truth,noiseStd)
% Empirical component coverage conditional on the selected contact branch.
% Unresolved/mismatched components are reported, never counted as successes.
score=struct('computed',false,'nominalProbability',.95,'coveredComponents',0, ...
    'eligibleComponents',0,'unresolvedComponents',0,'unmatchedFrames',0, ...
    'empiricalCoverage',nan,'meanIntervalWidthN',nan,'frameCoverage',[], ...
    'scope','Local Gaussian 1.95996398454005 SD intervals, active arc-order pairing; fixed synthetic trajectory, no candidate/mode marginalization.');
if noiseStd==0,score.scope=[score.scope ' Clean deterministic cases do not calibrate coverage.'];return;end
if ~isfield(estimate,'uncertainty')||~estimate.uncertainty.computed,return;end
score.computed=true;K=size(estimate.contactForce,2);Kt=size(truth.contactForces,2);
T=size(truth.tipForce,2);stdv=reshape(estimate.uncertainty.forceStd,3*(K+1),T);
unresolved=reshape(estimate.uncertainty.forceUnresolvedByData,3*(K+1),T);widths=[];
rows=repmat(struct('covered',0,'eligible',0,'unresolved',0,'countMatches',false),1,T);
for k=1:T
    slots=find(estimate.activeContacts(:,k));[~,order]=sort(estimate.contactArcLength(slots,k));slots=slots(order);
    match=numel(slots)==Kt;rows(k).countMatches=match;
    if ~match,score.unmatchedFrames=score.unmatchedFrames+1;end
    indices=3*K+(1:3);difference=estimate.tipForce(:,k)-truth.tipForce(:,k);
    if match
        indices=[reshape((3*(slots(:)'-1)+(1:3)'),[],1);indices(:)];
        delta=estimate.contactForce(:,slots,k)-truth.contactForces(:,:,k);difference=[delta(:);difference];
    end
    sigma=stdv(indices,k);eligible=isfinite(sigma)&sigma>=0&~unresolved(indices,k);
    covered=abs(difference)<=1.95996398454005*sigma;
    rows(k).covered=sum(covered&eligible);rows(k).eligible=sum(eligible);rows(k).unresolved=sum(~eligible);
    widths=[widths;2*1.95996398454005*sigma(eligible)]; %#ok<AGROW>
end
score.coveredComponents=sum([rows.covered]);score.eligibleComponents=sum([rows.eligible]);
score.unresolvedComponents=sum([rows.unresolved]);score.frameCoverage=rows;
if score.eligibleComponents>0
    score.empiricalCoverage=score.coveredComponents/score.eligibleComponents;score.meanIntervalWidthN=mean(widths);
end
end
