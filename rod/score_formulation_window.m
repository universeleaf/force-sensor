function metrics = score_formulation_window(estimate, truth)
%SCORE_FORMULATION_WINDOW Scoring boundary; truth is never passed to inference.
T=size(truth.tipForce,2); K=size(truth.contactForces,2); inferred=size(estimate.contactForce,2);
assert(size(estimate.tipForce,2)==T,'rod:ScoreFrameMismatch','Truth and estimate must cover the same frames.');
active=true(inferred,T);if isfield(estimate,'activeContacts'),active=estimate.activeContacts;end
counts=sum(active,1);matches=counts==K;
metrics=struct('trueContactCount',K,'candidateCount',inferred,'candidateCountMatches',K==inferred, ...
    'activeContactCount',counts,'countMatches',all(matches),'matchedFrameCount',sum(matches), ...
    'totalForceRmseN',sqrt(mean(sum((estimate.totalForceResultant-truth.totalForce).^2,1))), ...
    'tipForceRmseN',sqrt(mean(sum((estimate.tipForce-truth.tipForce).^2,1))), ...
    'contactForceRmseN',nan,'contactMagnitudeMaeN',nan,'contactArcRmseMm',nan, ...
    'shapeRmseMm',nan,'reviewCount',sum(estimate.quality.requiresReview), ...
    'frameCount',T,'optimizationSeconds',estimate.optimizationSeconds,'perFrame',[]);
rows=repmat(struct('frame',0,'trueForceN',[],'estimatedForceN',[], ...
    'contactVectorErrorN',[],'trueArcMm',[],'estimatedArcMm',[],'tipErrorN',0, ...
    'totalErrorN',0,'shapeRmseMm',0,'requiresReview',false),1,T);
shapeError=zeros(1,T);forceErrors=[];magnitudeErrors=[];arcErrors=[];
for k=1:T
    rows(k).frame=k; rows(k).trueForceN=vecnorm(truth.contactForces(:,:,k));
    slots=find(active(:,k));[~,order]=sort(estimate.contactArcLength(slots,k));slots=slots(order);
    rows(k).estimatedForceN=vecnorm(estimate.contactForce(:,slots,k));
    rows(k).trueArcMm=truth.contactS(:,k)'; rows(k).estimatedArcMm=estimate.contactArcLength(slots,k)';
    rows(k).tipErrorN=norm(estimate.tipForce(:,k)-truth.tipForce(:,k));
    rows(k).totalErrorN=norm(estimate.totalForceResultant(:,k)-truth.totalForce(:,k));
    shapeError(k)=sqrt(mean(sum((estimate.frames{k}.shape.p-truth.p(:,:,k)).^2,1)));
    rows(k).shapeRmseMm=shapeError(k); rows(k).requiresReview=estimate.quality.requiresReview(k);
    if matches(k)
        f=estimate.contactForce(:,slots,k);delta=f-truth.contactForces(:,:,k);
        rows(k).contactVectorErrorN=vecnorm(delta);
        forceErrors=[forceErrors,sum(delta.^2,1)]; %#ok<AGROW>
        magnitudeErrors=[magnitudeErrors,abs(vecnorm(f)-vecnorm(truth.contactForces(:,:,k)))]; %#ok<AGROW>
        arcErrors=[arcErrors,(estimate.contactArcLength(slots,k)-truth.contactS(:,k))'.^2]; %#ok<AGROW>
    end
end
if all(matches)&&K>0
    metrics.contactForceRmseN=sqrt(mean(forceErrors));metrics.contactMagnitudeMaeN=mean(magnitudeErrors);
    metrics.contactArcRmseMm=sqrt(mean(arcErrors));
elseif K==0&&all(matches)
    metrics.contactForceRmseN=0;metrics.contactMagnitudeMaeN=0;metrics.contactArcRmseMm=0;
end
metrics.shapeRmseMm=sqrt(mean(shapeError.^2)); metrics.perFrame=rows;
end
