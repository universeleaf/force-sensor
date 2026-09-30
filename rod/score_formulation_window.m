function metrics = score_formulation_window(estimate, truth)
%SCORE_FORMULATION_WINDOW Scoring boundary; truth is never passed to inference.
T=size(truth.tipForce,2); K=size(truth.contactForces,2); inferred=size(estimate.contactForce,2);
metrics=struct('trueContactCount',K,'candidateCount',inferred,'countMatches',K==inferred, ...
    'totalForceRmseN',sqrt(mean(sum((estimate.totalForceResultant-truth.totalForce).^2,1))), ...
    'tipForceRmseN',sqrt(mean(sum((estimate.tipForce-truth.tipForce).^2,1))), ...
    'contactForceRmseN',nan,'contactMagnitudeMaeN',nan,'contactArcRmseMm',nan, ...
    'shapeRmseMm',nan,'reviewCount',sum(estimate.quality.requiresReview), ...
    'frameCount',T,'optimizationSeconds',estimate.optimizationSeconds,'perFrame',[]);
rows=repmat(struct('frame',0,'trueForceN',[],'estimatedForceN',[], ...
    'contactVectorErrorN',[],'trueArcMm',[],'estimatedArcMm',[],'tipErrorN',0, ...
    'totalErrorN',0,'shapeRmseMm',0,'requiresReview',false),1,T);
shapeError=zeros(1,T);
for k=1:T
    rows(k).frame=k; rows(k).trueForceN=vecnorm(truth.contactForces(:,:,k));
    rows(k).estimatedForceN=vecnorm(estimate.contactForce(:,:,k));
    rows(k).trueArcMm=truth.contactS(:,k)'; rows(k).estimatedArcMm=estimate.contactArcLength(:,k)';
    rows(k).tipErrorN=norm(estimate.tipForce(:,k)-truth.tipForce(:,k));
    rows(k).totalErrorN=norm(estimate.totalForceResultant(:,k)-truth.totalForce(:,k));
    shapeError(k)=sqrt(mean(sum((estimate.frames{k}.shape.p-truth.p(:,:,k)).^2,1)));
    rows(k).shapeRmseMm=shapeError(k); rows(k).requiresReview=estimate.quality.requiresReview(k);
    if K==inferred, rows(k).contactVectorErrorN=vecnorm(estimate.contactForce(:,:,k)-truth.contactForces(:,:,k)); end
end
if K==inferred
    d=estimate.contactForce-truth.contactForces;
    metrics.contactForceRmseN=sqrt(mean(sum(d.^2,1),'all'));
    metrics.contactMagnitudeMaeN=mean(abs(vecnorm(estimate.contactForce)-vecnorm(truth.contactForces)),'all');
    metrics.contactArcRmseMm=sqrt(mean((estimate.contactArcLength-truth.contactS).^2,'all'));
end
metrics.shapeRmseMm=sqrt(mean(shapeError.^2)); metrics.perFrame=rows;
end
