function metrics = compute_aloi_comparison_metrics(forward, aloi, cfg)
%COMPUTE_ALOI_COMPARISON_METRICS Evaluate the raw fit and paper assumptions.

trueTotal = forward.totalForceResultant;
estimatedTotal = aloi.totalForceResultant;
error = estimatedTotal - trueTotal;
active = vecnorm(trueTotal, 2, 1) > 1e-8;
if ~any(active)
    active = true(1, size(trueTotal, 2));
end

metrics = struct;
metrics.resultantRmse = sqrt(mean(vecnorm(error(:, active), 2, 1) .^ 2));
metrics.finalErrorNorm = norm(error(:, end));
metrics.finalRelativeErrorPct = 100 * metrics.finalErrorNorm / ...
    max(norm(trueTotal(:, end)), eps);
metrics.finalEstimatedForce = estimatedTotal(:, end);
metrics.finalTrueForce = trueTotal(:, end);
metrics.finalMagnitudeErrorPct = 100 * ...
    abs(norm(estimatedTotal(:, end)) - norm(trueTotal(:, end))) / ...
    max(norm(trueTotal(:, end)), eps);
directionCosine = dot(estimatedTotal(:, end), trueTotal(:, end)) / ...
    max(norm(estimatedTotal(:, end)) * norm(trueTotal(:, end)), eps);
metrics.finalDirectionErrorDeg = acosd(min(max(directionCosine, -1), 1));

trueContactS = forward.contactArcLength;
validContactLocation = active & isfinite(trueContactS) & isfinite(aloi.centerMm);
locationErrorMm = abs(aloi.centerMm - trueContactS);
metrics.contactLocationErrorMm = locationErrorMm;
if any(validContactLocation)
    metrics.contactLocationRmseMm = ...
        sqrt(mean(locationErrorMm(validContactLocation) .^ 2));
else
    metrics.contactLocationRmseMm = nan;
end
metrics.finalContactLocationErrorMm = locationErrorMm(end);
metrics.finalShapeRmseMm = aloi.shapeRmseMm(end);
metrics.widthAtLowerBound = abs(aloi.sigmaMm - ...
    min(cfg.aloi.sigmaCandidatesMm)) <= 1e-6;

[trueLocalContactForce, trueAxialFraction] = ...
    contactForceInMaterialFrame(forward);
metrics.trueLocalContactForce = trueLocalContactForce;
metrics.trueAxialFractionPct = 100 * trueAxialFraction;
metrics.finalTrueLocalContactForce = trueLocalContactForce(:, end);
metrics.finalTrueAxialFractionPct = 100 * trueAxialFraction(end);
metrics.withinPaperTransverseLoadAssumption = ...
    trueAxialFraction <= cfg.aloi.maxAxialFractionForComparison;
metrics.finalWithinPaperTransverseLoadAssumption = ...
    metrics.withinPaperTransverseLoadAssumption(end);
end


function [localForce, axialFraction] = contactForceInMaterialFrame(forward)
nt = size(forward.contactForceResultant, 2);
localForce = nan(3, nt);
axialFraction = nan(1, nt);
for it = 1:nt
    contactS = forward.contactArcLength(it);
    force = forward.contactForceResultant(:, it);
    if ~isfinite(contactS) || norm(force) <= eps
        continue;
    end
    [~, idx] = min(abs(forward.s(:) - contactS));
    R = forward.R(:, :, idx, it);
    localForce(:, it) = R' * force;
    axialFraction(it) = abs(localForce(3, it)) / norm(force);
end
end
