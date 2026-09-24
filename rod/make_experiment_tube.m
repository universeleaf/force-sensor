function tube = make_experiment_tube(cfg)
%MAKE_EXPERIMENT_TUBE Construct the same rod for forward/inverse/baseline.
if isfield(cfg,'rod') && isfield(cfg.rod,'sMm')
    s = cfg.rod.sMm(:)';
    u = cfg.rod.intrinsicCurvaturePerMm;
    assert(numel(s)>=2 && all(isfinite(s)) && s(1)==0 && all(diff(s)>0), ...
        'Custom arclength must start at zero and strictly increase.');
    assert(abs(s(end)-cfg.exposedLengthMm)<1e-8, 'Custom rod length mismatch.');
    assert(isequal(size(u),[3 numel(s)]) && all(isfinite(u(:))), ...
        'Custom intrinsic curvature must be finite and 3-by-numel(s).');
    tube = CreatTube(cfg.exposedLengthMm,s,u);
else
    tube = CreatTube(cfg.exposedLengthMm);
end
end
