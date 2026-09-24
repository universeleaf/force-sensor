function validate_sensor_config(cfg)
%VALIDATE_SENSOR_CONFIG Reject invalid weights/budgets before optimization.
% Call after merging defaults. Zero process noise is valid; likelihood and
% prior standard deviations must be strictly positive.
f=cfg.forceSensor;
if isfield(f,'useObservationScaling')
    value=f.useObservationScaling;
    assert((islogical(value)||isnumeric(value))&&isreal(value)&&isscalar(value)&&any(value==[0 1]), ...
        'rod:InvalidSensorConfig','useObservationScaling must be scalar logical.');
end
if isfield(f,'nonlinearFiniteDifferenceStep')
    scalar(f.nonlinearFiniteDifferenceStep,'nonlinearFiniteDifferenceStep',false);
end
if isfield(f,'useNonlinearShapeSeed')
    value=f.useNonlinearShapeSeed;
    assert((islogical(value)||isnumeric(value))&&isreal(value)&&isscalar(value)&&any(value==[0 1]), ...
        'rod:InvalidSensorConfig','useNonlinearShapeSeed must be scalar logical.');
    assert(~value||strcmp(f.mechanicsModel,'cosserat-shooting'), ...
        'rod:InvalidSensorConfig','Nonlinear shape initialization requires Cosserat mechanics.');
end
if isfield(f,'mechanicsMaxRhsEvaluations')
    scalar(f.mechanicsMaxRhsEvaluations,'mechanicsMaxRhsEvaluations',false);
    assert(f.mechanicsMaxRhsEvaluations==fix(f.mechanicsMaxRhsEvaluations), ...
        'rod:InvalidSensorConfig','mechanicsMaxRhsEvaluations must be an integer.');
end
if isfield(f,'planeCollisionStepMm')
    scalar(f.planeCollisionStepMm,'planeCollisionStepMm',false);
end
if isfield(f,'planeContactGeometry')
    choice(f.planeContactGeometry,{'point-only','rod'},'planeContactGeometry');
    if strcmp(f.planeContactGeometry,'rod')
        assert(strcmp(f.mechanicsModel,'cosserat-shooting')&&strcmp(f.complementaritySolver,'scholtes'), ...
            'rod:InvalidSensorConfig','Rod-plane admissibility requires nonlinear Cosserat mechanics and full MPCC.');
        scalar(f.planeCollisionStepMm,'planeCollisionStepMm',false);
    end
end
if isfield(f,'historyPointInterpolation')
    choice(f.historyPointInterpolation,{'linear','integrated'},'historyPointInterpolation');
    if strcmp(f.historyPointInterpolation,'integrated')
        assert(strcmp(f.mechanicsModel,'cosserat-shooting'), ...
            'rod:InvalidSensorConfig','Integrated history points require the SE(3) Cosserat reconstruction.');
    end
end
if isfield(f,'historyStateMode')
    choice(f.historyStateMode,{'fixed','latent-fbg'},'historyStateMode');
    if strcmp(f.historyStateMode,'latent-fbg')
        assert(strcmp(f.mechanicsModel,'cosserat-shooting')&& ...
            strcmp(f.complementaritySolver,'scholtes'), ...
            'rod:InvalidSensorConfig','Latent FBG history requires nonlinear Cosserat mechanics and full MPCC.');
    end
end
choice(f.solver,{'fmincon','projected'},'solver');
choice(f.complementaritySolver,{'active-set','product-mpcc','scholtes'},'complementaritySolver');
if isfield(f,'mechanicsModel')
    choice(f.mechanicsModel,{'predecessor-linearized','cosserat-shooting'},'mechanicsModel');
end
if strcmp(f.complementaritySolver,'scholtes')
    assert(strcmp(f.solver,'fmincon')&&strcmp(f.subproblemCoordinates,'full'), ...
        'rod:InvalidSensorConfig','Full MPCC requires fmincon and full coordinates.');
    for field={'mechanicsRelativeTolerance','mechanicsMomentToleranceNmm','mpccLengthScaleMm','mpccForceScaleN'}
        scalar(f.(field{1}),field{1},false);
    end
    assert(isnumeric(f.mpccRelaxations)&&isreal(f.mpccRelaxations)&&isvector(f.mpccRelaxations)&& ...
        all(isfinite(f.mpccRelaxations))&&all(f.mpccRelaxations>0)&&all(diff(f.mpccRelaxations)<0), ...
        'rod:InvalidSensorConfig','MPCC relaxations must be positive and strictly decreasing.');
end
choice(f.subproblemCoordinates,{'full','mode-reduced'},'subproblemCoordinates');
if isfield(f,'curvatureObservedAxes')
    assert(isnumeric(f.curvatureObservedAxes)&&isvector(f.curvatureObservedAxes)&& ...
        all(ismember(f.curvatureObservedAxes,1:3))&&all(diff(f.curvatureObservedAxes)>0), ...
        'rod:InvalidSensorConfig','Observed curvature axes must be an ordered subset of 1:3.');
end
for name={'maxEkfIterations','linearizedSolveMaxIter','numFrictionDirs','maxStartCandidates'}
    value=f.(name{1}); scalar(value,name{1},false);
    assert(value==round(value),'rod:InvalidSensorConfig','%s must be an integer.',name{1});
end
assert(f.numFrictionDirs>=4,'rod:InvalidSensorConfig','At least four friction directions are required.');
for name={'historyCurvatureStdPerMm','slipToleranceMm','contactSeparationGateMm'}
    scalar(f.(name{1}),name{1},true);
end
scalar(f.slipConfidenceSigma,'slipConfidenceSigma',false);
scalar(f.convergenceTol,'convergenceTol',false);
for name={'useShapeOnlySeed','useMultiStart','useForceBounds','allowApproximateFallback','showProgress'}
    value=f.(name{1});
    assert((islogical(value)||isnumeric(value))&&isreal(value)&&isscalar(value)&& ...
        any(value==[0 1]),'rod:InvalidSensorConfig','%s must be scalar logical.',name{1});
end
assert(~(f.useMultiStart&&strcmp(f.subproblemCoordinates,'mode-reduced')), ...
    'rod:UnsupportedReducedMultiStart','Multiple starts require full coordinates.');
groups={'priorStd','processStd'};
fields={'planePointMm','normalParam','sMm','normalForceN','betaN','lambda','tipForceN'};
counts=[3 2 1 1 1 1 1];
for g=1:2
    for k=1:numel(fields)
        vector(f.(groups{g}).(fields{k}),counts(k),[groups{g} '.' fields{k}],g==2);
    end
end
vector(f.measurementStd.curvature,1,'measurementStd.curvature',false);
vector(f.measurementStd.planePointMm,3,'measurementStd.planePointMm',false);
vector(f.measurementStd.normalVector,3,'measurementStd.normalVector',false);
if f.useForceBounds
    for name={'normalForceN','betaN','lambda','tipForceN'}
        scalar(f.forceBounds.(name{1}),['forceBounds.' name{1}],false);
    end
end
if ~isempty(f.finiteDifferenceStep)
    vector(f.finiteDifferenceStep,11+f.numFrictionDirs,'finiteDifferenceStep',false);
end
if f.historyCurvatureStdPerMm>0
    assert(cfg.sensing.shapeSmoothing==0,'rod:UnsupportedHistorySmoothing', ...
        'History uncertainty propagation currently requires shapeSmoothing=0.');
end
    function scalar(v,label,allowZero), vector(v,1,label,allowZero); end
    function vector(v,count,label,allowZero)
        assert(isnumeric(v)&&isreal(v)&&numel(v)==count&&all(isfinite(v(:)))&& ...
            all(v(:)>=0)&& (allowZero||all(v(:)>0)), ...
            'rod:InvalidSensorConfig','Invalid %s: expected %d finite %s values.',label,count, ...
            stringChoice(allowZero));
    end
    function label=stringChoice(allowZero)
        if allowZero, label='nonnegative'; else, label='positive'; end
    end
    function choice(v,allowed,label)
        assert((ischar(v)&&isrow(v)||isstring(v)&&isscalar(v))&&any(strcmp(v,allowed)), ...
            'rod:InvalidSensorConfig','Unknown %s.',label);
    end
end
