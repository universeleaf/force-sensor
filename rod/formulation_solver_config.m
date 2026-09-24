function cfg = formulation_solver_config(cfg)
%FORMULATION_SOLVER_CONFIG Research solver for Formulation (4), (19)-(23).
% Historical packets retain their predecessor-linearized solver explicitly.
cfg.forceSensor.mechanicsModel='cosserat-shooting';
cfg.forceSensor.planeContactGeometry='rod';
cfg.forceSensor.planeCollisionStepMm=0.5;
cfg.forceSensor.historyPointInterpolation='integrated';
cfg.forceSensor.curvatureObservedAxes=[1 2];
cfg.forceSensor.complementaritySolver='scholtes';
cfg.forceSensor.subproblemCoordinates='full';
cfg.forceSensor.solver='fmincon';
cfg.forceSensor.allowApproximateFallback=false;
cfg.forceSensor.useMultiStart=false;
cfg.forceSensor.useNonlinearShapeSeed=true;
cfg.forceSensor.useObservationScaling=true;
cfg.forceSensor.linearizedSolveMaxIter=100;
cfg.forceSensor.mechanicsRelativeTolerance=2e-7;
cfg.forceSensor.mechanicsMomentToleranceNmm=2e-4;
cfg.forceSensor.mpccRelaxations=[1e-2 1e-4 1e-6 1e-8];
cfg.forceSensor.mpccLengthScaleMm=1;
cfg.forceSensor.mpccForceScaleN=1;
end
