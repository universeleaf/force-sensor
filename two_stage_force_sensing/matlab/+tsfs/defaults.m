function o=defaults
o.geometryMaxIterations=30; o.geometryMaxEvaluations=300;
o.geometryFunctionTolerance=1e-8; o.geometryOptimalityTolerance=1e-6; o.geometryStepTolerance=1e-8;
o.maxIterations=60; o.maxTrials=300; o.qpMaxIterations=200;
o.odeRelativeTolerance=2e-8; o.odeAbsoluteTolerance=2e-10;
o.collisionStepMm=0.5; o.diagnosticStepMm=0.1;
o.rhoNormal=1; o.rhoTangent=1; o.stationarityTolerance=1e-5;
o.momentTolerance=1e-5; o.projectionTolerance=1e-6; o.gapTolerance=1e-5;
o.coneTolerance=1e-6; o.productTolerance=1e-5; o.normalTolerance=1e-8;
o.tangencyWarning=1e-3; o.fbgRmsWarning=4;
o.forcePriorStd=35; o.tipPriorStd=4; o.forceProcessStd=10; o.tipProcessStd=2;
o.referencePeriodSeconds=0.02; o.initialDamping=1e-6;
o.candidateDistanceMm=2; o.endpointMarginMm=1; o.maxContacts=16;
% Explicit reconstruction truncation floors, not substituted FBG uncertainties.
% Midpoint kinematics on a 1 mm grid has O(ds^2) position error.
o.geometryPositionModelStdMm=1e-3; o.geometryTangentModelStd=1e-5;
o.geometryNormalChartBound=0.25; o.initialTrustRadius=1; o.initialMeritPenalty=0.01;
o.useForceTracking=true; o.verbose=false;
end
