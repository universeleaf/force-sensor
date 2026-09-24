function test_fbg_likelihood_calibration()
cfg=struct('forceSensor',struct('measurementStd',struct('curvature',1.5e-4)));
clean=calibrate_fbg_likelihood(cfg,0,1e-6);
noisy=calibrate_fbg_likelihood(cfg,5e-5,1e-6);
assert(clean.forceSensor.measurementStd.curvature==1e-6&&clean.forceSensor.historyCurvatureStdPerMm==0);
assert(abs(noisy.forceSensor.measurementStd.curvature^2-(5e-5)^2-(1e-6)^2)<1e-22);
assert(noisy.forceSensor.historyCurvatureStdPerMm==5e-5);
for values={[-1,1e-6],[NaN,1e-6],[0,0],[1e-5,Inf]}
    v=values{1};rejected=false;
    try,calibrate_fbg_likelihood(cfg,v(1),v(2));catch err,rejected=strcmp(err.identifier,'rod:InvalidSensorConfig');end
    assert(rejected,'Invalid calibration was accepted.');
end
disp('Declared FBG noise/model variance calibration passed.');
end
