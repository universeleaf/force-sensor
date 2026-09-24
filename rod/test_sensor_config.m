function test_sensor_config()
root=fileparts(fileparts(mfilename('fullpath')));
d=load(fullfile(root,'out','stage1','video_seeded','sensor_input.mat'),'sensorInput');
cfg=d.sensorInput.config; cfg.forceSensor.historyCurvatureStdPerMm=0;
cfg.forceSensor.slipConfidenceSigma=3;cfg.forceSensor.contactSeparationGateMm=0.5;
validate_sensor_config(cfg);
bad=cfg;bad.forceSensor.measurementStd.curvature=-1;reject(bad);
bad=cfg;bad.forceSensor.priorStd.tipForceN=0;reject(bad);
bad=cfg;bad.forceSensor.maxEkfIterations=0;reject(bad);
bad=cfg;bad.forceSensor.linearizedSolveMaxIter=1.5;reject(bad);
bad=cfg;bad.forceSensor.solver='typo';reject(bad);
bad=cfg;bad.forceSensor.processStd.sMm=NaN;reject(bad);
bad=cfg;bad.forceSensor.finiteDifferenceStep=0;reject(bad);
bad=cfg;bad.forceSensor.planeCollisionStepMm=0;reject(bad);
bad=cfg;bad.forceSensor.planeContactGeometry='typo';reject(bad);
bad=cfg;bad.forceSensor.mechanicsMaxRhsEvaluations=0;reject(bad);
bad=cfg;bad.forceSensor.mechanicsMaxRhsEvaluations=1.5;reject(bad);
bad=cfg;bad.forceSensor.useNonlinearShapeSeed=2;reject(bad);
bad=cfg;bad.forceSensor.useObservationScaling=NaN;reject(bad);
bad=cfg;bad.forceSensor.nonlinearFiniteDifferenceStep=0;reject(bad);
bad=cfg;bad.forceSensor.useNonlinearShapeSeed=true;
bad.forceSensor.mechanicsModel='predecessor-linearized';reject(bad);
good=cfg;good.forceSensor.processStd.sMm=0;validate_sensor_config(good);
disp('Solver budgets, likelihood/prior/process weights and configuration checks passed.');
    function reject(config)
        rejected=false;
        try,validate_sensor_config(config);catch err,rejected=strcmp(err.identifier,'rod:InvalidSensorConfig');end
        assert(rejected,'Invalid numerical configuration was accepted.');
    end
end
