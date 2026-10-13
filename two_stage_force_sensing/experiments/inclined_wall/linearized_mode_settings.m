function cfg=linearized_mode_settings
% Opt-in numerical settings. Physics and existing fmincon tolerances unchanged.
cfg=struct('constraintTolerance',1e-6,'modeSearch','first-acceptable','fbgRmsAcceptance',.01,'mechanicsApproximation','successive-linearization','linearization',struct('maxOuterIterations',40,'initialRadius',4,'maxRadius',64,'minRadius',1e-6,'stepTolerance',1e-4));
end
