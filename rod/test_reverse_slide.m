function results = test_reverse_slide()
%TEST_REVERSE_SLIDE Reverse the commanded slip and halve the internal step.
root=fileparts(fileparts(mfilename('fullpath')));
o.outputDir=fullfile(root,'out','reverse_tmp');
o.scenarioName='reverse_slip_regression';
o.planePointMm=[0;0;50]; o.planeNormal=[0;0;-1];
o.forward.pushDistanceMm=40; o.forward.slideDistanceMm=2;
o.forward.pushDirection=[0;0;1]; o.forward.slideDirection=[-1;0;0];
o.forward.maxSlideStepMm=0.01; o.forward.pushFraction=0.4;
o.numTimeSteps=6; o.video.enabled=false;
% This trajectory crosses stick/slip close to a saved frame. Allow the
% nonlinear iteration to converge instead of weakening the shape audit.
o.forceSensor.maxEkfIterations=6;
o.forceSensor.linearizedSolveMaxIter=60;
o.diagnostics.runMapCandidateCosts=false; o.diagnostics.stopAfterInverse=true;
results=run_rod_plane_force_sensing_experiment(false,o);
assert(results.forward.frictionForceResultant(1,end)>0.01, 'Forward friction failed to reverse.');
assert(results.ours.frictionForceResultant(1,end)>0, 'Estimated friction failed to reverse.');
assert(strcmp(results.ours.complementarityMode{end},'sliding-contact'), 'Half-sized sliding steps were misclassified.');
disp('Reverse-slip, half-step strict inverse regression passed.');
end
