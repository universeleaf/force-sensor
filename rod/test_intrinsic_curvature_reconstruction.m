function test_intrinsic_curvature_reconstruction()
%TEST_INTRINSIC_CURVATURE_RECONSTRUCTION A calibrated free rod stays free.
root=fileparts(fileparts(mfilename('fullpath')));
o.outputDir=fullfile(root,'out','curvature_tmp');
o.exposedLengthMm=200; o.scalePrecurvature=false;
o.rod.sMm=linspace(0,200,401);
o.rod.intrinsicCurvaturePerMm=zeros(3,401);
o.rod.intrinsicCurvaturePerMm(2,o.rod.sMm>=120)=1/30;
o.basePositionMm=zeros(3,1); o.baseRotation=eye(3);
o.planePointMm=[0;0;300]; o.planeNormal=[0;0;-1];
o.forward.pushDirection=[0;0;1];
o.forward.pushDistanceMm=0; o.forward.slideDistanceMm=0;
o.numTimeSteps=2; o.video.enabled=false;
o.diagnostics.stopAfterTruthConsistency=true;
o.diagnostics.runMapCandidateCosts=false;
o.sensing.curvatureInterpolation='absolute';
a=run_rod_plane_force_sensing_experiment(false,o);
absoluteError=max(abs(a.measurements.p-a.forward.p),[],'all');
assert(absoluteError>0.5,'The intended sparse-curvature stress case was not exercised.');
o.sensing.curvatureInterpolation='intrinsic-delta';
b=run_rod_plane_force_sensing_experiment(false,o);
calibratedError=max(abs(b.measurements.p-b.forward.p),[],'all');
assert(calibratedError<1e-10,'No-load calibration introduced artificial shape deformation.');
assert(isequal(a.measurements.uSparse,b.measurements.uSparse), ...
    'Calibration changed the actual sparse measurements.');
fprintf('Free-shape interpolation max coordinate error: absolute %.6g mm; calibrated %.6g mm.\n',absoluteError,calibratedError);
end
