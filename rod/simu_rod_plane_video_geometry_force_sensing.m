function results = simu_rod_plane_video_geometry_force_sensing(quickMode, inverseOnly)
%SIMU_ROD_PLANE_VIDEO_GEOMETRY_FORCE_SENSING Image-derived fail.mp4 geometry.
% Approximation: 120 mm straight stem, R=30 mm hook, total L=200 mm,
% ceiling z=160 mm, +z push 20 mm, -x slide 12 mm. Material properties and
% friction are inherited modeling assumptions, not recovered from the video.
if nargin<1, quickMode=false; end
if nargin<2, inverseOnly=false; end
root=fileparts(fileparts(mfilename('fullpath')));
o.scenarioName='video_geometry_upward_image_derived';
o.outputDir=fullfile(root,'out','video');
if quickMode, o.outputDir=fullfile(root,'out','video_tmp'); end
o.exposedLengthMm=200;
o.scalePrecurvature=false;
o.rod.sMm=linspace(0,200,401);
o.rod.intrinsicCurvaturePerMm=zeros(3,401);
o.rod.intrinsicCurvaturePerMm(2,o.rod.sMm>=120)=1/30;
o.basePositionMm=zeros(3,1);
o.baseRotation=eye(3);
o.planePointMm=[0;0;160];
o.planeNormal=[0;0;-1];
o.tipLoadN=zeros(3,1);
o.initialLowFrictionMu=0;
o.frictionMu=0.5;
o.sensing.curvatureInterpolation='intrinsic-delta';
o.numTimeSteps=18;
if quickMode, o.numTimeSteps=6; end
o.forward.motionMode='push-slide';
o.forward.pushDirection=[0;0;1];
o.forward.slideDirection=[-1;0;0];
o.forward.pushDistanceMm=20;
o.forward.slideDistanceMm=12;
o.forward.pushFraction=0.4;
o.diagnostics.runMapCandidateCosts=false;
o.diagnostics.stopAfterInverse=inverseOnly;
o.video.enabled=true;
o.video.frameRate=12;
o.video.durationSeconds=89/12;
o.video.renderFrameCount=89;
o.provenance.sourceVideo='fail.mp4';
o.provenance.geometryStatus='image-derived approximate reconstruction; not original parameter recovery';
o.provenance.assumptions='kb inherited; mu 0 then 0.5; independent tip load zero; noiseless synthetic sensing';
results=run_rod_plane_force_sensing_experiment(quickMode,o);
if inverseOnly
    results=save_result_checkpoint(results,o.outputDir,'inverse_results.mat');
end
end
