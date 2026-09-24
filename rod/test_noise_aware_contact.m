function report = test_noise_aware_contact(caseIndex,runRecord)
%TEST_NOISE_AWARE_CONTACT Regress the fixed paired-curvature noise failure.
% Calibration and observations enter the estimator; force truth is used
% exclusively below for offline scoring.
if nargin<1, caseIndex=2; end
if nargin<2,runRecord=new_run_record();end
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','stage1',sprintf('video_noise_gated_%02d',caseIndex));
d=load(fullfile(folder,'sensor_input.mat'),'sensorInput'); input=d.sensorInput;
file=resolve_result_file(folder);
d=load(file,'results'); reference=d.results;
input.config.forceSensor.historyCurvatureStdPerMm=5e-5*(caseIndex>1);
input.config.forceSensor.showProgress=false;
output=estimate_sensor_forces(input);
o=output.ours; f=reference.forward;
report=struct('caseIndex',caseIndex,'modes',{o.complementarityMode}, ...
    'contactRmseN',sqrt(mean(sum((o.contactForceResultant-f.contactForceResultant).^2,1))), ...
    'tipRmseN',sqrt(mean(sum((o.tipForce-f.tipLoad).^2,1))), ...
    'totalRmseN',sqrt(mean(sum((o.totalForceResultant-f.totalForceResultant).^2,1))), ...
    'maxShapeRmseMm',max(o.shapeRmseMm),'quality',output.quality);
report.before=struct('contactRmseN',sqrt(mean(sum((reference.ours.contactForceResultant-f.contactForceResultant).^2,1))), ...
    'tipRmseN',sqrt(mean(sum((reference.ours.tipForce-f.tipLoad).^2,1))), ...
    'totalRmseN',sqrt(mean(sum((reference.ours.totalForceResultant-f.totalForceResultant).^2,1))));
report.scope='Same saved six-frame sensor packets; cone/contact feasibility and force-accuracy regression. Unresolved modes do not pass full friction-kinematics certification.';
report.fullFrictionKinematicsChecked=all(o.frictionKinematicsEnforced);
report.unresolvedFrames=find(~o.frictionKinematicsEnforced);
report.maxActiveConstraintResidual=max(o.activeConstraintResidual);
report.maxRawFrictionComplementarity=max(o.frictionComplementarity);
report.minRawFrictionWmm=min(o.frictionWMin);
count=input.config.forceSensor.numFrictionDirs;
coneSlack=input.packet.frictionMu(:)'.*o.state(7,:)-sum(o.state(8:7+count,:),1);
report.minConeSlackN=min(coneSlack);
report.runRecord=runRecord;
destination=fullfile(root,'out','noise');
if ~isfolder(destination), mkdir(destination); end
atomic_write_artifact(fullfile(destination,sprintf('case%02d.mat',caseIndex)),'mat', ...
    struct('input',input,'output',output,'report',report));
atomic_write_artifact(fullfile(destination,sprintf('case%02d.json',caseIndex)),'json',report);
disp(report);
assert(all(isfinite(o.state),'all') && max(o.frictionConeViolation)<1e-7, ...
    'Noise handling violated finite-state or friction-cone constraints.');
assert(max(o.normalComplementarity)<1e-3 && min(o.gap)>-1e-5, ...
    'Noise handling violated unilateral contact.');
assert(min(coneSlack)>-1e-7 && all(o.state(7:8+count,:)>=-1e-8,'all'), ...
    'Recomputed polyhedral cone or nonnegative contact variables failed.');
if caseIndex>1
    assert(strcmp(o.complementarityMode{4},'unresolved-contact'), ...
        'A sub-resolution paired displacement was forced into a slip mode.');
    assert(output.quality.requiresReview(4) && ~output.quality.frictionDirectionResolved(4), ...
        'Unresolved friction direction was incorrectly certified.');
    assert(report.totalRmseN<0.5 && report.contactRmseN<1 && report.tipRmseN<1, ...
        'Noise-aware contact did not recover useful force accuracy on the fixed regression.');
else
    assert(report.totalRmseN<0.05,'Noiseless accuracy regressed.');
end
end
