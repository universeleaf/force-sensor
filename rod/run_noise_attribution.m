function report=run_noise_attribution(baselineRunId)
% Controlled simulation diagnostic, NOT a proposed denoising algorithm.
% Replace ONE observed channel with its matching zero-noise sparse packet.
% Keep likelihood/prior/covariance and solver settings fixed. Never supply
% true loads, dense shapes or contact locations to estimate_sensor_forces.
if nargin<1,error('rod:MissingBaseline','Pass the accuracy run ID used for the baseline.');end
root=fileparts(fileparts(mfilename('fullpath')));
archived=fullfile(root,'out','demos','496dc953-eb47-499d-9e38-bd83a92dd98b');
noisyFile=fullfile(archived,'sliding_noisy','results.mat');
cleanFile=fullfile(archived,'sliding_clean','results.mat');
d=load(noisyFile,'sensorInput','truth','scene');clean=load(cleanFile,'sensorInput','truth');
assert(isequal(d.sensorInput.packet.basePose,clean.sensorInput.packet.basePose)&& ...
    isequal(d.sensorInput.packet.previousBasePose,clean.sensorInput.packet.previousBasePose)&& ...
    max(abs(d.truth.u-clean.truth.u),[],'all')<1e-12,'Control packets do not share the same scene.');
input=d.sensorInput;
input.config.forceSensor.planeContactGeometry='rod';
input.config.forceSensor.planeCollisionStepMm=.5;
input.config.forceSensor.useNonlinearShapeSeed=true;
input.config.forceSensor.useObservationScaling=true;
input.config=calibrate_fbg_likelihood(input.config,d.scene.curvatureNoiseStd,1e-6);
input.config.forceSensor.showProgress=true;
record=new_run_record();parent=fullfile(root,'out','accuracy','noise');
folder=fullfile(parent,record.runId);mkdir(folder);
report=struct('state','running','runRecord',record,'baselineRunId',baselineRunId, ...
    'sourceSha256',file_sha256(noisyFile),'cleanControlSha256',file_sha256(cleanFile), ...
    'scope',['Single saved seed, three paired frames. Remove one observation noise source at a time; ', ...
    'all likelihood weights remain noisy-case weights. Interacting causes cannot be added as percentages. ', ...
    'Clean controls are simulation interventions, not deployable estimates.'], ...
    'observedSlipDiagnostic',slipDiagnostic(),'cases',{{}});
publish();
names={'clean_history','clean_current','clean_plane'};
fields={'previousCurvaturePerMm','curvaturePerMm','planePointMm'};
for j=1:numel(names)
    sensorInput=input;sensorInput.packet.(fields{j})=clean.sensorInput.packet.(fields{j});
    caseFolder=fullfile(folder,names{j});mkdir(caseFolder);
    sensorInput.config.forceSensor.frameCheckpointDirectory=caseFolder;
    entry=struct('id',names{j},'changedObservation',fields{j},'completed',false);timer=tic;
    atomic_write_artifact(fullfile(caseFolder,'input.mat'),'mat',struct('sensorInput',sensorInput));
    fprintf('\nNOISE_ATTRIBUTION %s\n',names{j});
    try
        output=estimate_sensor_forces(sensorInput);o=output.ours;
        entry.contactRmseN=rmse(o.contactForceResultant-d.truth.contactForce);
        entry.tipRmseN=rmse(o.tipForce-d.truth.tipForce);
        entry.totalRmseN=rmse(o.totalForceResultant-d.truth.totalForce);
        entry.maxConstraintResidual=max(o.activeConstraintResidual);
        entry.modes=o.complementarityMode;entry.quality=output.quality;
        entry.completed=true;
        atomic_write_artifact(fullfile(caseFolder,'results.mat'),'mat', ...
            struct('sensorInput',sensorInput,'output',output,'truth',d.truth));
        fprintf('NOISE_RESULT %s contact/tip/total %.9g %.9g %.9g N; constraint %.3g\n', ...
            names{j},entry.contactRmseN,entry.tipRmseN,entry.totalRmseN,entry.maxConstraintResidual);
    catch err
        entry.errorIdentifier=err.identifier;entry.error=getReport(err,'extended','hyperlinks','off');
        fprintf(2,'%s\n',entry.error);
    end
    entry.seconds=toc(timer);report.cases{j}=entry;publish();
end
report.state='complete';
if ~all(cellfun(@(v)v.completed,report.cases)),report.state='complete-with-failures';end
report.runRecord.state=report.state;publish();
fprintf('Noise attribution: %s\n',fullfile(folder,'comparison.json'));
    function publish()
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(parent,'comparison.json'),'json',report);
    end
    function result=slipDiagnostic()
        packet=input.packet;ideal=clean.sensorInput.packet;tube=input.tube;nt=size(packet.basePose,3);
        result=struct('scope','Post-hoc reconstruction at true contact arclength, never used to initialize the inverse.', ...
            'trueSlipMm',zeros(1,nt),'observedSlipMm',zeros(1,nt), ...
            'currentTangentialNoiseMm',zeros(1,nt),'historyTangentialNoiseMm',zeros(1,nt), ...
            'observedDirectionCosine',zeros(1,nt));
        for k=1:nt
            n=packet.planeNormal(:,k);P=eye(3)-n*n';s=d.truth.contactS(k);points=zeros(3,4);
            curves={packet.curvaturePerMm(:,:,k),packet.previousCurvaturePerMm(:,:,k), ...
                ideal.curvaturePerMm(:,:,k),ideal.previousCurvaturePerMm(:,:,k)};
            bases={packet.basePose(:,:,k),packet.previousBasePose(:,:,k), ...
                ideal.basePose(:,:,k),ideal.previousBasePose(:,:,k)};
            for a=1:4
                shape=reconstruct_sensor_curvature(tube,curves{a},packet.fbgIdx,bases{a},input.config);
                points(:,a)=sample_integrated_shape(tube,shape,s);
            end
            observed=P*(points(:,1)-points(:,2));reference=P*(points(:,3)-points(:,4));
            result.trueSlipMm(k)=norm(reference);result.observedSlipMm(k)=norm(observed);
            result.currentTangentialNoiseMm(k)=norm(P*(points(:,1)-points(:,3)));
            result.historyTangentialNoiseMm(k)=norm(P*(points(:,2)-points(:,4)));
            result.observedDirectionCosine(k)=observed'*reference/max(eps,norm(observed)*norm(reference));
        end
    end
end
function v=rmse(e),v=sqrt(mean(sum(e.^2,1)));end
