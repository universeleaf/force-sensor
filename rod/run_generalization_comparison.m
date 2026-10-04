function report=run_generalization_comparison(sceneId,options)
% Independent configurations, paired observation packets and cold-start fits.
% prepareOnly builds and audits clean equilibria without running estimators.
% A separate output ledger prevents overwriting the published fixed fixtures.
if nargin<2,options=struct;end
d=struct('seeds',[11 23 37],'noiseLevelsPerMm',2.5e-5, ...
    'methods',{{'full','no_geometry','point','gaussian'}}, ...
    'sampleEnvironment',true,'resume',true,'prepareOnly',false,'solverOptions',struct);
assert(isstruct(options)&&isscalar(options)&&all(ismember(fieldnames(options),fieldnames(d))), ...
    'rod:InvalidGeneralizationProtocol','Unknown option or nonscalar options.');
for field=fieldnames(d)',if ~isfield(options,field{1}),options.(field{1})=d.(field{1});end;end
ids={'parallel_channel','tapered_channel','wider_channel','serpentine_tip_load','longer_serpentine','spatial_sparse'};
assert(ischar(sceneId)&&ismember(sceneId,ids),'rod:InvalidGeneralizationProtocol','Unknown scene.');
options.seeds=options.seeds(:)';options.methods=options.methods(:)';
options.noiseLevelsPerMm=options.noiseLevelsPerMm(:)';
assert(isnumeric(options.seeds)&&~isempty(options.seeds)&&all(isfinite(options.seeds))&& ...
    all(options.seeds>=0)&&all(options.seeds==fix(options.seeds))&&numel(unique(options.seeds))==numel(options.seeds)&& ...
    isnumeric(options.noiseLevelsPerMm)&&~isempty(options.noiseLevelsPerMm)&&all(isfinite(options.noiseLevelsPerMm))&& ...
    all(options.noiseLevelsPerMm>=0)&&numel(unique(options.noiseLevelsPerMm))==numel(options.noiseLevelsPerMm)&& ...
    iscellstr(options.methods)&&~isempty(options.methods)&&numel(unique(options.methods))==numel(options.methods)&& ...
    all(ismember(options.methods,d.methods))&&isstruct(options.solverOptions)&&isscalar(options.solverOptions), ...
    'rod:InvalidGeneralizationProtocol','Invalid seeds, noise, methods or solver options.');
for field={'sampleEnvironment','resume','prepareOnly'}
    v=options.(field{1});assert(isscalar(v)&&ismember(v,[false true]),'rod:InvalidGeneralizationProtocol','Invalid logical option.');
end
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','benchmarks','generalization','v1',sceneId);
cleanFolder=fullfile(folder,'clean');if ~isfolder(cleanFolder),mkdir(cleanFolder);end
record=new_run_record();recipeSha=file_sha256([mfilename('fullpath') '.m']);
identity=struct('options',struct('sceneId',sceneId),'referenceComparisonSha256',recipeSha,'runRecord',record);
manifestPath=fullfile(cleanFolder,'manifest.json');
if isfile(manifestPath)
    manifest=jsondecode(fileread(manifestPath));
    assert(formulation_resume_matches(manifest,identity),'rod:GeneralizationFixtureMismatch', ...
        'Clean fixture source changed; preserve the old experiment and use a fresh version.');
    for field={'input','truth','geometry'}
        extension='.mat';if strcmp(field{1},'geometry'),extension='.json';end
        assert(strcmp(file_sha256(fullfile(cleanFolder,[field{1} extension])),manifest.artifactSha256.(field{1})), ...
            'rod:GeneralizationFixtureIntegrity','Changed clean fixture.');
    end
else
    [sensorInput,truth,definition]=buildScene(sceneId);
    atomic_write_artifact(fullfile(cleanFolder,'input.mat'),'mat',struct('sensorInput',sensorInput));
    atomic_write_artifact(fullfile(cleanFolder,'truth.mat'),'mat',struct('truth',truth));
    geometry=struct('pMm',truth.p,'contactPointsMm',truth.contactPoints,'contactArcMm',truth.contactS, ...
        'contactForceN',truth.contactForces,'tipForceN',truth.tipForce,'totalForceN',truth.totalForce, ...
        'planePointMm',sensorInput.packet.planePointMm,'planeNormal',sensorInput.packet.planeNormal, ...
        'timeSeconds',sensorInput.packet.timeSeconds,'scope',truth.scope);
    atomic_write_artifact(fullfile(cleanFolder,'geometry.json'),'json',geometry);
    manifest=identity;manifest.definition=definition;
    manifest.artifactSha256=struct('input',file_sha256(fullfile(cleanFolder,'input.mat')), ...
        'truth',file_sha256(fullfile(cleanFolder,'truth.mat')),'geometry',file_sha256(fullfile(cleanFolder,'geometry.json')));
    manifest.state='prepared';atomic_write_artifact(manifestPath,'json',manifest);
end
if options.prepareOnly,report=manifest;return;end
options=rmfield(options,'prepareOnly');options.sceneIds={sceneId};
report=struct('state','running','options',options,'runRecord',record,'cases',{{}}, ...
    'referenceComparisonSha256',file_sha256(manifestPath),'failureCount',0,'sotaEstablished',false, ...
    'scope','New fixed two-state configurations, not new trajectories per seed. Shape and plane measurements are sampled independently within each packet. All methods read identical bytes; literature adaptations omit environment/time factors and are not official implementations. Spatial sparse is a sensor-density control on the existing 3-D sliding trajectory. All unfavorable and nonpositive exits are retained.');
path=fullfile(folder,'comparison.json');
if options.resume&&isfile(path)
    previous=jsondecode(fileread(path));assert(formulation_resume_matches(previous,report), ...
        'rod:GeneralizationResumeMismatch','Resume options, source or fixtures changed.');
    report=previous;if isempty(report.cases),report.cases={};elseif ~iscell(report.cases),report.cases=num2cell(report.cases);end
    report.state='running';report.runRecord.state='running';
end
atomic_write_artifact(path,'json',report);
loaded=load(fullfile(cleanFolder,'input.mat'),'sensorInput');clean=loaded.sensorInput;
for sigma=options.noiseLevelsPerMm
    for seed=options.seeds
        packetId=sprintf('noise_%g_micro_per_mm/seed_%d',sigma*1e6,seed);
        dest=fullfile(folder,packetId);if ~isfolder(dest),mkdir(dest);end
        inputPath=fullfile(dest,'input.mat');truthPath=fullfile(dest,'truth.mat');
        sensorInput=resample_formulation_observations(clean,seed,sigma,options.sampleEnvironment);
        if isfile(inputPath)
            saved=load(inputPath,'sensorInput');assert(isequaln(saved.sensorInput,sensorInput), ...
                'rod:GeneralizationInputMismatch','Saved observations changed.');
        else,atomic_write_artifact(inputPath,'mat',struct('sensorInput',sensorInput));end
        if ~isfile(truthPath),copyfile(fullfile(cleanFolder,'truth.mat'),truthPath);end
        assert(strcmp(file_sha256(truthPath),manifest.artifactSha256.truth),'rod:GeneralizationFixtureIntegrity','Copied truth changed.');
        inputSha=file_sha256(inputPath);
        for method=options.methods
            name=method{1};key=[packetId '/' name];
            done=cellfun(@(v)strcmp(v.id,key)&&v.completed,report.cases);
            if any(done)
                entry=report.cases{find(done,1)};
                assert(strcmp(entry.inputSha256,inputSha)&&strcmp(entry.truthSha256,manifest.artifactSha256.truth)&& ...
                    strcmp(file_sha256(fullfile(folder,key,'estimate.mat')),entry.estimateSha256)&& ...
                    strcmp(file_sha256(fullfile(folder,key,'forces.csv')),entry.csvSha256), ...
                    'rod:GeneralizationResumeIntegrity','Completed artifacts changed.');continue;
            end
            entry=struct('id',key,'sceneId',sceneId,'method',name,'seed',seed,'noiseStdPerMm',sigma, ...
                'completed',false,'error','','metrics',[],'coverage',[], ...
                'inputSha256',inputSha,'truthSha256',manifest.artifactSha256.truth,'artifactFolder',key);
            clock=tic;
            try
                solver=options.solverOptions;solver.showProgress=false;solver.computeCovariance=true;
                if strcmp(name,'no_geometry'),solver.useContactGeometry=false;end
                if ismember(name,{'point','gaussian'})
                    obs=formulation_window_observations(sensorInput);
                    candidateOptions=struct('candidateMaxGapMm',5,'candidateMergeMm',10,'maxContacts',4, ...
                        'minArcSeparationMm',.1,'candidateTrackMaxFraction',.25);
                    candidates=formulation_contact_candidates(sensorInput.tube,obs,candidateOptions);
                    estimate=estimate_literature_curvature_baseline(sensorInput,name,numel(candidates.planeIndex));
                else,estimate=estimate_formulation_window(sensorInput,solver);end
                entry.wallSeconds=toc(clock);
                loadedTruth=load(truthPath,'truth');
                entry.metrics=score_formulation_window(estimate,loadedTruth.truth);
                entry.coverage=score_formulation_coverage(estimate,loadedTruth.truth,sigma);
                methodFolder=fullfile(folder,key);if ~isfolder(methodFolder),mkdir(methodFolder);end
                estimatePath=fullfile(methodFolder,'estimate.mat');csvPath=fullfile(methodFolder,'forces.csv');
                atomic_write_artifact(estimatePath,'mat',struct('estimate',estimate));write_formulation_window_csv(estimate,csvPath);
                entry.estimateSha256=file_sha256(estimatePath);entry.csvSha256=file_sha256(csvPath);
                entry.quality=estimate.quality;entry.solver=estimate.solver;entry.completed=true;
            catch err
                entry.wallSeconds=toc(clock);entry.error=getReport(err,'extended','hyperlinks','off');
                fprintf('FAILED %s/%s: %s\n',sceneId,key,err.message);
            end
            old=find(cellfun(@(v)strcmp(v.id,key),report.cases),1);
            if isempty(old),report.cases{end+1}=entry;else,report.cases{old}=entry;end
            report.failureCount=sum(cellfun(@(v)~v.completed,report.cases));atomic_write_artifact(path,'json',report);
            fprintf('Generalization %s/%s completed=%d seconds=%.1f recorded=%d\n',sceneId,key,entry.completed,entry.wallSeconds,numel(report.cases));
        end
    end
end
expected=numel(options.seeds)*numel(options.noiseLevelsPerMm)*numel(options.methods);
keys=cellfun(@(v)v.id,report.cases,'UniformOutput',false);
assert(numel(keys)==expected&&numel(unique(keys))==expected,'rod:IncompleteGeneralizationMatrix','Missing or duplicate configured cases.');
report.state='complete';if report.failureCount>0,report.state='complete-with-failures';end
report.runRecord.state=report.state;atomic_write_artifact(path,'json',report);
end
function [input,truth,definition]=buildScene(id)
scenes=multi_contact_demo_scenes();scene=scenes(1);scene.baseMotionMm=[-.6 .6];
switch id
    case 'tapered_channel',scene=scenes(2);scene.baseMotionMm=[-.6 .6];
    case 'wider_channel',scene.planePointXZ(1,:)=[-11 11];scene.tipForceXZ=[.15;-.10];
    case 'serpentine_tip_load',scene=scenes(3);scene.baseMotionMm=[-.6 .6];scene.tipForceXZ=[.25;-.15];
    case 'longer_serpentine'
        scene=scenes(3);scale=240/scene.lengthMm;
        scene.lengthMm=scene.lengthMm*scale;scene.segmentLengthMm=scene.segmentLengthMm*scale;
        scene.intrinsicCurvature=scene.intrinsicCurvature/scale;scene.planePointXZ=scene.planePointXZ*scale;
        scene.baseMotionMm=[-.6 .6]*scale;scene.tipForceXZ=scene.tipForceXZ/scale^2;
    case 'spatial_sparse'
        [input,truth]=build_spatial_friction_packet(0,811);
        keep=round(linspace(1,numel(input.packet.sFbgMm),12));
        input.packet.sFbgMm=input.packet.sFbgMm(keep);input.packet.curvaturePerMm=input.packet.curvaturePerMm(:,keep,:);
        definition=struct('id',id,'lengthMm',140,'shapeObservationCount',12,'frictionMu',.03, ...
            'baseTranslationMm',[0 .5 0],'tipForceN',[.1 .3 -.08], ...
            'scope','Existing intrinsically spatial sliding truth; only observation density changes.');return;
end
scene.id=id;scene.curvatureNoiseStd=0;
[input,truth]=build_formulation_multi_packet(scene,[1 2]);
definition=scene;definition.shapeObservationCount=numel(input.packet.sFbgMm);
definition.scope='Independent planar shooting equilibrium embedded in 3-D; two fully solved base states specified in baseMotionMm. Longer serpentine scales all geometry consistently.';
end
