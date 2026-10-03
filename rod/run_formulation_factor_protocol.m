function report=run_formulation_factor_protocol(options)
% Full 3-D factors, repeated sensor noise, covariance and same-input baseline.
% All variants receive exactly the same saved sensor packet; truth is loaded
% after inference. No reference estimate/posterior initializes any variant.
if nargin<1,options=struct;end
d=struct('sceneIds',{{'three_contact','spatial_sliding'}},'seeds',[11 23 37], ...
    'noiseLevelsPerMm',[0 2.5e-5 5e-5],'sampleEnvironment',false, ...
    'methods',{{'full','no_temporal','cone_only','no_geometry','no_camera','legacy_partitions'}}, ...
    'quickMode',false,'resume',true,'solverOptions',struct,'outputName','formulation_factors');
if isfield(options,'quickMode')&&isequal(options.quickMode,true)
    d.sceneIds={'two_contact'};d.seeds=11;d.noiseLevelsPerMm=0;
end
names=fieldnames(d);for j=1:numel(names),if ~isfield(options,names{j}),options.(names{j})=d.(names{j});end;end
% MATLAB for-loops iterate columns. Canonical row vectors make a valid
% column-vector experiment specification execute every configured value.
options.sceneIds=options.sceneIds(:)';options.methods=options.methods(:)';
options.seeds=options.seeds(:)';options.noiseLevelsPerMm=options.noiseLevelsPerMm(:)';
assert(iscellstr(options.sceneIds)&&~isempty(options.sceneIds)&& ...
    all(ismember(options.sceneIds,{'two_contact','three_contact','spatial_sliding','tapered_two_contact'}))&& ...
    numel(unique(options.sceneIds))==numel(options.sceneIds)&& ...
    isnumeric(options.seeds)&&isvector(options.seeds)&&~isempty(options.seeds)&&all(isfinite(options.seeds))&& ...
    all(options.seeds>=0)&all(options.seeds==fix(options.seeds))&&numel(unique(options.seeds))==numel(options.seeds)&& ...
    isnumeric(options.noiseLevelsPerMm)&&isvector(options.noiseLevelsPerMm)&&~isempty(options.noiseLevelsPerMm)&& ...
    all(isfinite(options.noiseLevelsPerMm))&&all(options.noiseLevelsPerMm>=0)&& ...
    numel(unique(options.noiseLevelsPerMm))==numel(options.noiseLevelsPerMm)&& ...
    iscellstr(options.methods)&&~isempty(options.methods)&&numel(unique(options.methods))==numel(options.methods)&& ...
    all(ismember(options.methods,{'full','no_temporal','cone_only','no_geometry','no_camera','legacy_partitions','point','gaussian'})), ...
    'rod:InvalidFactorProtocol','Invalid unique scene/seed/noise/method selection.');
for name={'quickMode','resume','sampleEnvironment'}
    v=options.(name{1});assert(isscalar(v)&&ismember(v,[false true]),'rod:InvalidFactorProtocol','Invalid logical protocol switch.');
end
assert(isstruct(options.solverOptions)&&isscalar(options.solverOptions),'rod:InvalidFactorProtocol','solverOptions must be a scalar struct.');
assert(ischar(options.outputName)&&~isempty(regexp(options.outputName,'^[a-z][a-z0-9_]*$','once')), ...
    'rod:InvalidFactorProtocol','outputName must be a simple lowercase result-folder name.');
root=fileparts(fileparts(mfilename('fullpath')));sourceFolder=fullfile(root,'out','formulation_window');
folder=fullfile(root,'out','benchmarks',options.outputName);
if options.quickMode,folder=fullfile(folder,'smoke');end
if ~isfolder(folder),mkdir(folder);end
path=fullfile(folder,'comparison.json');sourcePath=fullfile(sourceFolder,'comparison.json');
source=jsondecode(fileread(sourcePath));
assert(strcmp(source.state,'complete')&&source.failureCount==0,'rod:IncompleteReference','Clean source fixtures must be complete.');
record=new_run_record();
report=struct('state','running','runRecord',record,'options',options,'cases',{{}}, ...
    'referenceComparisonSha256',file_sha256(sourcePath),'failureCount',0,'sotaEstablished',false, ...
    'scope','Independent curvature-noise draws on archived fixed two-state equilibria, full nonlinear window factors and cold starts. Seeds are independent sensor realizations, not independent trajectories. Coverage is conditional on a local contact branch; no global calibration claim.');
if options.resume&&isfile(path)
    previous=jsondecode(fileread(path));
    assert(formulation_resume_matches(previous,report), ...
        'rod:FactorResumeMismatch', ...
        'Resume requires the same options, fixtures, source and mechanics hashes; use a fresh output if any changed.');
    report=previous;if isempty(report.cases),report.cases={};elseif ~iscell(report.cases),report.cases=num2cell(report.cases);end
    report.state='running';report.runRecord.state='running';
end
atomic_write_artifact(path,'json',report);
% Execute historically difficult spatial/no-camera/high-noise cases first.
% This changes scheduling only, not the Cartesian experiment or cold starts.
sceneOrder=[options.sceneIds(strcmp(options.sceneIds,'spatial_sliding')), ...
    options.sceneIds(~strcmp(options.sceneIds,'spatial_sliding'))];
firstMethod=find(strcmp(options.methods,'no_camera'));
methodOrder=options.methods([firstMethod,setdiff(1:numel(options.methods),firstMethod,'stable')]);
for scene=sceneOrder
    id=scene{1};reference=source.cases(strcmp({source.cases.id},id));
    assert(isscalar(reference),'rod:MissingFactorFixture','Missing clean fixture %s.',id);
    baseFolder=fullfile(sourceFolder,reference.artifactFolder);
    assert(strcmp(file_sha256(fullfile(baseFolder,'input.mat')),reference.artifactSha256.input)&& ...
        strcmp(file_sha256(fullfile(baseFolder,'truth.mat')),reference.artifactSha256.truth), ...
        'rod:ReferenceIntegrity','Clean input/truth checksum mismatch.');
    loaded=load(fullfile(baseFolder,'input.mat'),'sensorInput');clean=loaded.sensorInput;
    for sigma=sort(options.noiseLevelsPerMm,'descend')
        for seed=options.seeds
            noiseId='clean';if sigma>0,noiseId=sprintf('noise_%g_micro_per_mm',sigma*1e6);end
            packetId=[id '/' noiseId '/seed_' num2str(seed)];
            dest=fullfile(folder,packetId);if ~isfolder(dest),mkdir(dest);end
            inputPath=fullfile(dest,'input.mat');truthPath=fullfile(dest,'truth.mat');
            sensorInput=resample_formulation_observations(clean,seed,sigma,options.sampleEnvironment);
            if isfile(inputPath)
                old=load(inputPath,'sensorInput');assert(isequaln(old.sensorInput,sensorInput),'rod:FactorInputMismatch','Saved observations changed.');
            else,atomic_write_artifact(inputPath,'mat',struct('sensorInput',sensorInput));end
            if ~isfile(truthPath),copyfile(fullfile(baseFolder,'truth.mat'),truthPath);end
            assert(strcmp(file_sha256(truthPath),reference.artifactSha256.truth),'rod:ReferenceIntegrity','Copied truth changed.');
            inputSha=file_sha256(inputPath);
            for method=methodOrder
                name=method{1};key=[packetId '/' name];
                completed=cellfun(@(v)strcmp(v.id,key)&&v.completed,report.cases);
                if any(completed)
                    entry=report.cases{find(completed,1)};verifyCompleted(entry,folder,inputSha);continue;
                end
                entry=struct('id',key,'sceneId',id,'method',name,'seed',seed,'noiseStdPerMm',sigma, ...
                    'completed',false,'error','','metrics',[],'coverage',[], ...
                    'inputSha256',inputSha,'truthSha256',reference.artifactSha256.truth,'artifactFolder',key);
                try
                    solver=options.solverOptions;solver.showProgress=true;solver.computeCovariance=true;
                    switch name
                        case 'no_temporal',solver.useTemporalPrior=false;
                        case 'cone_only',solver.useFrictionHistory=false;
                        case 'no_geometry',solver.useContactGeometry=false;
                        case 'no_camera',solver.useEnvironmentLikelihood=false;
                        case 'legacy_partitions',solver.contactArcMode='partitioned';
                    end
                    clock=tic;
                    if ismember(name,{'point','gaussian'})
                        obs=formulation_window_observations(sensorInput);
                        candidateOptions=struct('candidateMaxGapMm',5,'candidateMergeMm',10,'maxContacts',4, ...
                            'minArcSeparationMm',.1,'candidateTrackMaxFraction',.25);
                        candidates=formulation_contact_candidates(sensorInput.tube,obs,candidateOptions);
                        estimate=estimate_literature_curvature_baseline(sensorInput,name,numel(candidates.planeIndex));
                    else,estimate=estimate_formulation_window(sensorInput,solver);end
                    entry.wallSeconds=toc(clock);
                    loadedTruth=load(truthPath,'truth');entry.metrics=score_formulation_window(estimate,loadedTruth.truth);
                    entry.coverage=score_formulation_coverage(estimate,loadedTruth.truth,sigma);
                    methodFolder=fullfile(dest,name);if ~isfolder(methodFolder),mkdir(methodFolder);end
                    estimatePath=fullfile(methodFolder,'estimate.mat');csvPath=fullfile(methodFolder,'forces.csv');
                    atomic_write_artifact(estimatePath,'mat',struct('estimate',estimate));write_formulation_window_csv(estimate,csvPath);
                    entry.estimateSha256=file_sha256(estimatePath);entry.csvSha256=file_sha256(csvPath);
                    entry.quality=estimate.quality;entry.solver=estimate.solver;entry.completed=true;
                catch err
                    entry.error=getReport(err,'extended','hyperlinks','off');fprintf('FAILED %s: %s\n',key,err.message);
                end
                existing=find(cellfun(@(v)strcmp(v.id,key),report.cases),1);
                if isempty(existing),report.cases{end+1}=entry;else,report.cases{existing}=entry;end
                report.failureCount=sum(cellfun(@(v)~v.completed,report.cases));atomic_write_artifact(path,'json',report);
                fprintf('Factor protocol %s completed=%d (%d recorded)\n',key,entry.completed,numel(report.cases));
            end
        end
    end
end
expectedCount=numel(options.sceneIds)*numel(options.seeds)*numel(options.noiseLevelsPerMm)*numel(options.methods);
caseIds=cellfun(@(v)v.id,report.cases,'UniformOutput',false);
assert(numel(caseIds)==expectedCount&&numel(unique(caseIds))==expectedCount, ...
    'rod:IncompleteFactorMatrix','Every configured case must be recorded exactly once.');
report.state='complete';if report.failureCount>0,report.state='complete-with-failures';end
report.runRecord.state=report.state;atomic_write_artifact(path,'json',report);
end
function verifyCompleted(entry,folder,inputSha)
assert(strcmp(entry.inputSha256,inputSha)&& ...
    strcmp(file_sha256(fullfile(folder,entry.artifactFolder,'estimate.mat')),entry.estimateSha256)&& ...
    strcmp(file_sha256(fullfile(folder,entry.artifactFolder,'forces.csv')),entry.csvSha256), ...
    'rod:FactorResumeIntegrity','Saved completed artifact was changed.');
end
