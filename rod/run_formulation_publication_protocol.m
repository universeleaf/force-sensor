function report=run_formulation_publication_protocol(quickMode)
% One reproducible software workflow: factors, literature, timing, checks.
% Truth is isolated in each inference runner. All baselines reuse the exact
% saved observation bytes from the factor run, not freshly serialized MATs.
if nargin<1,quickMode=false;end
assert(isscalar(quickMode)&&ismember(quickMode,[false true]),'rod:InvalidPublicationProtocol','Invalid quickMode.');
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'out','benchmarks','publication');
if quickMode,folder=fullfile(folder,'smoke');end
if ~isfolder(folder),mkdir(folder);end
path=fullfile(folder,'completion.json');report=struct('state','running','runRecord',new_run_record(), ...
    'quickMode',quickMode,'completedSteps',{{}},'failure','','sotaEstablished',false);
atomic_write_artifact(path,'json',report);
try
    factors=run_formulation_factor_protocol(struct('quickMode',quickMode));
    assert(factors.failureCount==0,'rod:IncompletePublicationProtocol','Factor run contains exceptions.');
    factorFolder=fullfile(root,'out','benchmarks','formulation_factors');
    baselineFolder=fullfile(root,'out','benchmarks','formulation_literature');
    if quickMode,factorFolder=fullfile(factorFolder,'smoke');baselineFolder=fullfile(baselineFolder,'smoke');end
    packetIds=unique(cellfun(@(v)fileparts(v.artifactFolder),factors.cases,'UniformOutput',false));
    for j=1:numel(packetIds)
        dest=fullfile(baselineFolder,packetIds{j});if ~isfolder(dest),mkdir(dest);end
        for name={'input.mat','truth.mat'}
            sourcePath=fullfile(factorFolder,packetIds{j},name{1});destPath=fullfile(dest,name{1});
            if ~isfile(destPath),copyfile(sourcePath,destPath);end
            assert(strcmp(file_sha256(sourcePath),file_sha256(destPath)), ...
                'rod:PublicationInputMismatch','Baseline observation/truth bytes differ.');
        end
    end
    report.completedSteps{end+1}=struct('name','factors','path',relative(fullfile(factorFolder,'comparison.json')), ...
        'sha256',file_sha256(fullfile(factorFolder,'comparison.json')),'caseCount',numel(factors.cases));
    atomic_write_artifact(path,'json',report);
    baselines=run_formulation_factor_protocol(struct('quickMode',quickMode, ...
        'methods',{{'point','gaussian'}},'outputName','formulation_literature'));
    assert(baselines.failureCount==0,'rod:IncompletePublicationProtocol','Literature run contains exceptions.');
    report.completedSteps{end+1}=struct('name','literature','path',relative(fullfile(baselineFolder,'comparison.json')), ...
        'sha256',file_sha256(fullfile(baselineFolder,'comparison.json')),'caseCount',numel(baselines.cases));
    atomic_write_artifact(path,'json',report);
    derivatives=run_formulation_derivative_benchmark;
    derivativePath=fullfile(root,'out','benchmarks','formulation_derivatives','comparison.json');
    report.completedSteps{end+1}=struct('name','derivatives','path',relative(derivativePath), ...
        'sha256',file_sha256(derivativePath),'caseCount',numel(derivatives.cases));
    atomic_write_artifact(path,'json',report);
    checks=run_project_checks(~quickMode);assert(checks.allPassed,'rod:IncompletePublicationProtocol','Engineering checks failed.');
    checkPath=fullfile(root,'out','completion','project_checks.json');
    report.completedSteps{end+1}=struct('name','engineering','path',relative(checkPath), ...
        'sha256',file_sha256(checkPath),'caseCount',numel(checks.checks));
    report.state='complete';report.runRecord.state='complete';atomic_write_artifact(path,'json',report);
catch err
    report.state='failed';report.runRecord.state='failed';report.failure=getReport(err,'extended','hyperlinks','off');
    atomic_write_artifact(path,'json',report);rethrow(err);
end
    function value=relative(file)
        value=strrep(file(numel(root)+2:end),'\','/');
    end
end
