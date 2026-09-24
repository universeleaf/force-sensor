function protocol = run_sensor_noise_protocol(useShapeOnlySeed,outputTag)
%RUN_SENSOR_NOISE_PROTOCOL Fixed, small noise stress test; retain failures.
% Four paired-input replays of the SAME trajectory at six preselected output
% frames. These are noise realizations, not four independent trajectories.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1, useShapeOnlySeed=true; end
if nargin<2, outputTag='aware'; end
assert(isempty(outputTag)||~isempty(regexp(outputTag,'^[a-z0-9_]+$','once')),'Invalid output tag.');
validate_lcp_dependency(root);
d=load(fullfile(root,'out','video','inverse_results.mat'),'results'); original=d.results;
ids=[1 5 8 9 14 18];
seeds=[11 11 22 33]; sigmas=[0 5e-5 5e-5 5e-5];
protocol=struct('state','running','runRecord',new_run_record(), ...
    'frameIndices',ids,'independentTrajectories',1, ...
    'scope','One noiseless control and three fixed noise seeds; six selected output frames, 20 ms paired history, offline planar simulation.');
protocol.useShapeOnlySeed=useShapeOnlySeed;
prefix='video_noise_'; protocolName='noise_protocol.json';
if useShapeOnlySeed
    prefix='video_noise_seeded_'; protocolName='noise_seeded_protocol.json';
end
if ~isempty(outputTag)
    prefix=['video_noise_' outputTag '_']; protocolName=['noise_' outputTag '_protocol.json'];
end
entries=cell(1,numel(seeds));
protocolFolder=fullfile(root,'out','stage1');
if ~isfolder(protocolFolder),mkdir(protocolFolder);end
atomic_write_artifact(fullfile(protocolFolder,protocolName),'json',protocol);
for j=1:numel(seeds)
    cfg=original.config; cfg.randomSeed=seeds(j);
    cfg.scenarioName=sprintf('%s%02d',prefix,j);
    cfg.outputDir=fullfile(root,'out','stage1',cfg.scenarioName);
    if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
    runRecord=begin_result_run(cfg.outputDir);
    cfg.video.enabled=false;
    cfg.sensing.historySource='sparse-paired';
    cfg.sensing.curvatureInterpolation='intrinsic-delta';
    cfg.sensing.samplePeriodSeconds=0.02;
    cfg.sensing.curvatureNoiseStd=sigmas(j);
    cfg.forceSensor.historyCurvatureStdPerMm=sigmas(j);
    cfg.sensing.planeOffsetNoiseStdMm=0.1*(j>1);
    cfg.sensing.planeNormalNoiseStdDeg=0.2*(j>1);
    cfg.forceSensor.subproblemCoordinates='mode-reduced';
    cfg.forceSensor.useShapeOnlySeed=useShapeOnlySeed;
    tube=make_experiment_tube(cfg);
    packet=simulate_sensor_packet(tube,original.forward,cfg);
    originalFrameCount=numel(original.forward.betaMm);
    assert(max(ids)<=originalFrameCount,'Protocol needs at least 18 original output frames.');
    packet=subset(packet,ids,originalFrameCount,{'fbgIdx','sFbgMm'});
    cfg.forceSensor.fbgIdx=packet.fbgIdx;
    cfg.forceSensor.normalReference=packet.planeNormal(:,1);
    sensorInput=struct('tube',tube,'packet',packet,'config',sensor_estimator_config(cfg));
    atomic_write_artifact(fullfile(cfg.outputDir,'sensor_input.mat'),'mat',struct('sensorInput',sensorInput));
    item=struct('name',cfg.scenarioName,'seed',seeds(j),'curvatureNoisePerMm',sigmas(j), ...
        'planeOffsetNoiseMm',cfg.sensing.planeOffsetNoiseStdMm, ...
        'planeNormalNoiseDeg',cfg.sensing.planeNormalNoiseStdDeg);
    try
        estimate=run_rod_plane_force_sensing_experiment('replay-sensors',sensorInput);
        results=struct('config',cfg,'setup',original.setup, ...
            'forward',subset(original.forward,ids,originalFrameCount,{'s'}), ...
            'measurements',estimate.measurements,'ours',estimate.ours,'runRecord',runRecord);
        % No truth residual was recomputed during the sensor-only replay.
        results.truthConsistency=struct('maxInequalityViolation',nan(1,numel(ids)), ...
            'maxEqualityResidual',nan(1,numel(ids)));
        results.metrics.ours.finalRelativeErrorPct=100*norm( ...
            results.ours.totalForceResultant(:,end)-results.forward.totalForceResultant(:,end))/ ...
            max(norm(results.forward.totalForceResultant(:,end)),eps);
        try
            results.validation=validate_rod_plane_displacement_results(results);
            item.validationPassed=true; item.validationError='';
            filename='inverse_results.mat';
        catch validationError
            item.validationPassed=false; item.validationError=validationError.message;
            results.validation=struct('passed',false,'error',validationError.message);
            filename='failed_results.mat';
        end
        results=save_result_checkpoint(results,cfg.outputDir,filename);
        export_inverse_audit(results);
        item.comparison=audit_sensor_stage1(cfg.outputDir,filename);
    catch solveError
        item.validationPassed=false; item.validationError=solveError.message;
        item.exception=getReport(solveError,'extended','hyperlinks','off');
    end
    entries{j}=item;
    protocol.cases=entries(1:j);
    atomic_write_artifact(fullfile(protocolFolder,protocolName),'json',protocol);
end
protocol.state='complete';protocol.runRecord.state='complete';
protocol.allStrictValidationsPassed=all(cellfun(@(c)c.validationPassed,entries));
atomic_write_artifact(fullfile(protocolFolder,protocolName),'json',protocol);
end

function out=subset(in,ids,nt,keep)
out=in; names=fieldnames(in);
for k=1:numel(names)
    name=names{k}; if ismember(name,keep), continue; end
    a=in.(name); sz=size(a);
    if (isnumeric(a)||iscell(a)||islogical(a)) && ~isscalar(a) && sz(end)==nt
        indices=repmat({':'},1,ndims(a)); indices{end}=ids;
        out.(name)=a(indices{:});
    end
end
end
