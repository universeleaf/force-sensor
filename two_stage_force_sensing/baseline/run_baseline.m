function report=run_baseline(name,options)
% Call unchanged upstream estimators. Truth is used only after estimation.
% options.profile enables a separately labelled, timing-contaminated run.
if nargin<1,name='s_channel_two_contact';end
if nargin<2,options=struct;end
if ~isfield(options,'profile'),options.profile=false;end
if ~isfield(options,'frames'),options.frames=[];end
oldPath=path;restorePath=onCleanup(@()path(oldPath));
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'baseline');if ~isfolder(folder),mkdir(folder);end
profileFolder=folder;if options.profile,profileFolder=fullfile(folder,'profiled');if ~isfolder(profileFolder),mkdir(profileFolder);end,end
metadata=struct('scenario',name,'matlab',version,'profiled',options.profile,'created',char(datetime('now')), ...
 'dependencies',jsondecode(fileread(fullfile(root,'dependencies.json'))),'actualLcpRevision',paths.lcpRevision, ...
 'sourcePaths',struct('single',which('estimate_sensor_forces'),'planar',which('estimate_planar_multi_contact'),'window',which('estimate_formulation_window')), ...
 'metricNote','No copied or instrumented algorithms. Unavailable internal counters are not inferred. Profiling timings are not benchmark timings.');
metadata.actualSources=baseline_provenance(paths.baseline,paths.lcp);
if options.profile,profile clear;profile on;stopProfile=onCleanup(@()profile('off'));end
switch name
 case 'sliding_clean'
  inputFile=fullfile(folder,'sliding_input.mat');if ~isfile(inputFile),prepare_inputs;end
  d=load(inputFile);sensorInput=d.sensorInput;selection=options.frames;if isempty(selection),selection=1:numel(sensorInput.packet.timeSeconds);end
  sensorInput.packet=subset_sensor_packet(sensorInput.packet,selection);
  sensorInput.config.outputDir=profileFolder;
  sensorInput.config.forceSensor.frameCheckpointDirectory=fullfile(profileFolder,'sliding_checkpoints');
  if ~isfolder(sensorInput.config.forceSensor.frameCheckpointDirectory),mkdir(sensorInput.config.forceSensor.frameCheckpointDirectory);end
  timer=tic;output=estimate_sensor_forces(sensorInput);elapsed=toc(timer);o=output.ours;
  report=struct('scenario',name,'elapsedSeconds',elapsed,'frameSeconds',o.frameSeconds, ...
   'contactRmseN',sqrt(mean(sum((o.contactForceResultant-d.truth.contactForce(:,selection)).^2,1))), ...
   'tipRmseN',sqrt(mean(sum((o.tipForce-d.truth.tipForce(:,selection)).^2,1))),'quality',output.quality,'solverTrace',{o.solverTrace},'metadata',metadata);
  save(fullfile(profileFolder,'sliding_results.mat'),'output','sensorInput','elapsed','report','-v7.3');
 case 's_channel_two_contact'
  inputFile=fullfile(folder,'two_contact_input.mat');if ~isfile(inputFile),prepare_inputs;end
  d=load(inputFile);selection=options.frames;if isempty(selection),selection=1:numel(d.sensorInput.frames);end
  estimates=cell(1,numel(selection));entries=estimates;
  for j=1:numel(selection)
   k=selection(j);timer=tic;e=estimate_planar_multi_contact(d.sensorInput.frames{k});elapsed=toc(timer);estimates{j}=e;
   entries{j}=struct('frame',k,'elapsedSeconds',elapsed,'iterations',e.iterations,'exitflag',e.exitflag, ...
    'contactErrorN',vecnorm(e.state.contactForceXZ-d.truth.contactForces([1 3],:,k)), ...
    'tipErrorN',norm(e.state.tipForceXZ-d.truth.tipForce([1 3],k)),'audit',e.audit,'maxScaledResidual',e.maxScaledResidual);
  end
  report=struct('scenario',name,'entries',{entries},'metadata',metadata);
  save(fullfile(profileFolder,'planar_results.mat'),'estimates','entries','selection','report','-v7.3');
 otherwise
  allowed={'two_contact','three_contact','spatial_sliding','spatial_sliding_noisy','free_space'};
  assert(ismember(name,allowed),'tsfs:Scenario','Unsupported baseline scenario.');
  assert(isempty(options.frames),'tsfs:WindowSubset','Window baseline runs its complete recorded window.');
  d=load(fullfile(root,'datasets',name,'input.mat'));t=load(fullfile(root,'datasets',name,'truth.mat'));
  timer=tic;estimate=estimate_formulation_window(d.sensorInput);elapsed=toc(timer);metrics=score_formulation_window(estimate,t.truth);
  report=struct('scenario',name,'elapsedSeconds',elapsed,'metrics',metrics,'solver',estimate.solver,'quality',estimate.quality,'metadata',metadata);
  save(fullfile(profileFolder,[name '_results.mat']),'estimate','elapsed','metrics','report','-v7.3');
end
if options.profile
 profile off;profileInfo=profile('info');save(fullfile(profileFolder,[name '_profile.mat']),'profileInfo','-v7.3');
end
atomic_write_artifact(fullfile(profileFolder,[name '_summary.json']),'json',report);
end
