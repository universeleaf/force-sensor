function result = estimate_temporal_window_forces(sensorInput, windowLength, options)
% Complete equilibrium/MPCC window, with each raw observation counted once.
if nargin<2||isempty(windowLength), windowLength=3; end
if nargin<3, options=struct; end
assert(isscalar(windowLength)&&windowLength>=1&&windowLength==round(windowLength), ...
    'rod:InvalidWindowLength','Window length must be a positive integer.');
sensorInput.config=formulation_solver_config(sensorInput.config);
if isfield(options,'frameIndices')&&~isempty(options.frameIndices), indices=options.frameIndices;
else, indices=1:min(windowLength,numel(sensorInput.packet.timeSeconds)); end
assert(all(diff(indices)==1),'rod:NoncontiguousWindow','Use consecutive samples in a physics window.');
if sensorInput.packet.schemaVersion==1
    sensorInput.packet=subset_sensor_packet(sensorInput.packet,indices);
else
    sensorInput.packet=subset_formulation_packet(sensorInput.packet,indices);
end
result=estimate_formulation_window(sensorInput,options);
result.windowLength=numel(indices);
result.quality.temporalWindowOptimized=true;
result.quality.temporalWindowLength=numel(result.windowTimeSeconds);
result.quality.temporalWindowUsesPreviousCosseratBalance=true;
result.quality.observationsCountedOnce=true;
end
