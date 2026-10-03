function matches=formulation_resume_matches(previous,current)
% Compare checkpoint identity despite JSON struct orientation/field order.
% Only declared protocol vectors are orientation-insensitive; solver values,
% every executed source hash and every external dependency hash stay exact.
matches=false;
if ~isstruct(previous)||~isstruct(current),return;end
required={'options','referenceComparisonSha256','runRecord'};
if ~all(isfield(previous,required))||~all(isfield(current,required)),return;end
try
    oldOptions=protocolOptions(previous.options);newOptions=protocolOptions(current.options);
    oldSource=previous.runRecord.source(:)';newSource=current.runRecord.source(:)';
    oldDependency=previous.runRecord.dependency;newDependency=current.runRecord.dependency;
    oldDependency.files=oldDependency.files(:)';newDependency.files=newDependency.files(:)';
    matches=strcmp(jsonencode(canonical(oldOptions)),jsonencode(canonical(newOptions)))&& ...
        strcmp(previous.referenceComparisonSha256,current.referenceComparisonSha256)&& ...
        strcmp(jsonencode(canonical(oldSource)),jsonencode(canonical(newSource)))&& ...
        strcmp(jsonencode(canonical(oldDependency)),jsonencode(canonical(newDependency)));
catch err
    if ~ismember(err.identifier,{'MATLAB:nonExistentField','MATLAB:structRefFromNonStruct'}),rethrow(err);end
end
end
function options=protocolOptions(options)
for field={'sceneIds','methods','seeds','noiseLevelsPerMm'}
    if isfield(options,field{1}),options.(field{1})=options.(field{1})(:)';end
end
end
function value=canonical(value)
if isstruct(value)
    value=orderfields(value);fields=fieldnames(value);
    for k=1:numel(value)
        for j=1:numel(fields),value(k).(fields{j})=canonical(value(k).(fields{j}));end
    end
elseif iscell(value)
    for k=1:numel(value),value{k}=canonical(value{k});end
end
end
