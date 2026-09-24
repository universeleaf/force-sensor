function [path,record] = resolve_result_file(folder)
% Prefer the indexed current run; legacy folders must be unambiguous.
index=fullfile(folder,'latest_result.json'); record=struct;
if isfile(index)
    record=jsondecode(fileread(index));
    assert(isfield(record,'state')&&any(strcmp(record.state,{'complete','validation_failed'})), ...
        'rod:IncompleteResult','Latest run is incomplete; inspect latest_result.json.');
    assert(isfield(record,'resultFile')&&any(strcmp(record.resultFile, ...
        {'results.mat','inverse_results.mat','failed_results.mat'}))&&isfield(record,'sha256'), ...
        'rod:InvalidResultIndex','Invalid result index.');
    path=fullfile(folder,record.resultFile);
    assert(isfile(path)&&strcmp(file_sha256(path),record.sha256), ...
        'rod:ResultIntegrity','Latest result is missing or no longer matches its checksum.');
else
    names={'inverse_results.mat','failed_results.mat','results.mat'};
    present=cellfun(@(name)isfile(fullfile(folder,name)),names);
    assert(sum(present)==1,'rod:AmbiguousLegacyResult', ...
        'Expected one legacy result; found %d. Select a historical file explicitly.',sum(present));
    path=fullfile(folder,names{present});
end
end
