function results = save_result_checkpoint(results,folder,filename)
% Save the selected result and publish a checksum-bound latest-run index.
assert(any(strcmp(filename,{'results.mat','inverse_results.mat','failed_results.mat'})), ...
    'rod:ArtifactName','Use a standard result filename.');
if ~isfield(results,'runRecord')
    results.runRecord=begin_result_run(folder);
end
record=results.runRecord;
record.state='complete';
if strcmp(filename,'failed_results.mat'),record.state='validation_failed';end
record.finishedAtUtc=char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss'Z'"));
record.resultFile=filename; results.runRecord=record;
path=fullfile(folder,filename);
atomic_write_artifact(path,'mat',struct('results',results));
record.sha256=file_sha256(path);
atomic_write_artifact(fullfile(folder,'latest_result.json'),'json',record);
end
