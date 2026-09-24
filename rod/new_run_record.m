function record = new_run_record()
% Source snapshot at invocation, not a claim about historical input origins.
root=fileparts(fileparts(mfilename('fullpath')));
files=dir(fullfile(root,'rod','*.m')); files(end+1)=dir(fullfile(root,'force.m'));
source=struct('path',{},'sha256',{});
for k=1:numel(files)
    path=fullfile(files(k).folder,files(k).name);
    source(k)=struct('path',strrep(path(numel(root)+2:end),'\','/'),'sha256',file_sha256(path));
end
record=struct('schemaVersion',1,'runId',char(java.util.UUID.randomUUID()), ...
    'state','running','startedAtUtc',char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss'Z'")), ...
    'matlabVersion',version,'platform',computer,'source',source, ...
    'sourceScope','Code present when this operation started; historical input provenance is separate.');
end
