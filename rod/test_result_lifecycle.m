function test_result_lifecycle()
% Real success -> failure -> interruption sequence, with corruption rejection.
root=fileparts(fileparts(mfilename('fullpath'))); parent=fullfile(root,'tmp');
folder=tempname(parent);mkdir(folder);
% Preserve the small test directory as evidence; no recursive cleanup.
r=struct('value',1,'runRecord',begin_result_run(folder));
r=save_result_checkpoint(r,folder,'inverse_results.mat');
assert(endsWith(resolve_result_file(folder),'inverse_results.mat'));
r=struct('value',2,'runRecord',begin_result_run(folder));
save_result_checkpoint(r,folder,'failed_results.mat');
path=resolve_result_file(folder);d=load(path,'results');
assert(endsWith(path,'failed_results.mat')&&d.results.value==2,'Stale success selected after failure.');
begin_result_run(folder); assertRejected('rod:IncompleteResult');
r=struct('value',3,'runRecord',begin_result_run(folder));
save_result_checkpoint(r,folder,'inverse_results.mat');
fid=fopen(fullfile(folder,'inverse_results.mat'),'a');fprintf(fid,'corruption');fclose(fid);
assertRejected('rod:ResultIntegrity');
delete(fullfile(folder,'latest_result.json'));
assertRejected('rod:AmbiguousLegacyResult');
disp('Result lifecycle: success/failure/interruption/checksum/legacy ambiguity passed.');
    function assertRejected(identifier)
        rejected=false;
        try,resolve_result_file(folder);catch err,rejected=strcmp(err.identifier,identifier);end
        assert(rejected,'Result lifecycle did not reject %s.',identifier);
    end
end
