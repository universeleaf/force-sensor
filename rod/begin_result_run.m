function record = begin_result_run(folder)
% A started/interrupted run must not silently fall back to older successes.
if ~isfolder(folder),mkdir(folder);end
record=new_run_record();
atomic_write_artifact(fullfile(folder,'latest_result.json'),'json',record);
end
