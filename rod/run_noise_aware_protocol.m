function protocol = run_noise_aware_protocol()
%RUN_NOISE_AWARE_PROTOCOL Fixed paired-input regression; preserve all cases.
% Uses the same packets as noise_gated_protocol, not newly sampled noise.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','noise');
if ~isfolder(folder), mkdir(folder); end
protocol=struct('scope','One trajectory, six fixed frames, one zero-noise control and three noise seeds; unresolved friction direction is explicitly uncertified.', ...
    'seeds',[11 11 22 33],'frameIndices',[1 5 8 9 14 18],'independentTrajectories',1, ...
    'state','running','runRecord',new_run_record());
atomic_write_artifact(fullfile(folder,'protocol.json'),'json',protocol);
entries=cell(1,4);
for k=1:4
    try
        report=test_noise_aware_contact(k,protocol.runRecord);
        entries{k}=struct('regressionPassed',true,'error','','report',report);
    catch problem
        report=struct;
        % A solve can fail before it writes a new per-case artifact. Never
        % label a leftover report from an older run as this run's evidence.
        entries{k}=struct('regressionPassed',false,'error',problem.message,'report',report);
    end
    protocol.cases=entries(1:k);
    atomic_write_artifact(fullfile(folder,'protocol.json'),'json',protocol);
end
protocol.allRegressionsPassed=all(cellfun(@(item)item.regressionPassed,entries));
protocol.state='complete';
protocol.runRecord.state='complete';
atomic_write_artifact(fullfile(folder,'protocol.json'),'json',protocol);
assert(protocol.allRegressionsPassed,'Noise regression failed; see out/noise/protocol.json.');
end
