function replay_mu0_failure
% Instrument an isolated copy of OUR adapter; original solver remains unchanged.
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall');a=load(fullfile(folder,'truth.mat'));b=load(fullfile(folder,'comparison_unreduced_attempt.mat'));frames=tsfs.read_input(a.input);fr=frames{4};fr.mu(:)=0;fr.previous.mu(:)=0;fr.CU=eye(numel(fr.u))*1e-8;fr.previous.CU=fr.CU;g=b.runs{1,4}.geometry;g.mu(:)=0;previous=b.runs{2,3};o=b.o;cfg=b.cfg;
probe=fullfile(root,'results','runs','inclined_wall_difficulty_probe');if ~isfolder(probe),mkdir(probe);end
source=fileread(fullfile(root,'matlab','+tsfs','relaxed_solve.m'));source=strrep(source,'function sol=relaxed_solve(','function sol=audit_mu0_adapter(');
old="e=tsfs.evaluate(fr,g,zz,scale,prior,history,o,true);e.relaxedHasHistory=~isempty(history);";
new=strjoin(["try";"e=tsfs.evaluate(fr,g,zz,scale,prior,history,o,true);";"catch failure";"failedPhysicalX=zz.*scale;failureMessage=failure.message;failureIdentifier=failure.identifier;";"save(fullfile(fileparts(mfilename('fullpath')),'failure.mat'),'zz','failedPhysicalX','trace','counts','cacheZ','cacheE','fr','g','scale','prior','history','failureMessage','failureIdentifier','-v7.3');";"rethrow(failure);";"end";"e.relaxedHasHistory=~isempty(history);"],newline);
assert(contains(source,old));source=strrep(source,old,new);fid=fopen(fullfile(probe,'audit_mu0_adapter.m'),'w');fprintf(fid,'%s',source);fclose(fid);addpath(probe);cleanup=onCleanup(@()rmpath(probe));
try
 sol=audit_mu0_adapter(fr,g,previous,o,'fmincon',cfg);save(fullfile(probe,'returned.mat'),'sol','-v7.3');fprintf('Replay returned %s\n',sol.reason);
catch failure
 fprintf('REPRODUCED %s: %s\n',failure.identifier,failure.message);
end
end
