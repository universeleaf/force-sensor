function audit_inclined_seeds
% Controlled initialization diagnostic, same frozen geometry/history/NLP.
% Uses truth ONLY as a diagnostic seed; never reported as estimator performance.
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall');a=load(fullfile(folder,'truth.mat'));b=load(fullfile(folder,'comparison_unreduced_attempt.mat'));t=a.truth;frames=tsfs.read_input(a.input);o=b.o;pub=fullfile(root,'results','published','inclined_wall_difficulty_20261008');probe=fullfile(root,'results','runs','inclined_wall_difficulty_probe');if ~isfolder(probe),mkdir(probe);end
source=fileread(fullfile(root,'matlab','+tsfs','relaxed_solve.m'));source=strrep(source,'function sol=relaxed_solve(','function sol=audit_seed_adapter(');source=strrep(source,'nx=numel(z);C=numel(g.s);','nx=numel(z);C=numel(g.s);z=cfg.auditInitialZ;');fid=fopen(fullfile(probe,'audit_seed_adapter.m'),'w');fprintf(fid,'%s',source);fclose(fid);addpath(probe);cleanup=onCleanup(@()rmpath(probe));rows=[];runs=cell(1,3);
for k=[6 15 34]
 fr=frames{k};fr.CU=eye(numel(fr.u))*1e-8;fr.previous.CU=fr.CU;out=b.runs{1,k};old=out.mechanics;g=out.geometry;scale=old.scale;A=[g.normal g.B];ft=t.tipForce(:,k);fc=t.contactForce(:,k);pc=interp1(t.sMm,t.p(:,:,k)',g.s,'pchip')';m=cross(t.p(:,end,k)-t.p(:,1,k),ft)+cross(pc-t.p(:,1,k),fc);z=[m;ft;A'*fc]./scale;
 for it=1:8,sh=tsfs.ivp(fr,g,z,scale,o,true);if norm(sh.tipMoment,inf)<1e-9,break,end;z(1:3)=z(1:3)-sh.JtipMoment(:,1:3)\sh.tipMoment;end
 cfg=b.cfg;cfg.auditInitialZ=z;sol=audit_seed_adapter(fr,g,b.runs{1,k-1},o,'fmincon',cfg);sh=sol.evaluation.shape;d=tsfs.diagnose_relaxed(fr,g,sol,o);
 rows=[rows;struct('frame',k,'reason',sol.reason,'seconds',sol.seconds,'iterations',sol.iterations,'ivps',sol.counts.augmentedIVPs,'sensorCost',sol.evaluation.objective,'originalSensorCost',old.evaluation.objective,'relaxedViolation',sol.relaxed.violation,'optimality',sol.stationarity,'tipErrorN',norm(sh.x(4:6)-ft),'contactErrorN',norm(sh.force-fc),'shapeRmsMm',sqrt(mean(sum((sh.p-t.p(:,:,k)).^2,1))),'fbgRms',d.fbgRms,'lambdaN',sh.x(7),'flags',strjoin(d.flags,';'))];runs{find([6 15 34]==k)}=sol;
 writetable(struct2table(rows,'AsArray',true),fullfile(pub,'diagnostic_truth_seeds.csv'));save(fullfile(probe,'truth_seed_replays.mat'),'runs','rows','-v7.3');fprintf('SEED frame %d %s tip %.6g cost %.6g->%.6g violation %.3g\n',k,sol.reason,rows(end).tipErrorN,old.evaluation.objective,sol.evaluation.objective,sol.relaxed.violation);
end
end
