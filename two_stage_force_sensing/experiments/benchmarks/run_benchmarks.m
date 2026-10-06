function summary=run_benchmarks(names,repeats)
[root,paths]=setup_tsfs;if nargin<1,names={'sliding_clean','s_channel_two_contact','two_contact','three_contact','spatial_sliding','spatial_sliding_noisy','spatial_three_channel','free_space'};end
if nargin<2,repeats=2;end
o=tsfs.defaults;summary=struct;allRows=struct([]);
for j=1:numel(names)
 name=names{j};file=fullfile(root,'datasets',[name '.mat']);
 if isfile(file),a=load(file);truth=a.truth;else,a=load(fullfile(root,'datasets',name,'input.mat'));b=load(fullfile(root,'datasets',name,'truth.mat'));truth=b.truth;end
 frames=tsfs.read_input(a.sensorInput);runs=cell(repeats,numel(frames));rows=[];
 for rep=1:repeats
  previous=[];
  for k=1:numel(frames)
   clock=tic;
   try
    out=tsfs.step(frames{k},previous,o);row=score(out,truth,k,name,rep);
    % A fresh diagnostic call must not mutate estimates or solver state.
    if k==1&&rep==1,before=out;dd=tsfs.diagnose(frames{k},out.geometry,out.mechanics,o);assert(isequaln(before,out),'Diagnostics mutated estimate');end
    previous=out;
   catch err
    out=struct('valid',false,'time',frames{k}.time,'seconds',toc(clock),'error',getReport(err,'extended','hyperlinks','off'));
    row=emptyrow(name,rep,k);row.valid=false;row.seconds=out.seconds;row.flags=err.identifier;fprintf('%s\n',out.error);
   end
   saved=out;
   if saved.valid
    % ODE solution handles capture MATLAB workspaces; never serialize them.
    saved.mechanics.evaluation.shape.pieces={};saved.geometry.kinematicShape.Jp=[];saved.geometry.kinematicShape.Jt=[];
   end
   runs{rep,k}=saved;rows=[rows;row];
   fprintf('%s rep %d frame %d/%d %.3fs C=%d SQP=%d IVP=%d conv=%d FcErr=%.5g tipErr=%.5g flags=%s\n',name,rep,k,numel(frames),row.seconds,row.contacts,row.sqpIterations,row.totalIVPs,row.converged,row.contactErrorN,row.tipErrorN,row.flags);
   save(fullfile(paths.results,[name '_benchmark.mat']),'runs','rows','o','-v7.3');
  end
 end
 allRows=[allRows;rows];summary.(name)=aggregate(rows);writetable(struct2table(rows),fullfile(paths.results,[name '_frames.csv']));
 fid=fopen(fullfile(paths.results,'benchmark_summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
end
writetable(struct2table(allRows),fullfile(paths.results,'all_frames.csv'));disp(summary);
end
function r=emptyrow(name,rep,k)
r=struct('scenario',name,'repeat',rep,'frame',k,'valid',true,'seconds',NaN,'geometrySeconds',NaN,'mechanicsSeconds',NaN,'diagnosticSeconds',NaN, ...
 'geometryIterations',0,'sqpIterations',0,'qpIterations',0,'stateIVPs',0,'augmentedIVPs',0,'totalIVPs',0,'rhsEvaluations',0,'odeSegments',0,'stateDimension',0,'rejectedTrials',0, ...
 'contacts',0,'activeContacts',0,'converged',false,'contactErrorN',NaN,'tipErrorN',NaN,'totalErrorN',NaN,'arcErrorMm',NaN,'shapeErrorMm',NaN, ...
 'penetrationMm',NaN,'projectionN',NaN,'normalProductNmm',NaN,'tangency',NaN,'coneExcessN',NaN,'tipMomentNmm',NaN,'fbgRms',NaN,'maxFrictionWorkNmm',NaN,'stationarity',NaN,'flags','');
end
function r=score(out,t,k,name,rep)
r=emptyrow(name,rep,k);m=out.mechanics;d=out.diagnostics;c=m.counts;
r.seconds=out.seconds;r.geometrySeconds=out.geometry.seconds;r.mechanicsSeconds=m.seconds;r.diagnosticSeconds=d.seconds;
r.geometryIterations=out.geometry.iterations;r.sqpIterations=m.iterations;r.qpIterations=c.qpIterations;r.stateIVPs=c.stateIVPs;r.augmentedIVPs=c.augmentedIVPs;r.totalIVPs=c.stateIVPs+c.augmentedIVPs;r.rhsEvaluations=c.rhsEvaluations;r.odeSegments=c.odeSegments;r.stateDimension=c.maxStateDimension;r.rejectedTrials=c.trialRejections;
r.contacts=d.candidateCount;r.activeContacts=d.activeCount;r.converged=m.converged;
if isfield(t,'contactForces'),fc=t.contactForces(:,:,k);else,fc=t.contactForce(:,k);end
trueS=t.contactS(:,min(k,size(t.contactS,2)))';[trueS,order]=sort(trueS);fc=fc(:,order);
if numel(trueS)==numel(out.geometry.s),r.contactErrorN=norm(out.contactForce-fc,'fro');r.arcErrorMm=norm(out.geometry.s-trueS)/sqrt(max(1,numel(trueS)));end
r.tipErrorN=norm(out.tipForce-t.tipForce(:,k));r.totalErrorN=norm(sum(out.contactForce,2)+out.tipForce-sum(fc,2)-t.tipForce(:,k));
p=m.evaluation.shape.p;pt=t.p(:,:,k);if isequal(size(p),size(pt)),r.shapeErrorMm=sqrt(mean(sum((p-pt).^2,1)));end
r.penetrationMm=d.maximumPenetrationMm;r.projectionN=max([0;abs(d.projectionN(:))]);r.normalProductNmm=max([0;abs(d.normalProductNmm(:))]);r.tangency=max([0;abs(d.tangency(:))]);r.coneExcessN=max([0;d.coneExcessN(:)]);r.tipMomentNmm=norm(d.tipMomentNmm,inf);r.fbgRms=d.fbgRms;r.maxFrictionWorkNmm=max([0;d.frictionWorkNmm(:)]);r.stationarity=m.stationarity;r.flags=strjoin(d.flags,';');
end
function s=aggregate(rows)
v=[rows.valid];steady=[rows.repeat]>1;if ~any(steady),steady=true(size(v));end
s=struct('totalFrames',numel(rows),'validFrames',sum(v),'convergedFrames',sum([rows.converged]),'flaggedFrames',sum(~cellfun(@isempty,{rows.flags})), ...
 'firstFrameSeconds',rows(1).seconds,'warmMedianSeconds',median([rows(steady).seconds]),'warmP95Seconds',prctile([rows(steady).seconds],95), ...
 'meanGeometrySeconds',mean([rows(v).geometrySeconds]),'meanMechanicsSeconds',mean([rows(v).mechanicsSeconds]), ...
 'meanSQPIterations',mean([rows(v).sqpIterations]),'meanIVPs',mean([rows(v).totalIVPs]),'meanRHSEvaluations',mean([rows(v).rhsEvaluations]), ...
 'contactRMSE_N',sqrt(mean([rows.contactErrorN].^2,'omitnan')),'tipRMSE_N',sqrt(mean([rows.tipErrorN].^2,'omitnan')), ...
 'totalRMSE_N',sqrt(mean([rows.totalErrorN].^2,'omitnan')),'arcRMSE_Mm',sqrt(mean([rows.arcErrorMm].^2,'omitnan')), ...
 'shapeRMSE_Mm',sqrt(mean([rows.shapeErrorMm].^2,'omitnan')),'maximumPenetrationMm',max([rows.penetrationMm]),'maximumFbgRms',max([rows.fbgRms]));
end
