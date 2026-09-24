function summary = audit_sensor_stage1(folder,filename)
%AUDIT_SENSOR_STAGE1 Component errors, solver outcomes and equal-input ablation.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1, folder=fullfile(root,'out','stage1','video_zero_reduced'); end
if nargin<2
    path=resolve_result_file(folder); [~,name,ext]=fileparts(path); filename=[name ext];
end
d=load(fullfile(folder,filename),'results'); r=d.results;
validate_lcp_dependency(root);
tube=make_experiment_tube(r.config);
baseline=estimate_shape_only_point_loads(tube,r.measurements,r.config);
summary=struct;
summary.scenario=r.config.scenarioName;
summary.historySource=r.measurements.historySource;
summary.resultFile=filename;
summary.validation=r.validation;
summary.full=errors(r.ours);
summary.shapeOnly=errors(baseline);
summary.shapeOnlyCaveat=baseline.description;
summary.noise=r.config.sensing;
summary.trueTipN=r.config.tipLoadN;
summary.independentTrajectories=1;
summary.frameCount=size(r.ours.state,2);
delta=r.ours.p-r.measurements.p;
summary.maxMeasuredShapeRmseMm=max(reshape(sqrt(mean(sum(delta.^2,1),2)),1,[]));
delta=r.ours.p-r.forward.p;
summary.maxTrueShapeRmseMm=max(reshape(sqrt(mean(sum(delta.^2,1),2)),1,[]));
summary.selectedInitializations=r.ours.initializationName;
separated=vecnorm(r.forward.contactForceResultant)>0.05 & ...
    (r.forward.s(end)-r.forward.contactArcLength)>=10;
summary.separatedContactFrameIndices=find(separated);
summary.separatedContactDefinition='Scoring subset only: true contact > 0.05 N, true tip-to-contact separation >= 10 mm; all-frame scores retained.';
if any(separated)
    summary.separatedFull=errors(r.ours,separated);
    summary.separatedShapeOnly=errors(baseline,separated);
end
if isfield(r.ours,'frameSeconds')
    summary.solverSecondsTotal=sum(r.ours.frameSeconds);
    summary.solverSecondsMedian=median(r.ours.frameSeconds);
    if isfield(r.ours,'preprocessingSeconds')
        summary.preprocessingSeconds=r.ours.preprocessingSeconds;
    end
    flags=[]; stalled=0;
    for k=1:numel(r.ours.solverTrace)
        q=r.ours.solverTrace{k};
        for j=1:numel(q)
            flags(end+1)=q{j}.exitflag; %#ok<AGROW>
            stalled=stalled+(q{j}.acceptedAlpha==0);
        end
    end
    summary.subproblemExitFlags=flags;
    summary.nonpositiveSubproblemExits=sum(flags<=0);
    summary.zeroAcceptedSteps=stalled;
end
save(fullfile(folder,'shape_only_baseline.mat'),'baseline','summary');
fid=fopen(fullfile(folder,'comparison.json'),'w');
fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true)); fclose(fid);
disp(summary);
    function e=errors(o,mask)
        if nargin<2, mask=true(1,size(r.ours.state,2)); end
        e.contactRmseN=sqrt(mean(sum((o.contactForceResultant(:,mask)-r.forward.contactForceResultant(:,mask)).^2,1)));
        e.tipRmseN=sqrt(mean(sum((o.tipForce(:,mask)-r.forward.tipLoad(:,mask)).^2,1)));
        e.totalRmseN=sqrt(mean(sum((o.contactForceResultant(:,mask)+o.tipForce(:,mask)-r.forward.totalForceResultant(:,mask)).^2,1)));
        e.finalContactN=o.contactForceResultant(:,end);
        e.finalTipN=o.tipForce(:,end);
    end
end
