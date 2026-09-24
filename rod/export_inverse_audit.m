function audit = export_inverse_audit(r)
%EXPORT_INVERSE_AUDIT Export inverse-only evidence with explicit scope.
% No baseline is manufactured when only the inverse has been run.
audit.scenario=r.config.scenarioName;
audit.scope='All independently solved output frames; inverse-only, no new Aloi comparison';
audit.historySource='oracle';
if isfield(r.measurements,'historySource'), audit.historySource=r.measurements.historySource; end
audit.limitations='Simulation; calibrated material/intrinsic profile; checks do not certify force accuracy or optimizer convergence.';
if strcmp(audit.historySource,'oracle')
    audit.limitations=[audit.limitations ' Exact previous internal truth shape supplied.'];
else
    audit.historyIntervalSeconds=r.measurements.historyIntervalSeconds;
    audit.predictionIntervalSeconds=r.measurements.predictionIntervalSeconds;
end
audit.validation=r.validation;
audit.outputFrameCount=size(r.ours.state,2);
audit.internalFrameCount=r.forward.internalFrameCount;
audit.finalContactArcLengthMm=r.ours.state(6,end);
audit.finalTrueContactArcLengthMm=r.forward.contactArcLength(end);
audit.finalTrueContactForceN=r.forward.contactForceResultant(:,end);
audit.finalEstimatedContactForceN=r.ours.contactForceResultant(:,end);
audit.finalTrueTipForceN=r.forward.tipLoad(:,end);
audit.finalEstimatedTipForceN=r.ours.tipForce(:,end);
audit.finalMode=r.ours.complementarityMode{end};
audit.maxPlaneShiftMm=max(vecnorm(r.ours.state(1:3,:)-r.measurements.planePointMeasured,2,1));
phases=unique(r.forward.phase,'stable');
for k=1:numel(phases)
    mask=strcmp(r.forward.phase,phases{k});
    eContact=r.ours.contactForceResultant(:,mask)-r.forward.contactForceResultant(:,mask);
    eTip=r.ours.tipForce(:,mask)-r.forward.tipLoad(:,mask);
    audit.phaseMetrics.(phases{k})=struct('frameCount',sum(mask), ...
        'contactForceRmseN',sqrt(mean(sum(eContact.^2,1))), ...
        'tipForceRmseN',sqrt(mean(sum(eTip.^2,1))), ...
        'totalForceRmseN',sqrt(mean(sum((eContact+eTip).^2,1))));
end
audit.config=r.config;
folder=r.config.outputDir;
fid=fopen(fullfile(folder,'audit.json'),'w');
assert(fid>=0,'Cannot create audit.json');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(audit,'PrettyPrint',true));
clear cleanup;
T=table((1:audit.outputFrameCount)',r.forward.actuationMm(:), ...
    r.forward.phase(:),r.ours.complementarityMode(:),r.ours.state(6,:)', ...
    r.forward.contactArcLength(:),r.ours.shapeRmseMm(:), ...
    'VariableNames',{'frame','path_mm','phase','mode','estimated_contact_s_mm','true_contact_s_mm','shape_rmse_mm'});
axes={'x','y','z'};
for j=1:3
    T.(['true_contact_' axes{j} '_N'])=r.forward.contactForceResultant(j,:)';
    T.(['estimated_contact_' axes{j} '_N'])=r.ours.contactForceResultant(j,:)';
    T.(['true_tip_' axes{j} '_N'])=r.forward.tipLoad(j,:)';
    T.(['estimated_tip_' axes{j} '_N'])=r.ours.tipForce(j,:)';
end
writetable(T,fullfile(folder,'trajectory.csv'));
end
