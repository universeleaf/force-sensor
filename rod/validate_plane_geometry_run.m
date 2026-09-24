function report=validate_plane_geometry_run(runId,kind)
% Independently audit saved force directions, cones and dense rod geometry.
% A passed audit certifies these numerical tolerances, not force accuracy.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
if nargin<2,kind='geometry';end
assert(any(strcmp(kind,{'geometry','accuracy'})),'rod:InvalidRunKind','Unknown geometry audit folder.');
folder=fullfile(root,'out',kind);
if nargin<1||isempty(runId)
    source=jsondecode(fileread(fullfile(folder,'comparison.json')));runId=source.runRecord.runId;
end
folder=fullfile(folder,runId);source=jsondecode(fileread(fullfile(folder,'comparison.json')));
assert(strcmp(source.state,'complete'),'rod:IncompleteGeometryRun','Geometry benchmark is not complete.');
report=struct('runId',runId,'scope','Post-hoc numerical physics audit; no accuracy certification.', ...
    'auditorSha256',file_sha256([mfilename('fullpath') '.m']), ...
    'gapToleranceMm',1e-5,'constraintTolerance',1e-5,'workToleranceNmm',1e-6, ...
    'allPassed',false,'cases',{{}});
for j=1:numel(source.cases)
    entry=source.cases(j);d=load(fullfile(folder,entry.id,'results.mat'),'output','sensorInput');
    o=d.output.ours;nt=size(o.state,2);work=zeros(1,nt);normalComponent=work;coneExcess=work;
    cosine=nan(1,nt);stepMm=work;
    for k=1:nt
        n=o.planeNormal(:,k); ft=o.frictionForceResultant(:,k); s=o.contactArcLength(k);
        previous=d.output.measurements.previousShape{k};
        if ~isempty(o.historyEstimate{k}),previous=o.historyEstimate{k}.shape;end
        if strcmp(d.output.config.forceSensor.historyPointInterpolation,'integrated')
            oldPoint=sample_integrated_shape(d.sensorInput.tube,previous,s);
        else
            oldPoint=interp1(d.sensorInput.tube.s,previous.p',s,'linear')';
        end
        v=(eye(3)-n*n')*(o.contactPoint(:,k)-oldPoint);
        work(k)=ft'*v;normalComponent(k)=abs(n'*ft);
        coneExcess(k)=max(0,norm(ft)-o.frictionMu(k)*o.state(7,k));
        stepMm(k)=norm(v);
        if norm(v)*norm(ft)>1e-10,cosine(k)=work(k)/(norm(v)*norm(ft));end
    end
    item=struct('id',entry.id,'frictionWorkNmm',work,'frictionSlipCosine',cosine, ...
        'tangentialStepMm',stepMm,'frictionNormalComponentN',normalComponent, ...
        'euclideanConeExcessN',coneExcess,'denseMinimumGapMm',entry.corrected.denseMinimumGapMm, ...
        'activeConstraintResidual',o.activeConstraintResidual,'passed',false);
    item.passed=all(work<=report.workToleranceNmm) && max(normalComponent)<1e-7 && ...
        max(coneExcess)<1e-6 && min(item.denseMinimumGapMm)>=-report.gapToleranceMm && ...
        max(o.activeConstraintResidual)<report.constraintTolerance;
    report.cases{j}=item;
    fprintf('GEOMETRY_AUDIT %s: passed=%d, max work %.3g N mm, min gap %.3g mm\n', ...
        entry.id,item.passed,max(work),min(item.denseMinimumGapMm));
end
report.allPassed=all(cellfun(@(c)c.passed,report.cases));
atomic_write_artifact(fullfile(folder,'physics_audit.json'),'json',report);
assert(report.allPassed,'rod:GeometryAuditFailure','Saved geometry results failed the physics audit; inspect physics_audit.json.');
end
