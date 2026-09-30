function data = export_literature_geometry()
% Export solved geometry for a paper figure; no new inference or labels.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','formulation_window');
report=jsondecode(fileread(fullfile(folder,'comparison.json')));
assert(strcmp(report.state,'complete'),'rod:IncompleteGeometry','Reference experiment is incomplete.');
ids={'two_contact','three_contact_noisy','spatial_sliding_noisy'};
data=struct('referenceRunId',report.runRecord.runId,'referenceComparisonSha256', ...
    file_sha256(fullfile(folder,'comparison.json')),'cases',{{}});
for i=1:numel(ids)
    row=report.cases(strcmp({report.cases.id},ids{i}));assert(numel(row)==1);
    source=fullfile(folder,row.artifactFolder);
    for name={'input','truth','estimate'}
        n=name{1};assert(strcmp(file_sha256(fullfile(source,[n '.mat'])),row.artifactSha256.(n)), ...
            'rod:GeometryIntegrity','Reference artifact changed.');
    end
    input=load(fullfile(source,'input.mat'),'sensorInput');
    t=load(fullfile(source,'truth.mat'),'truth');e=load(fullfile(source,'estimate.mat'),'estimate');
    t=t.truth;e=e.estimate;sh=e.frames{1}.shape;
    contact=interp1(input.sensorInput.tube.s,t.p(:,:,1)',t.contactS(:,1))';
    entry=struct('id',ids{i},'frame',1,'timeSeconds',e.timeSeconds(1),'truthP',t.p(:,:,1), ...
        'estimateP',sh.p,'truthContactP',contact,'estimateContactP',sh.contactPoints, ...
        'truthContactForce',t.contactForces(:,:,1),'estimateContactForce',e.contactForce(:,:,1), ...
        'planePoint',e.frames{1}.point,'planeNormal',e.frames{1}.normal, ...
        'artifactSha256',row.artifactSha256);
    data.cases{end+1}=entry;
end
atomic_write_artifact(fullfile(root,'out','benchmarks','literature','geometry.json'),'json',data);
end
