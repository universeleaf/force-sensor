function export_single_contact_evidence
% Export saved solutions without rerunning or changing their measured timing.
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'single_contact');
a=load(fullfile(root,'datasets','sliding_clean.mat'));d=load(fullfile(folder,'ours.mat'));b=load(fullfile(folder,'aloi.mat'));
packet=a.sensorInput.packet;t=a.truth;
pointError=max(abs(packet.planePointMm-t.planePoint),[],'all');
normalError=max(abs(packet.planeNormal-t.planeNormal),[],'all');
assert(pointError==0&&normalError==0,'Expected exact environment observations.');
meta=jsondecode(fileread(fullfile(folder,'metadata.json')));
meta.environmentPointMaximumErrorMm=pointError;meta.environmentNormalMaximumError=normalError;
meta.environmentPriorStd=sqrt(diag(packet.planeCovariance(:,:,1)));
meta.packetProvenance=packet.provenance;
meta.fbgCount=numel(packet.fbgIdx);
atomic_write_artifact(fullfile(folder,'metadata.json'),'json',meta);
rows=[];shapeRows=[];
for j=1:2
 for k=1:size(d.runs,3)
  out=d.runs{j,2,k};q=out.diagnostics;e=out.mechanics.evaluation;
  rows=[rows;struct('mu',d.mus(j),'frame',k,'gapMm',q.gapMm(1),'coneExcessN',q.coneExcessN(1), ...
   'normalProductNmm',q.normalProductNmm(1),'frictionWorkNmm',q.frictionWorkNmm(1), ...
   'slidingLawErrorN',q.slidingLawErrorN(1),'slipNormMm',norm(e.slip(:,1)), ...
   'contactPointMismatchMm',q.contactPointMismatchMm(1),'segmentsPerIVP',e.shape.segments, ...
   'ode45SegmentCalls',e.shape.segments*(out.mechanics.counts.stateIVPs+out.mechanics.counts.augmentedIVPs))];
 end
end
for k=[1 size(d.runs,3)]
 p=d.runs{1,2,k}.mechanics.evaluation.shape;z=d.runs{2,2,k}.mechanics.evaluation.shape;
 assert(isequal(size(p.p),size(t.p(:,:,k)),size(b.aloi.p(:,:,k))));
 for i=1:numel(p.s)
  shapeRows=[shapeRows;struct('frame',k,'node',i,'arcMm',p.s(i),'trueX',t.p(1,i,k),'trueY',t.p(2,i,k),'trueZ',t.p(3,i,k), ...
   'oursX',p.p(1,i),'oursY',p.p(2,i),'oursZ',p.p(3,i),'zeroX',z.p(1,i),'zeroY',z.p(2,i),'zeroZ',z.p(3,i), ...
   'aloiX',b.aloi.p(1,i,k),'aloiY',b.aloi.p(2,i,k),'aloiZ',b.aloi.p(3,i,k))];
 end
end
writetable(struct2table(rows),fullfile(folder,'physics.csv'));
writetable(struct2table(shapeRows),fullfile(folder,'shape_profiles.csv'));
fprintf('No FBG or environment noise injected; saved diagnostic and shape evidence exported.\n');
end
