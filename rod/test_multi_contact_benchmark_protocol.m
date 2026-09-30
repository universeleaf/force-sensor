function test_multi_contact_benchmark_protocol()
% Baseline must be invariant to environment geometry; ablation options run.
root=fileparts(fileparts(mfilename('fullpath')));
fixture=fullfile(root,'out','demos','multi_contact','s_channel_two_contact','input.mat');
d=load(fixture,'sensorInput');
input=d.sensorInput.frames{1};
baseline=estimate_planar_shape_only_point_loads(input);
changed=input;
changed.model.planePointXZ=changed.model.planePointXZ+[30;-10];
changed.model.planeNormalXZ=-changed.model.planeNormalXZ;
second=estimate_planar_shape_only_point_loads(changed);
assert(max(abs(baseline.contactS(:)-second.contactS(:)))<1e-9);
assert(max(abs(baseline.contactForceXZ(:)-second.contactForceXZ(:)))<1e-9);
assert(max(abs(baseline.tipForceXZ(:)-second.tipForceXZ(:)))<1e-9);
ablation=estimate_planar_multi_contact(input,struct('gapWeight',0,'tangentWeight',0));
assert(ablation.gapWeight==0 && ablation.tangentWeight==0);
offsetInput=input;
plane=offsetInput.model.contactPlaneIndex(1);
normal=offsetInput.model.planeNormalXZ(:,plane);
offsetInput.model.planePointXZ(:,plane)=offsetInput.model.planePointXZ(:,plane)+normal/norm(normal);
corrected=estimate_planar_multi_contact(offsetInput,struct('planePointStdMm',1));
assert(abs(corrected.planePointOffsetMm(1)+1)<0.05);
assert(~corrected.requiresReview);
end
