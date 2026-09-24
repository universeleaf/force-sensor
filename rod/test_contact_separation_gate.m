function test_contact_separation_gate()
% A noisy force seed must not create contact with a clearly distant plane.
root=fileparts(fileparts(mfilename('fullpath')));
d=load(fullfile(root,'out','stage2','independent_video_tip','sensor_input.mat'),'sensorInput');
input=d.sensorInput; packet=input.packet; nt=numel(packet.timeSeconds);
names=fieldnames(packet);
for k=1:numel(names)
    name=names{k}; if ismember(name,{'fbgIdx','sFbgMm'}), continue;end
    a=packet.(name); sz=size(a);
    if isnumeric(a) && ~isscalar(a) && sz(end)==nt
        ix=repmat({':'},1,ndims(a));ix{end}=1;packet.(name)=a(ix{:});
    end
end
input.packet=packet;
output=estimate_sensor_forces(input);
assert(strcmp(output.ours.complementarityMode{1},'no-contact'), ...
    'A force seed created contact with an observably distant plane.');
assert(norm(output.ours.contactForceResultant)<1e-8);
assert(~output.quality.forceAccuracyCertified && ~output.quality.historyUncertaintyModeled);
disp('Observable separation and public sensor API quality checks passed.');
end
