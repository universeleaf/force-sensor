function out=subset_sensor_packet(packet,indices)
%SUBSET_SENSOR_PACKET Subset frames while preserving calibration coordinates.
n=numel(packet.timeSeconds);
assert(isvector(indices)&&all(indices==round(indices))&&all(indices>=1&indices<=n)&& ...
    all(diff(indices)>0),'Frame indices must be ordered, unique and in range.');
out=packet;
for item=fieldnames(packet)'
    name=item{1}; if ismember(name,{'fbgIdx','sFbgMm'}),continue;end
    a=packet.(name); sz=size(a);
    if (isnumeric(a)||iscell(a)||islogical(a))&&~isscalar(a)&&sz(end)==n
        ix=repmat({':'},1,ndims(a)); ix{end}=indices; out.(name)=a(ix{:});
    end
end
end
