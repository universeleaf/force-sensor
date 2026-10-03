function [candidates,audit] = track_formulation_candidates(raw,s,time,options)
% Associate geometry minima through time; never read truth/mode labels.
T=numel(raw);L=s(end)-s(1);tracks=struct('plane',{},'arcs',{},'gaps',{},'last',{});
ambiguous=false;generated=sum(cellfun(@(v)size(v,1),raw));
for k=1:T
    pool=sortrows(raw{k},3);kept=zeros(0,3);
    for j=1:size(pool,1)
        if ~any(kept(:,1)==pool(j,1)&abs(kept(:,2)-pool(j,2))<options.candidateMergeMm)
            kept(end+1,:)=pool(j,:); %#ok<AGROW>
        end
    end
    used=false(1,numel(tracks));taken=false(1,size(kept,1));
    while ~isempty(kept)
        cost=inf(numel(tracks),size(kept,1));
        for i=1:numel(tracks)
            if used(i),continue;end
            previous=tracks(i).last;dt=(time(k)-time(previous))/options.referencePeriodSeconds;
            gate=options.candidateTrackMaxFraction*L*sqrt(max(1,dt));
            for j=find(~taken)
                distance=abs(kept(j,2)-tracks(i).arcs(previous));
                if kept(j,1)==tracks(i).plane&&distance<=gate,cost(i,j)=distance;end
            end
        end
        [distance,index]=min(cost(:));
        if isempty(distance)||~isfinite(distance),break;end
        [i,j]=ind2sub(size(cost),index);
        alternatives=[cost(i,[1:j-1 j+1:end])';cost([1:i-1 i+1:end],j)];
        if any(abs(alternatives-distance)<options.minArcSeparationMm),ambiguous=true;end
        tracks(i).arcs(k)=kept(j,2);tracks(i).gaps(k)=kept(j,3);tracks(i).last=k;
        used(i)=true;taken(j)=true;
    end
    for j=find(~taken)
        arcs=nan(1,T);gaps=arcs;arcs(k)=kept(j,2);gaps(k)=kept(j,3);
        tracks(end+1)=struct('plane',kept(j,1),'arcs',arcs,'gaps',gaps,'last',k); %#ok<AGROW>
    end
end
count=numel(tracks);scores=zeros(count,2);
for i=1:count,scores(i,:)=[-sum(isfinite(tracks(i).arcs)),min(tracks(i).gaps,[],'omitnan')];end
[~,order]=sortrows(scores,[1 2]);truncated=count>options.maxContacts;order=order(1:min(count,options.maxContacts));
planes=zeros(1,numel(order));seed=zeros(numel(order),T);support=false(size(seed));
for j=1:numel(order)
    tr=tracks(order(j));planes(j)=tr.plane;support(j,:)=isfinite(tr.arcs);at=find(support(j,:));
    if numel(at)==1,seed(j,:)=tr.arcs(at);
    else
        seed(j,:)=interp1(time(at),tr.arcs(at),time,'linear','extrap');
        seed(j,time<time(at(1)))=tr.arcs(at(1));seed(j,time>time(at(end)))=tr.arcs(at(end));
    end
end
[~,order]=sort(seed(:,1));planes=planes(order);seed=seed(order,:);support=support(order,:);
corner=false;
if size(seed,1)>1
    remove=find(diff(seed(:,1))<options.minArcSeparationMm)+1;corner=~isempty(remove);
    planes(remove)=[];seed(remove,:)=[];support(remove,:)=[];
end
candidates=struct('planeIndex',planes,'seedArcMm',seed(:,1)','seedArcByFrameMm',seed);
audit=struct('generatedCount',generated,'trackedCount',count,'retainedCount',numel(planes), ...
    'truncated',truncated,'ambiguousCornerCandidates',corner,'ambiguousTrackAssignment',ambiguous, ...
    'trackObservedAtFrame',support,'trackArcRangeMm',[min(seed,[],2) max(seed,[],2)], ...
    'usesContactLabels',false,'coverageCertified',false, ...
    'scope','Per-frame gap-minimum merging and bounded temporal nearest-arc association; contact births/deaths use inactive slots.');
end
