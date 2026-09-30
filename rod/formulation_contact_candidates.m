function [candidates, shapes, audit] = formulation_contact_candidates(tube, obs, options)
%FORMULATION_CONTACT_CANDIDATES Gap minima from measured shape, never labels.
% A fixed superset of slots is used over a window. Complementarity can turn
% their forces off. Enumeration is bounded and its coverage is not certified.
T=obs.frameCount; shapes=cell(1,T); pool=zeros(0,3);
for k=1:T
    elastic=zeros(3,numel(obs.arcs));
    for a=1:numel(obs.axes)
        ax=obs.axes(a); intrinsic=interp1(tube.s,tube.uhat(ax,:),obs.arcs,'previous');
        elastic(ax,:)=obs.u(a,:,k)-intrinsic;
    end
    u=tube.uhat;
    for ax=obs.axes, u(ax,:)=u(ax,:)+interp1(obs.arcs,elastic(ax,:),tube.s,'pchip','extrap'); end
    [R,p]=integrate_curvature_field(tube,u,obs.basePose(:,:,k));
    shapes{k}=struct('p',p,'R',R,'u',u);
    for j=1:obs.planeCount
        gap=obs.normal(:,j,k)'*(p-obs.point(:,j,k));
        minima=find(gap(2:end-1)<=gap(1:end-2)&gap(2:end-1)<gap(3:end))+1;
        if isempty(minima), [~,z]=min(gap(2:end-1)); minima=z+1; end
        for z=minima
            if gap(z)<=options.candidateMaxGapMm
                pool(end+1,:)=[j,tube.s(z),abs(gap(z))]; %#ok<AGROW>
            end
        end
    end
end
if ~isempty(pool)
    pool=sortrows(pool,3); kept=zeros(0,3);
    for k=1:size(pool,1)
        if isempty(kept)||~any(kept(:,1)==pool(k,1)&abs(kept(:,2)-pool(k,2))<options.candidateMergeMm)
            kept(end+1,:)=pool(k,:); %#ok<AGROW>
        end
    end
else, kept=zeros(0,3); end
truncated=size(kept,1)>options.maxContacts;
kept=kept(1:min(size(kept,1),options.maxContacts),:); kept=sortrows(kept,2);
% Different planes can nominate indistinguishable arclengths. Keep the nearer
% surface and report the excluded slot; simultaneous corner contact requires
% a distinct formulation and is not silently approximated here.
ambiguous=false;
if size(kept,1)>1
    remove=find(diff(kept(:,2))<options.minArcSeparationMm)+1;
    ambiguous=~isempty(remove); kept(remove,:)=[];
end
candidates=struct('planeIndex',kept(:,1)','seedArcMm',kept(:,2)');
audit=struct('generatedCount',size(pool,1),'retainedCount',size(kept,1), ...
    'truncated',truncated,'ambiguousCornerCandidates',ambiguous, ...
    'usesContactLabels',false,'coverageCertified',false);
end
