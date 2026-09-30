function write_formulation_window_csv(estimate, path)
%WRITE_FORMULATION_WINDOW_CSV One row per time/contact; world-frame N and mm.
T=numel(estimate.timeSeconds); K=size(estimate.contactForce,2); rows=max(1,K)*T;
values=zeros(rows,15); row=0;
for k=1:T
    for j=1:max(1,K)
        row=row+1; force=zeros(3,1); s=nan; active=false; plane=nan;
        if K>0
            force=estimate.contactForce(:,j,k); s=estimate.contactArcLength(j,k);
            active=estimate.activeContacts(j,k); plane=estimate.contactPlaneIndex(j);
        end
        values(row,:)=[estimate.timeSeconds(k),j,plane,s,active,force',norm(force), ...
            estimate.tipForce(:,k)',norm(estimate.tipForce(:,k)), ...
            norm(estimate.totalForceResultant(:,k)),estimate.quality.requiresReview(k)];
    end
end
names={'timeSeconds','contactSlot','planeIndex','arcMm','active', ...
    'contactFxN','contactFyN','contactFzN','contactMagnitudeN', ...
    'tipFxN','tipFyN','tipFzN','tipMagnitudeN','totalMagnitudeN','requiresReview'};
writetable(array2table(values,'VariableNames',names),path);
end
