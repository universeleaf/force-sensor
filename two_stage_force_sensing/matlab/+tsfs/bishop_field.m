function w=bishop_field(fr,query)
% Interpolate geometric components within calibrated material segments.
% No material intrinsic curvature or true twist is subtracted from w.
s=fr.tube.s(:)';K=reshape(getTubeK(fr.tube),3,[]);
change=find(any(abs(diff(fr.tube.uhat,1,2))>1e-14,1)|any(abs(diff(K,1,2))>1e-10,1))+1;
edges=unique([s(1) s(change) s(end)]);query=query(:)';w=zeros(2,numel(query));
for j=1:numel(edges)-1
 if j<numel(edges)-1,obs=fr.arcs>=edges(j)&fr.arcs<edges(j+1);take=query>=edges(j)&query<edges(j+1);
 else,obs=fr.arcs>=edges(j)&fr.arcs<=edges(j+1);take=query>=edges(j)&query<=edges(j+1);end
 assert(sum(obs)>=2,'tsfs:BishopSampling','Each calibrated segment needs at least two sensor stations.');
 w(:,take)=interp1(fr.arcs(obs),fr.u(:,obs)',query(take),'pchip','extrap')';
end
assert(all(query>=s(1)&query<=s(end)),'Query outside rod.');
end
