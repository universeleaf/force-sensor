function [p, R] = cosserat_state_at_arc(shape, arcs)
%COSSERAT_STATE_AT_ARC Continuous evaluation, including predecessor material points.
arcs=arcs(:)'; edges=shape.segmentEdges;
assert(all(arcs>=edges(1))&&all(arcs<=edges(end)),'rod:InvalidMaterialArc','Material point outside rod.');
q=zeros(15,numel(arcs));
for k=1:numel(shape.pieces)
    take=arcs>=edges(k)&arcs<=edges(k+1);
    if any(take), q(:,take)=deval(shape.pieces{k},arcs(take)); end
end
p=q(1:3,:); R=reshape(q(4:12,:),3,3,[]);
end
