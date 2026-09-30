function spec = formulation_window_spec(planeCount, contactPlaneIndex, m)
%FORMULATION_WINDOW_SPEC One latent geometry block per observed plane.
% Additional base moments lift the implicit Cosserat boundary-value map;
% terminal equilibrium constraints eliminate these nuisance coordinates.
spec=struct('planeCount',planeCount,'contactCount',numel(contactPlaneIndex), ...
    'contactPlaneIndex',contactPlaneIndex(:)','frictionDirectionCount',m);
spec.plane=repmat(struct('point',[],'eta',[]),1,planeCount); cursor=0;
for j=1:planeCount
    spec.plane(j).point=cursor+(1:3); spec.plane(j).eta=cursor+(4:5); cursor=cursor+5;
end
spec.contact=repmat(struct('s',[],'fn',[],'beta',[],'lambda',[],'plane',[]),1,spec.contactCount);
for j=1:spec.contactCount
    spec.contact(j)=struct('s',cursor+1,'fn',cursor+2,'beta',cursor+(3:2+m), ...
        'lambda',cursor+3+m,'plane',contactPlaneIndex(j)); cursor=cursor+3+m;
end
spec.tip=cursor+(1:3); spec.baseMoment=cursor+(4:6); spec.stateLength=cursor+6;
spec.physicalState=1:(cursor+3);
end
