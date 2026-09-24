function test_plane_contact_geometry()
% A candidate contact can be feasible while another part of the rod penetrates.
% Both continuation and active-set polishing must retain rod admissibility.
m=4; x=zeros(15,1); x(2)=.2; x(6)=.5; x(7)=1;
lo=-10*ones(15,1); hi=10*ones(15,1);
lo(6)=.1; hi(6)=1.9; lo(7)=.8; lo(8:12)=0;
cfg=struct('numFrictionDirs',m,'currentMu',0,'linearizedSolveMaxIter',80, ...
    'mpccRelaxations',[1e-2 1e-4 1e-6 1e-8],'mpccLengthScaleMm',1, ...
    'mpccForceScaleN',1,'showProgress',false,'complementaritySolver','scholtes', ...
    'planeContactGeometry','rod');
target=zeros(15,1); target(2)=.4; target(6)=1.5; target(7)=1;
objective=@(v)sum((v-target).^2);
[answer,info]=solve_contact_mpcc(x,objective,@decode,{lo,hi},ones(15,1),cfg);
assert(info.exitflag>0 && answer(6)<=1+1e-6 && abs(answer(2))<1e-6, ...
    'rod:PlaneGeometryRegression','Full MPCC accepted rod penetration or a nontangent body contact.');
mode=struct('name','frictionless-contact','activeDirection',[]);
[answer,info]=solve_contact_mode_map(x,objective,@decode,{lo,hi},ones(15,1),cfg,mode);
assert(info.exitflag>0 && answer(6)<=1+1e-6 && abs(answer(2))<1e-6, ...
    'rod:PlaneGeometryRegression','Mode polishing discarded rod geometry.');
cfg.planeContactGeometry='point-only';
[answer,info]=solve_contact_mpcc(x,objective,@decode,{lo,hi},ones(15,1),cfg);
assert(info.exitflag>0 && abs(answer(6)-1.5)<1e-5 && abs(answer(2)-.4)<1e-5, ...
    'Historical point-only behavior changed.');
cfg.planeContactGeometry='rod'; d=decode(x);
% A hard fn*tangent=0 equality traps the fn=0 seed despite a much better
% contact optimum. The continuation must relax this product too.
start=x; start(7)=0; start(2)=.1; lo(7)=0;
target(6)=.5; target(2)=.1;
[answer,info]=solve_contact_mpcc(start,@(v)sum((v-target).^2),@decode,{lo,hi},ones(15,1),cfg);
assert(info.exitflag>0 && answer(7)>.99 && abs(answer(2))<1e-6, ...
    'Tangency continuation was trapped at a separated initial state.');
[c,e]=plane_contact_constraints(d,cfg);
Q=[0 0 1;1 0 0;0 1 0]; shift=[9;-2;4];
d.n=Q*d.n; d.p1=Q*d.p1+shift;
d.mechanics.collisionP=Q*d.mechanics.collisionP+shift;
d.mechanics.contactTangent=Q*d.mechanics.contactTangent;
[cr,er]=plane_contact_constraints(d,cfg);
assert(norm(c-cr)+norm(e-er)<1e-12,'Geometry constraints depend on world coordinates.');
for s=[0 2]
    d=decode(x); d.s1=s; [~,e]=plane_contact_constraints(d,cfg);
    assert(abs(e)<1e-12,'Endpoint contact incorrectly requires tangency.');
end
d=decode(x); d.fn=0; [~,e]=plane_contact_constraints(d,cfg);
assert(abs(e)<1e-12,'Separated rod incorrectly requires tangency.');
disp('Plane contact geometry and full/reduced constraint retention passed.');
    function d=decode(v)
        mechanics=struct('collisionS',[0 2],'collisionP',[0 1;0 0;v(3) v(3)+1-v(6)], ...
            'contactTangent',[1;0;v(2)],'rodArcBoundsMm',[0 2]);
        d=struct('n',[0;0;1],'p1',zeros(3,1),'s1',v(6),'fn',v(7), ...
            'beta',v(8:11),'lambda',v(12),'gap',v(3), ...
            'D',[1 0 -1 0;0 1 0 -1;0 0 0 0],'vTangential',zeros(3,1), ...
            'frictionW',v(12)*ones(4,1),'frictionConeSlack',-sum(v(8:11)), ...
            'mechanics',mechanics);
    end
end
