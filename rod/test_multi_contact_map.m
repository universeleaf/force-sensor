function test_multi_contact_map()
%TEST_MULTI_CONTACT_MAP Regression for the independent two-contact map.
s=linspace(0,100,81);u=zeros(3,numel(s));u(2,s>=60)=1/40;
tube=make_experiment_tube(struct('exposedLengthMm',100, ...
    'rod',struct('sMm',s,'intrinsicCurvaturePerMm',u)));tube.T_base=eye(4);
opts=struct('relativeTolerance',1e-5,'momentToleranceNmm',1e-3, ...
    'collisionStepMm',1,'maxRhsEvaluations',1000000);
fc=[[0;-0.10;0.02],[0;-0.06;-0.01]];
shape=solve_cosserat_multi_contact_map(tube,[62,84],fc,[0.03;-0.02;0.01],opts);
assert(isscalar(shape)&&shape.tipMomentResidualNmm<1e-3,'Multi-contact shooting residual is too large.');
assert(isequal(size(shape.contactPoints),[3 2]),'Multi-contact point shape is invalid.');
end
