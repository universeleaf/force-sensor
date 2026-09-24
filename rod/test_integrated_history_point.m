function diagnostics=test_integrated_history_point()
% Expose the artificial tangent jump of linear position interpolation, then
% check the partial exponential against an analytic circular arc.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
s=linspace(0,80,81);curvature=0.02;tube=CreatTube(80,s,repmat([0;curvature;0],1,numel(s)));
[R,p]=integrate_curvature_field(tube,tube.uhat,tube.T_base);
shape=struct('R',R,'p',p,'u',tube.uhat);arc=40;h=1e-4;
old=@(a)interp1(s,p',a,'linear')';
oldLeft=(old(arc)-old(arc-h))/h;oldRight=(old(arc+h)-old(arc))/h;
center=sample_integrated_shape(tube,shape,arc);
left=(center-sample_integrated_shape(tube,shape,arc-h))/h;
right=(sample_integrated_shape(tube,shape,arc+h)-center)/h;
diagnostics=struct('linearTangentJump',norm(oldRight-oldLeft), ...
    'integratedTangentJump',norm(right-left),'maximumAnalyticPointErrorMm',0);
assert(diagnostics.linearTangentJump>0.01,'Fixture did not expose the interpolation kink.');
assert(diagnostics.integratedTangentJump<3e-6,'Integrated history has a grid-knot tangent jump.');
for a=[0,0.3,39.8,40,40.2,79.9,80]
    point=sample_integrated_shape(tube,shape,a);
    expected=[(1-cos(curvature*a))/curvature;0;sin(curvature*a)/curvature];
    diagnostics.maximumAnalyticPointErrorMm=max(diagnostics.maximumAnalyticPointErrorMm,norm(point-expected));
end
assert(diagnostics.maximumAnalyticPointErrorMm<1e-10,'Partial integration does not match the analytic circle.');
disp(diagnostics);disp('INTEGRATED_HISTORY_POINT_CHECKS_PASSED');
end
