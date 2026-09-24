function report = test_rotation_equivariance()
%TEST_ROTATION_EQUIVARIANCE Compare actual independent wall/upward runs.
root=fileparts(fileparts(mfilename('fullpath')));
a=load(fullfile(root,'out','wall','results.mat'),'results'); a=a.results;
b=load(fullfile(root,'out','upward','inverse_results.mat'),'results'); b=b.results;
Q=b.config.provenance.rotationFromWall;
t=b.config.provenance.translationFromWallMm;
expectedShape=reshape(Q*reshape(a.forward.p,3,[])+t,size(a.forward.p));
report.forwardShapeMaxAbsMm=max(abs(b.forward.p-expectedShape),[],'all');
report.forwardForceMaxAbsN=max(abs(b.forward.totalForceResultant-Q*a.forward.totalForceResultant),[],'all');
report.inverseForceMaxVectorDifferenceN=max(vecnorm(b.ours.totalForceResultant-Q*a.ours.totalForceResultant,2,1));
report.inverseContactMaxVectorDifferenceN=max(vecnorm(b.ours.contactForceResultant-Q*a.ours.contactForceResultant,2,1));
report.inverseShapeMaxAbsMm=max(abs(b.ours.p-reshape(Q*reshape(a.ours.p,3,[])+t,size(a.ours.p))),[],'all');
assert(report.forwardShapeMaxAbsMm<1e-6 && report.forwardForceMaxAbsN<1e-6, ...
    'Rigidly rotated forward simulation changed physical results.');
% Finite differences, state scaling and local optimizer introduce numerical
% variation. This practical tolerance does not assert exact EKF invariance.
assert(report.inverseForceMaxVectorDifferenceN<0.1, ...
    'Inverse rotation difference exceeds 0.1 N.');
fid=fopen(fullfile(root,'out','upward','rotation_check.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true)); fclose(fid);
disp(report);
end
