function test_environment_covariance()
%TEST_ENVIRONMENT_COVARIANCE Preserve correlated plane evidence in Eq. (11).
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(root,'out','stage1','video_noise_gated_01','sensor_input.mat'),'sensorInput');
input=d.sensorInput; nt=numel(input.packet.timeSeconds);
A=diag([0.2 0.3 0.4 0.01 0.02 0.03]); A(1,4)=0.05;
C=A*A'; input.packet.planeCovariance=repmat(C,1,1,nt);
m=measurements_from_sensor_packet(input.tube,input.packet,input.config);
assert(isfield(m,'environmentWhitening'), ...
    'rod:EnvironmentCovarianceDropped','Plane covariance was silently ignored.');
W=m.environmentWhitening(:,:,1); r=[1;-2;3;0.01;0.02;-0.03];
assert(abs(norm(W*r)^2-r'*(C\r))<1e-9,'Whitening lost correlation in the plane likelihood.');
bad=input.packet; bad.planeCovariance(1,1,1)=-1;
mustReject(@()measurements_from_sensor_packet(input.tube,bad,input.config),'rod:InvalidPlaneCovariance');
bad=input.packet; bad.planeCovariance(1,2,1)=1;
mustReject(@()measurements_from_sensor_packet(input.tube,bad,input.config),'rod:InvalidPlaneCovariance');
disp('ENVIRONMENT_COVARIANCE_TEST_PASSED');
end

function mustReject(f,id)
try, f(); catch err, assert(strcmp(err.identifier,id),'Unexpected error: %s',err.message); return; end
error('Expected rejection: %s',id);
end
