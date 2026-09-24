function report = test_sensor_contracts()
% Reject ambiguous or corrupt observations before mechanics/optimization.
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(root,'out','stage1','video_seeded','sensor_input.mat'),'sensorInput');
input=d.sensorInput; packet=input.packet; cfg=input.config; tube=input.tube;
cases={}; names={};
bad=cfg; bad.sensing.curvatureInterpolation='intrisic-delta';
add('unknown interpolation',@()measurements_from_sensor_packet(tube,packet,bad));
bad=packet; bad.environmentTimeSeconds=packet.timeSeconds+1;
add('stale environment',@()measurements_from_sensor_packet(tube,bad,cfg));
bad=packet; bad.sFbgMm(3)=NaN;
add('NaN sensor coordinate',@()measurements_from_sensor_packet(tube,bad,cfg));
bad=packet; bad.previousTimeSeconds(2)=bad.timeSeconds(1);
bad.previousCurvaturePerMm(:,:,2)=bad.curvaturePerMm(:,:,1)+1e-4;
bad.previousBasePose(:,:,2)=bad.basePose(:,:,1);
add('conflicting repeated sample',@()measurements_from_sensor_packet(tube,bad,cfg));
bad=tube; bad.uhat(1,3)=NaN;
add('NaN intrinsic calibration',@()measurements_from_sensor_packet(bad,packet,cfg));
bad=packet; bad.curvaturePerMm(1,3,1)=1i*1e-4;
add('complex curvature',@()measurements_from_sensor_packet(tube,bad,cfg));
bad=cfg; bad.sensing.shapeSmoothing=-1;
add('negative smoothing',@()measurements_from_sensor_packet(tube,packet,bad));
rejected=false(size(cases)); errors=cell(size(cases));
for k=1:numel(cases)
    try, cases{k}(); catch err, rejected(k)=true; errors{k}=err.message; end
end
report=struct('names',{names},'rejected',rejected,'errors',{errors},'passed',all(rejected));
disp(report);
assert(report.passed,'rod:SensorContractRegression','Accepted invalid input: %s',strjoin(names(~rejected),', '));
    function add(name,callback)
        names{end+1}=name; cases{end+1}=callback;
    end
end
