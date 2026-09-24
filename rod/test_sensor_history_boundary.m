function test_sensor_history_boundary()
% Poison hidden dense truth while retaining identical sensor samples.
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(root,'out','video','inverse_results.mat'),'results'); r=d.results;
cfg=r.config; cfg.sensing.samplePeriodSeconds=0.02;
cfg.sensing.planeNormalNoiseStdDeg=0;
tube=make_experiment_tube(cfg);
p=simulate_sensor_packet(tube,r.forward,cfg);
f=r.forward; idx=p.fbgIdx;
other=setdiff(1:numel(tube.s),idx);
f.p(:)=NaN; f.R(:)=NaN; f.contactForceResultant(:)=NaN;
f.totalForceResultant(:)=NaN; f.tipLoad(:)=NaN;
f.contactArcLength(:)=NaN; f.u(:,other,:)=NaN;
for k=1:numel(f.previousState)
    f.previousState{k}.p(:)=NaN; f.previousState{k}.R(:)=NaN;
    f.previousState{k}.u(:,other)=NaN;
end
q=simulate_sensor_packet(tube,f,cfg);
assert(isequaln(p,q),'Sensor packet reads dense truth or loads outside allowed samples.');
m=measurements_from_sensor_packet(tube,p,cfg);
assert(all(isfinite(m.p(:))) && all(isfinite(m.previousShape{end}.p(:))));
assert(max(abs(m.historyIntervalSeconds-0.02))<1e-10);
assert(any(m.predictionIntervalSeconds>m.historyIntervalSeconds+0.02), ...
    'Test must distinguish history interval from filter update interval.');
bad=p; bad.previousTimeSeconds(end)=bad.timeSeconds(end)+1;
rejected=false;
try, measurements_from_sensor_packet(tube,bad,cfg); catch, rejected=true; end
assert(rejected,'Future sensor history was accepted.');
cfg.sensing.curvatureNoiseStd=5e-5;
a=simulate_sensor_packet(tube,r.forward,cfg);
b=simulate_sensor_packet(tube,r.forward,cfg);
assert(isequaln(a,b),'Seeded sensor packets are not reproducible.');
assert(~isequal(a.curvaturePerMm,p.curvaturePerMm),'Noise configuration had no effect.');
for k=1:numel(a.timeSeconds)
    j=find(abs(a.previousTimeSeconds-a.timeSeconds(k))<1e-10);
    for h=j
        assert(isequal(a.curvaturePerMm(:,:,k),a.previousCurvaturePerMm(:,:,h)), ...
            'The same sensor timestamp was independently sampled twice.');
    end
end
bad=p; bad.basePose(1,:,1)=-bad.basePose(1,:,1);
rejected=false;
try, measurements_from_sensor_packet(tube,bad,cfg); catch, rejected=true; end
assert(rejected,'Improper/reflected rotation was accepted.');
disp('Sensor boundary, time alignment, causal history and seeded noise checks passed.');
end
