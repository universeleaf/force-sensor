function test_latent_fbg_history()
% Full history coordinates, exact zero correction, and nuisance polishing.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
cfg=formulation_solver_config(run_rod_plane_force_sensing_experiment('sensor-config'));
cfg.forceSensor.historyCurvatureStdPerMm=5e-5;
cfg.forceSensor.historyStateMode='latent-fbg';validate_sensor_config(cfg);
s=linspace(0,80,81);tube=CreatTube(80,s,zeros(3,numel(s)));
idx=unique(round(linspace(1,numel(s),24)));
u=zeros(3,numel(idx));u(2,:)=0.006+0.001*sin(s(idx)/80*pi);
previous=reconstruct_sensor_curvature(tube,u,idx,tube.T_base,cfg);
model=latent_fbg_history(tube,previous,cfg);q=zeros(model.count,1);
assert(model.count==48,'A predecessor FBG channel was truncated.');
zero=model.shape(q);
assert(isequal(zero,previous),'Zero latent correction changed the measured history.');
q(:)=linspace(-1,1,48);updated=model.shape(q);
assert(max(abs(reshape((updated.sparseU(1:2,:)-u(1:2,:))/5e-5,[],1)-q))<1e-12);
assert(isequal(updated.sparseU(3,:),u(3,:)),'Unobserved torsion was changed.');
assert(norm(updated.p-previous.p,'fro')>0&&isequal(model.shape(q),updated));
assert(isequal(model.shape(zeros(48,1)),previous),'History cache changed a zero correction.');
bad=cfg;bad.forceSensor.historyStateMode='misspelled';reject(bad);
bad=cfg;bad.forceSensor.complementaritySolver='active-set';reject(bad);
% A nuisance coordinate must be optimized in BOTH full MPCC and polishing.
% Under a separated rod all mechanical constraints are independent of q;
% the analytic optimum of .5*(q-2)^2 is q=2.
m=4;nx=11+m;x=zeros(nx+1,1);x(6)=1;
lo=-inf(size(x));hi=inf(size(x));lo(7:8+m)=0;
settings=cfg.forceSensor;settings.numFrictionDirs=m;settings.currentMu=0;
settings.planeContactGeometry='point-only'; % analytic nuisance fixture has no rod
settings.showProgress=false;settings.linearizedSolveMaxIter=30;
mode=struct('name','no-contact','activeDirection',[]);
[answer,info]=solve_contact_mode_map(x,@objective,@decode,{lo,hi},ones(size(x)),settings,mode);
assert(info.exitflag>0&&abs(answer(end)-2)<1e-5,'Polishing froze the latent history coordinate.');
[answer,info]=solve_contact_mpcc(x,@objective,@decode,{lo,hi},ones(size(x)),settings);
assert(info.exitflag>0&&abs(answer(end)-2)<1e-5,'Full MPCC lost the latent history coordinate.');
disp('LATENT_HISTORY_CHECKS_PASSED: all 48 channels, exact reconstruction, cache, config and nuisance polishing.');
    function value=objective(v),value=0.5*(v(end)-2)^2+0.5*sum(v(1:nx).^2);end
    function d=decode(v)
        d=struct('n',[0;0;1],'D',[1 0 -1 0;0 1 0 -1;0 0 0 0], ...
            'vTangential',zeros(3,1),'gap',1,'fn',v(7),'beta',v(8:11), ...
            'lambda',v(12),'frictionW',ones(4,1)*v(12),'frictionConeSlack',-sum(v(8:11)));
    end
    function reject(config)
        caught=false;
        try,validate_sensor_config(config);catch err,caught=strcmp(err.identifier,'rod:InvalidSensorConfig');end
        assert(caught,'Invalid latent history configuration was accepted.');
    end
end
