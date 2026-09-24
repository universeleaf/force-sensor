function report = run_mesh_convergence(stepsMm)
% Multi-level convergence evidence for the actual frictionless bent rod.
% This protocol does not claim mesh convergence until TWO successive changes
% meet predeclared absolute reaction/position tolerances on every test frame.
if nargin<1,stepsMm=[1 0.5 0.25 0.125];end
assert(isnumeric(stepsMm)&&isreal(stepsMm)&&numel(stepsMm)>=3&& ...
    all(isfinite(stepsMm))&&all(stepsMm>0)&&all(diff(stepsMm)<0)&&stepsMm(1)==1);
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','completion','mesh');if ~isfolder(folder),mkdir(folder);end
d=load(fullfile(root,'out','stage2','independent_video_tip_gated','independent_forward.mat'));
push=[14 18 20]; nLevels=numel(stepsMm); equilibria=cell(nLevels,3);
report=struct('scope','Independent frictionless planar local equilibria; actual bent-rod geometry, three push positions. No global-minimum or frictional convergence claim.', ...
    'stepsMm',stepsMm,'pushMm',push,'forceToleranceN',0.02,'tipToleranceMm',0.01, ...
    'complete',false,'meshConverged',false,'levels',{{}});
atomic_write_artifact(fullfile(folder,'convergence.json'),'json',report);
for level=1:nLevels
    stats=struct('stepMm',stepsMm(level),'contactForcesXZ',zeros(2,3),'forceChangeN',nan(1,3), ...
        'tipChangeMm',nan(1,3),'stationarityInfNmm',zeros(1,3),'seconds',zeros(1,3));
    for frame=1:3
        if level==1
            result=d.equilibria{2,frame+2};
        else
            prior=equilibria{level-1,frame}; model=d.model;
            model.sMm=0:stepsMm(level):200;
            assert(abs(model.sMm(end)-200)<1e-10,'Mesh spacing must divide the rod length.');
            mid=(model.sMm(1:end-1)+model.sMm(2:end))/2;
            model.intrinsicCurvaturePerMm=double(mid>=120)/30;
            model.baseXZ=[0;push(frame)];
            initial=interp1(prior.sMm,prior.thetaRad,model.sMm(2:end),'pchip');
            result=solve_planar_energy_rod(model,initial);
            assert(result.exitflag>0&&result.stationarityInfNmm<1e-5&& ...
                result.maxPenetrationMm<1e-6&&result.complementarityNmm<1e-5, ...
                'Independent equilibrium did not pass local equilibrium checks.');
            stats.forceChangeN(frame)=norm(result.contactResultantXZ-prior.contactResultantXZ);
            stats.tipChangeMm(frame)=norm(result.pXZ(:,end)-prior.pXZ(:,end));
        end
        equilibria{level,frame}=result;
        stats.contactForcesXZ(:,frame)=result.contactResultantXZ;
        stats.stationarityInfNmm(frame)=result.stationarityInfNmm;
        stats.seconds(frame)=result.seconds;
        fprintf('Mesh %.4f mm, push %.1f: force change %.6g N, stationarity %.3g\n', ...
            stepsMm(level),push(frame),stats.forceChangeN(frame),result.stationarityInfNmm);
    end
    report.levels{level}=stats;
    atomic_write_artifact(fullfile(folder,'equilibria.mat'),'mat',struct('equilibria',{equilibria},'report',report));
    atomic_write_artifact(fullfile(folder,'convergence.json'),'json',report);
end
last=[report.levels{end-1}.forceChangeN report.levels{end}.forceChangeN];
tip=[report.levels{end-1}.tipChangeMm report.levels{end}.tipChangeMm];
report.complete=true;report.meshConverged=all(last<report.forceToleranceN)&&all(tip<report.tipToleranceMm);
atomic_write_artifact(fullfile(folder,'equilibria.mat'),'mat',struct('equilibria',{equilibria},'report',report));
atomic_write_artifact(fullfile(folder,'convergence.json'),'json',report);
disp(report);
end
