function report=run_literature_strain_scenarios()
%RUN_LITERATURE_STRAIN_SCENARIOS Ferguson-parameter sensing adaptation.
% Reuses paper Sec. V material (207 GPa, 1.5 mm solid diameter), 1 m rod,
% N={5,15,50}, 10 microstrain. Our curved rod / one body + tip load / plane
% replace their straight rod / three distributed Gaussians. NOT a reproduction
% of the Ferguson estimator or its 4000 trials, nor a superiority test.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
source=fullfile(root,'out','completion','continuous');
d=load(fullfile(source,'sensor_input.mat')); template=d.sensorInput;
d=load(fullfile(source,'independent_forward.mat')); f=d.f;
a=5; diameter=1.5; young=207000; shear=young/2.6;
EI=young*pi*diameter^4/64; GJ=shear*pi*diameter^4/32;
forceScale=EI/template.tube.kb/a^2;
tube=template.tube;tube.s=tube.s*a;tube.uhat=tube.uhat/a;tube.kb=EI;tube.kt=GJ;
tube.rout=diameter/2;tube.rin=0;
sig=10e-6/(diameter/2); Ns=[5 15 50];
folder=fullfile(root,'out','formulation','literature_strain');if ~isfolder(folder),mkdir(folder);end
report=struct('state','running','runRecord',new_run_record(),'cases',{{}}, ...
    'reference','Ferguson et al., IEEE TRO 2024, DOI 10.1109/TRO.2024.3360950, Sec. V', ...
    'scope','Parameter-matched sparse-strain adaptation; own curved single-contact plus tip-load scene. No implementation/comparison of Ferguson algorithm.', ...
    'rodLengthMm',1000,'diameterMm',diameter,'youngModulusNPerMm2',young, ...
    'strainStdMicrostrain',10,'curvatureStdPerMm',sig,'seed',71);
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
for j=1:numel(Ns)
    input=template;input.tube=tube;input.config=formulation_solver_config(template.config);
    cfg=input.config; cfg.sensing.numFbgPoints=Ns(j);
    for group={'priorStd','processStd'}
        for field={'normalForceN','betaN','tipForceN'}
            cfg.forceSensor.(group{1}).(field{1})=cfg.forceSensor.(group{1}).(field{1})*forceScale;
        end
        for field={'planePointMm','sMm','lambda'}
            cfg.forceSensor.(group{1}).(field{1})=cfg.forceSensor.(group{1}).(field{1})*a;
        end
    end
    cfg.forceSensor.measurementStd.curvature=sig;
    cfg.forceSensor.measurementStd.planePointMm=cfg.forceSensor.measurementStd.planePointMm*a;
    cfg.forceSensor.historyCurvatureStdPerMm=sig;
    cfg.forceSensor.mpccForceScaleN=forceScale;cfg.forceSensor.mpccLengthScaleMm=a;
    cfg.forceSensor.mechanicsMomentToleranceNmm=2e-4*forceScale*a;
    input.config=cfg;
    packet=subset_sensor_packet(template.packet,4);
    idx=unique(round(linspace(1,numel(tube.s),Ns(j))));stream=RandStream('mt19937ar','Seed',71);
    packet.fbgIdx=idx;packet.sFbgMm=tube.s(idx);
    packet.curvaturePerMm=f.u(:,idx,4)/a+sig*randn(stream,3,numel(idx));
    packet.previousCurvaturePerMm=f.previousState{4}.u(:,idx)/a+sig*randn(stream,3,numel(idx));
    packet.curvaturePerMm(3,:)=tube.uhat(3,idx);packet.previousCurvaturePerMm(3,:)=tube.uhat(3,idx);
    packet.basePose(1:3,4)=packet.basePose(1:3,4)*a;
    packet.previousBasePose(1:3,4)=packet.previousBasePose(1:3,4)*a;
    packet.planePointMm=packet.planePointMm*a;packet.actuationMm=packet.actuationMm*a;
    input.packet=packet;
    fc=f.contactForceResultant(:,4)*forceScale;fe=f.tipLoad(:,4)*forceScale;
    fprintf('\nLITERATURE-PARAMETER ADAPTATION: N=%d, 10 microstrain\n',Ns(j));
    try
        output=estimate_sensor_forces(input);o=output.ours;
        baseline=estimate_shape_only_point_loads(tube,output.measurements,cfg);
        entry=struct('completed',true,'numSensors',Ns(j), ...
            'contactErrorN',norm(o.contactForceResultant-fc),'tipErrorN',norm(o.tipForce-fe), ...
            'totalErrorN',norm(o.totalForceResultant-fc-fe), ...
            'shapeOnlyContactErrorN',norm(baseline.contactForceResultant-fc), ...
            'shapeOnlyTipErrorN',norm(baseline.tipForce-fe), ...
            'shapeOnlyTotalErrorN',norm(baseline.totalForceResultant-fc-fe), ...
            'quality',output.quality,'seconds',o.frameSeconds);
        atomic_write_artifact(fullfile(folder,sprintf('N%d.mat',Ns(j))),'mat', ...
            struct('sensorInput',input,'output',output,'baseline',baseline, ...
            'trueContactN',fc,'trueTipN',fe,'report',entry));
    catch err
        entry=struct('completed',false,'numSensors',Ns(j),'error',err.message);
        fprintf(2,'%s\n',getReport(err,'extended','hyperlinks','off'));
    end
    report.cases{j}=entry;atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.state='complete';report.runRecord.state='complete';
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
