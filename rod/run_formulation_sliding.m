function report=run_formulation_sliding()
%RUN_FORMULATION_SLIDING Independent sliding truth with an unknown tip force.
% Translating an equilibrium parallel to an infinite plane preserves static
% balance. The preceding rod is translated +x; current slip is -x, so +x
% Coulomb force dissipates work. Mode/force truth never enters the estimator.
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
d=load(fullfile(root,'out','completion','continuous','sensor_input.mat')); template=d.sensorInput;
d=load(fullfile(root,'out','completion','continuous','independent_forward.mat')); model=d.model;
model.baseXZ=[0;18]; model.tangentialNormalRatio=0.3;
eq=solve_planar_contact_shooting(model);
tube=template.tube; ns=numel(tube.s); u=tube.uhat;
u(2,:)=eq.curvaturePerMm; p=zeros(3,ns); p([1 3],:)=eq.pXZ;
folder=fullfile(root,'out','formulation','sliding');if ~isfolder(folder),mkdir(folder);end
report=struct('state','running','runRecord',new_run_record(),'cases',{{}}, ...
    'scope','Independent prescribed sliding branch; two paired-sensor cases, not a stick/slip transition or Monte Carlo study.');
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
steps=[1 .02]; noise=[0 5e-5];
for k=1:2
    input=template; input.packet=subset_sensor_packet(template.packet,4);
    input.config=formulation_solver_config(input.config);
    input.config.forceSensor.historyCurvatureStdPerMm=noise(k);
    packet=input.packet; idx=packet.fbgIdx;
    stream=RandStream('mt19937ar','Seed',93);
    packet.curvaturePerMm=u(:,idx)+noise(k)*randn(stream,3,numel(idx));
    packet.previousCurvaturePerMm=u(:,idx)+noise(k)*randn(stream,3,numel(idx));
    packet.curvaturePerMm(3,:)=tube.uhat(3,idx);packet.previousCurvaturePerMm(3,:)=tube.uhat(3,idx);
    packet.basePose=eye(4); packet.basePose(3,4)=18;
    packet.previousBasePose=packet.basePose; packet.previousBasePose(1,4)=steps(k);
    packet.frictionMu=0.3; packet.planePointMm=[0;0;160];packet.planeNormal=[0;0;-1];
    input.packet=packet;
    fc=zeros(3,1); fc([1 3])=eq.contactResultantXZ; fe=zeros(3,1);fe([1 3])=eq.tipForceXZ;
    fprintf('\nINDEPENDENT SLIDING: step %.3f mm, sigma %.2g /mm\n',steps(k),noise(k));
    try
        output=estimate_sensor_forces(input);o=output.ours;
        entry=struct('completed',true,'stepMm',steps(k),'curvatureStdPerMm',noise(k), ...
            'contactErrorN',norm(o.contactForceResultant-fc),'tipErrorN',norm(o.tipForce-fe), ...
            'totalErrorN',norm(o.totalForceResultant-fc-fe), ...
            'estimatedContactN',o.contactForceResultant,'trueContactN',fc, ...
            'mode',{o.complementarityMode},'quality',output.quality,'seconds',o.frameSeconds);
        atomic_write_artifact(fullfile(folder,sprintf('case%d.mat',k)),'mat', ...
            struct('sensorInput',input,'output',output,'truth',eq,'report',entry));
    catch err
        entry=struct('completed',false,'stepMm',steps(k),'error',err.message);
        fprintf(2,'%s\n',getReport(err,'extended','hyperlinks','off'));
    end
    report.cases{k}=entry; atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.state='complete';report.runRecord.state='complete';
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
