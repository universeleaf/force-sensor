function report=run_formulation_benchmark(frameIndices)
%RUN_FORMULATION_BENCHMARK Same independent truth/observations, three solvers.
% Existing continuous truth is a different planar ODE boundary-value solver.
% Recompute the legacy control: subsets must share the same temporal prior.
if nargin<1,frameIndices=1:5;end
root=fileparts(fileparts(mfilename('fullpath')));
source=fullfile(root,'out','completion','continuous');
if ~isfile(fullfile(source,'sensor_input.mat')),run_independent_truth_benchmark('continuous');end
data=load(fullfile(source,'sensor_input.mat'),'sensorInput'); input=data.sensorInput;
truth=load(fullfile(source,'independent_forward.mat'),'f'); f=truth.f;
input.packet=subset_sensor_packet(input.packet,frameIndices);
folder=fullfile(root,'out','formulation'); if ~isfolder(folder),mkdir(folder);end
report=struct('state','running','frameIndices',frameIndices,'runRecord',new_run_record(), ...
    'scope','Fixed independent continuous single-contact frictionless truth; same sparse input/prior; local nonlinear solvers. Not a friction or literature-superiority result.', ...
    'inputSha256',file_sha256(fullfile(source,'sensor_input.mat')),'cases',{{}});
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
names={'legacy','nonlinear_mechanics','full_formulation'};
for j=1:3
    current=input;
    if j>=2
        current.config=formulation_solver_config(current.config);
        if j==2
            current.config.forceSensor.complementaritySolver='active-set';
            current.config.forceSensor.subproblemCoordinates='mode-reduced';
        end
    else
        current.config.forceSensor.mechanicsModel='predecessor-linearized';
    end
    % Isolate the original mechanics/MPCC comparison. The separate geometry
    % benchmark tests rod admissibility on paired archived observations.
    current.config.forceSensor.planeContactGeometry='point-only';
    current.config.forceSensor.useNonlinearShapeSeed=false;
    current.config.forceSensor.useObservationScaling=false;
    current.config.forceSensor.showProgress=true;
    fprintf('\nFORMULATION COMPARISON %s\n',names{j});
    timer=tic;
    try
        output=estimate_sensor_forces(current); o=output.ours;
        entry=struct('name',names{j},'completed',true,'seconds',toc(timer), ...
            'contactRmseN',rmse(o.contactForceResultant-f.contactForceResultant(:,frameIndices)), ...
            'tipRmseN',rmse(o.tipForce-f.tipLoad(:,frameIndices)), ...
            'totalRmseN',rmse(o.totalForceResultant-f.totalForceResultant(:,frameIndices)), ...
            'maxComplementarityResidual',max(output.quality.complementarityResidual), ...
            'maxActiveConstraintResidual',max(o.activeConstraintResidual), ...
            'quality',output.quality,'frameSeconds',o.frameSeconds);
        atomic_write_artifact(fullfile(folder,[names{j},'.mat']),'mat', ...
            struct('sensorInput',current,'output',output,'report',entry,'frameIndices',frameIndices));
    catch err
        entry=struct('name',names{j},'completed',false,'error',err.message,'seconds',toc(timer));
        fprintf(2,'%s failed: %s\n',names{j},getReport(err,'extended','hyperlinks','off'));
    end
    report.cases{j}=entry;
    atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.state='complete'; report.runRecord.state='complete';
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
disp(report);
end
function v=rmse(e),v=sqrt(mean(sum(e.^2,1)));end
