function results = force(scenario, quickMode)
%FORCE Run the current rod-plane experiment from the workspace root.
% force() / force('wall')   : separated body contact, strict fmincon solver
% force('senior')           : horizontal plane, contact at the tip
% force('upward')           : entire wall scene rotated 90 deg, upward push
% force('video')            : image-derived geometry resembling fail.mp4
% force('sensor-video')     : sparse paired history, video geometry, reduced SQP
% force('sensor-tip')       : sparse paired history, unknown nonzero tip load
% force('sensor-noise')     : fixed six-frame noise stress protocol (failures kept)
% force('noise')            : uncertainty-aware replay of the fixed noise packets
% force('depth')            : synthetic depth images -> plane -> force protocol
% force('depth-ablation')   : isolate plane covariance correlations, saved input
% force('independent')      : independent nonlinear frictionless truth test
% force('continuous')       : continuous ODE single-contact truth test
% force('formulation')      : nonlinear 3-D Cosserat + complete MPCC comparison
% force('formulation-slide'): independent sliding truth, exact friction solve
% force('literature-strain'): Ferguson-parameter sparse-strain scenario adaptation
% force('demos')           : multiple continuous-rod/environment contact demos
% force('history')         : paired latent-history MAP vs fixed history
% force('temporal-window'): joint short-window Cosserat MAP smoother
% force('statistics')      : multi-seed/multi-noise conditional coverage protocol
% force('baselines')       : same-observation fair baseline comparison
% force('mismatch')        : multi-contact/nonplanar/friction mismatch stress tests
% force('realtime')        : wall-clock replay benchmark
% force('geometry')        : paired rod-plane admissibility fix on saved demos
% force('accuracy')        : archived-input geometry + calibrated FBG comparison
% force('mesh')             : multi-level independent-truth convergence audit
% force('check')            : project checks including full sensor replay
% force('wall', true)       : approximate pipeline smoke test
% Historical standalone examples are preserved in legacy/.
if nargin < 1, scenario = 'wall'; end
if nargin < 2, quickMode = false; end
if islogical(scenario)
    quickMode = scenario; scenario = 'wall';
end
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'rod'));
switch lower(char(scenario))
    case 'wall'
        results = simu_rod_plane_displacement_force_sensing(quickMode);
    case 'senior'
        results = simu_rod_plane_senior_geometry_force_sensing(quickMode);
    case 'upward'
        results = simu_rod_plane_upward_force_sensing(quickMode);
    case 'video'
        results = simu_rod_plane_video_geometry_force_sensing(quickMode);
    case 'sensor-video'
        results = run_sensor_stage1('video_zero');
    case 'sensor-tip'
        results = run_sensor_stage1('wall_tip');
    case 'sensor-noise'
        results = run_sensor_noise_protocol();
    case 'noise'
        results = run_noise_aware_protocol();
    case 'depth'
        results = run_depth_protocol();
    case 'depth-ablation'
        results = run_depth_protocol('ablation');
    case 'independent'
        results = run_independent_truth_benchmark();
    case 'continuous'
        results = run_independent_truth_benchmark('continuous');
    case 'formulation'
        results = run_formulation_benchmark();
    case 'formulation-slide'
        results = run_formulation_sliding();
    case 'literature-strain'
        results = run_literature_strain_scenarios();
    case 'demos'
        results = run_contact_demo_suite();
    case 'history'
        results = run_history_map_benchmark();
    case {'temporal-window','window'}
        if islogical(quickMode), W=3; else, W=quickMode; end
        sensorInput=loadDemoSensorInput(); W=min(W,numel(sensorInput.packet.timeSeconds));
        results = estimate_temporal_window_forces(sensorInput,W,struct('showProgress',true,'frameIndices',1:W));
    case {'statistics','submission-stats'}
        results = run_submission_statistics(quickMode);
    case {'baselines','fair-baselines'}
        results = run_fair_baseline_protocol(quickMode);
    case {'mismatch','model-mismatch'}
        results = run_model_mismatch_protocol(quickMode);
    case {'realtime','real-time'}
        results = run_realtime_benchmark(quickMode);
    case 'geometry'
        results = run_plane_geometry_benchmark();
    case 'accuracy'
        results = run_accuracy_benchmark();
    case 'mesh'
        results = run_mesh_convergence();
    case 'check'
        results = run_project_checks(true);
    otherwise
        error('force:UnknownScenario', ...
            ['Use wall, senior, upward, video, sensor-video, sensor-tip, ', ...
             'sensor-noise, noise, depth, depth-ablation, independent, ', ...
             'continuous, formulation, formulation-slide, literature-strain, demos, history, temporal-window, ', ...
             'statistics, baselines, mismatch, realtime, geometry, accuracy, mesh or check.']);
end
end

function sensorInput=loadDemoSensorInput()
rootDir=fileparts(mfilename('fullpath'));folder=fullfile(rootDir,'out','demos');
source=jsondecode(fileread(fullfile(folder,'comparison.json')));
% Use the clean sliding packet for the default temporal smoke path. The
% noisy packet remains available through estimate_temporal_window_forces when
% explicitly passed, but it is a poor default because one frame can take
% minutes before the window optimizer even starts.
preferred={'sliding_clean','ceiling_hook','sliding_noisy'};saved=[];
for i=1:numel(preferred)
    idx=find(strcmp({source.cases.id},preferred{i}) & [source.cases.completed],1);
    if ~isempty(idx),saved=source.cases(idx);break;end
end
assert(~isempty(saved),'rod:MissingDemo','Run force(''demos'') before force(''temporal-window'').');
d=load(fullfile(folder,saved.artifactFolder,'input_and_truth.mat'),'sensorInput');sensorInput=d.sensorInput;
end
