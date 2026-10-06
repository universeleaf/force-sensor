function report=run_bottleneck_audit(task,options)
% Compatibility entry point using direct calls; no shadow solvers are created.
if nargin<2,options=struct;end
switch task
 case 'prepare',report=prepare_inputs;
 case 'sliding',report=run_baseline('sliding_clean',options);
 case 'planar',report=run_baseline('s_channel_two_contact',options);
 case 'window',report=run_baseline('two_contact',options);
 case {'sliding_frozen','window_frozen'}
  error('tsfs:HistoricalOnly','This historical ablation required modified solver copies. Its evidence is preserved; new runs require approved upstream hooks.');
 otherwise,error('tsfs:AuditTask','Choose prepare, sliding, planar, or window.');
end
end
