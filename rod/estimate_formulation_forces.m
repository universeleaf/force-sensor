function output = estimate_formulation_forces(sensorInput, options)
%ESTIMATE_FORMULATION_FORCES Run the current Formulation-aligned estimator.
% This is the public replay entry for calibrated sensor packets. It applies
% the nonlinear 3-D Cosserat shooting map, the complete Scholtes MPCC and
% the two measured bending channels before calling the common estimator.
% The historical estimate_sensor_forces entry remains unchanged so old
% packets and archived comparisons keep their explicitly selected solver.
assert(isstruct(sensorInput) && all(isfield(sensorInput, {'tube','packet','config'})), ...
    'Provide sensorInput with tube, packet and estimator config.');
sensorInput.config = formulation_solver_config(sensorInput.config);
if sensorInput.packet.schemaVersion==2 || nargin>1
    if nargin<2, options=struct; end
    output=estimate_formulation_window(sensorInput,options);
    return;
end
output = estimate_sensor_forces(sensorInput);
output.method = 'formulation-aligned-cosserat-mpcc';
end
