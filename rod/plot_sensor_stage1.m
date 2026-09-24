function plot_sensor_stage1(folder,makeVideo)
%PLOT_SENSOR_STAGE1 Diagnostic component plots and optional simulation video.
if nargin<2, makeVideo=false; end
d=load(resolve_result_file(folder),'results'); r=d.results;
audit_sensor_stage1(folder);
b=load(fullfile(folder,'shape_only_baseline.mat'),'baseline'); baseline=b.baseline;
f=figure('Visible','off','Color','w','Position',[50 50 1250 820]);
cleanup=onCleanup(@()close(f));
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
x=r.forward.actuationMm;
for j=1:4
    nexttile;
    axisIndex=1+2*mod(j-1,2);
    if j<=2
        truth=r.forward.contactForceResultant; ours=r.ours.contactForceResultant;
        other=baseline.contactForceResultant; label='Contact';
    else
        truth=r.forward.tipLoad; ours=r.ours.tipForce; other=baseline.tipForce; label='Tip';
    end
    plot(x,truth(axisIndex,:),'k-','LineWidth',1.6); hold on;
    plot(x,ours(axisIndex,:),'o-','LineWidth',1.2,'MarkerSize',4);
    plot(x,other(axisIndex,:),':','LineWidth',1.1);
    axesNames='xyz'; ylabel(sprintf('%s F_%s (N)',label,axesNames(axisIndex)));
    xlabel('Commanded path (mm)'); grid on;
    if j==1, legend('Simulation truth','Fused MAP','Shape-only ablation','Location','best'); end
end
nexttile;
plot(x,vecnorm(r.ours.contactForceResultant-r.forward.contactForceResultant),'o-'); hold on;
plot(x,vecnorm(r.ours.tipForce-r.forward.tipLoad),'s-');
plot(x,vecnorm(r.ours.totalForceResultant-r.forward.totalForceResultant),'^-');
xlabel('Commanded path (mm)'); ylabel('Vector error norm (N)');
legend('Contact','Tip','Total','Location','best'); grid on;
nexttile;
location=r.ours.state(6,:); location(vecnorm(r.ours.contactForceResultant)<0.05)=NaN;
plot(x,location,'o-'); hold on;
plot(x,r.forward.contactArcLength,'k-','LineWidth',1.4);
location=baseline.contactArcLength; location(vecnorm(baseline.contactForceResultant)<0.05)=NaN;
plot(x,location,':');
xlabel('Commanded path (mm)'); ylabel('Contact arclength (mm)'); grid on;
sgtitle(strrep([r.config.scenarioName ' | sparse sensor history | offline simulation'],'_',' '));
exportgraphics(f,fullfile(folder,'component_comparison.png'),'Resolution',150);
clear cleanup;
if makeVideo
    % Render a copy; do not mutate the saved numerical evidence/configuration.
    r.config.video.enabled=true; results=r; %#ok<NASGU>
    renderPath=fullfile(folder,'render_results.mat');
    save(renderPath,'results','-v7.3');
    run_rod_plane_force_sensing_experiment('render-inverse',renderPath);
end
end
