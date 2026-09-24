function plot_friction(r)
%PLOT_FRICTION Show force-on-rod convention, cone and displacement checks.
nt=size(r.ours.state,2); k=1:nt;
t=r.config.forward.slideDirection(:);
trueFn=vecnorm(r.forward.normalForceResultant,2,1);
estFn=vecnorm(r.ours.normalForceResultant,2,1);
trueFt=t'*r.forward.frictionForceResultant;
estFt=t'*r.ours.frictionForceResultant;
mu=r.forward.frictionMu;
fig=figure('Visible','off','Color','w','Position',[100 100 1200 760]);
cleanup=onCleanup(@() close(fig));
tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
nexttile(tl); hold on; grid on;
plot(k,trueFt,'k-','LineWidth',1.8);
plot(k,estFt,'--','Color',[0.7 0.2 0.65],'LineWidth',1.8);
plot(k,mu.*estFn,':','Color',[0.45 0.45 0.45]);
plot(k,-mu.*estFn,':','Color',[0.45 0.45 0.45],'HandleVisibility','off');
yline(0,'Color',[0.7 0.7 0.7]);
xlabel('Simulation frame'); ylabel('Friction along commanded motion [N]');
legend('True friction','Estimated friction','+/- mu F_n','Location','best');
title('Force exerted by plane on rod');
nexttile(tl); hold on; grid on;
active=mu>0 & trueFn>0.05;
if any(active)
    ratioTrue=vecnorm(r.forward.frictionForceResultant(:,active),2,1)./(mu(active).*trueFn(active));
    ratioEst=vecnorm(r.ours.frictionForceResultant(:,active),2,1)./max(mu(active).*estFn(active),eps);
    plot(k(active),ratioTrue,'ko-','LineWidth',1.4);
    plot(k(active),ratioEst,'s--','Color',[0.7 0.2 0.65],'LineWidth',1.4);
end
yline(1,'r:','Cone boundary','LabelHorizontalAlignment','left');
ylim([0 1.08]);
xlabel('Simulation frame'); ylabel('||F_t|| / (mu F_n)');
title('Coulomb cone: ratio must be <= 1');
legend('True','Estimated','Location','best');
nexttile(tl); hold on; grid on;
observed=zeros(1,nt); predicted=zeros(1,nt); work=zeros(1,nt);
for j=k
    s=r.ours.state(6,j); normal=r.ours.planeNormal(:,j); P=eye(3)-normal*normal';
    old=interp1(r.forward.s,r.measurements.previousShape{j}.p',s)';
    pObs=interp1(r.forward.s,r.measurements.p(:,:,j)',s)';
    pEst=interp1(r.forward.s,r.ours.p(:,:,j)',s)';
    observed(j)=t'*P*(pObs-old); predicted(j)=t'*P*(pEst-old);
    work(j)=r.ours.frictionForceResultant(:,j)'*P*(pObs-old);
end
plot(k,observed,'k-','LineWidth',1.5); plot(k,predicted,'--','Color',[0.7 0.2 0.65],'LineWidth',1.5);
yline(0,':'); xlabel('Simulation frame'); ylabel('Contact step along motion [mm]');
title('World-frame contact step');
legend('Observed','Estimated','Location','best');
nexttile(tl); hold on; grid on;
plot(k,1e3*work,'o-','Color',[0.7 0.2 0.65],'LineWidth',1.5);
yline(0,'k:'); xlabel('Simulation frame'); ylabel('F_t dot observed tangential step [mN mm]');
title('Sliding: work <= 0 (small stick drift tolerated)');
scenarioLabel='Plane / body contact';
if r.forward.contactArcLength(end)>r.forward.s(end)-1, scenarioLabel='Plane / tip contact'; end
caption=sprintf('%s | final mode: %s',scenarioLabel,r.ours.complementarityMode{end});
if r.forward.contactArcLength(end)>r.forward.s(end)-1
    caption={caption,'Contact at tip: contact / independent tip-load split is not identifiable'};
end
title(tl,caption,'Interpreter','none');
exportgraphics(fig,fullfile(r.config.outputDir,'friction.png'),'Resolution',160);
end
