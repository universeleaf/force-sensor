function videoReport=render_multi_contact_demo_video(loaded,videoPath,options)
% Render saved forward/inverse states in the established two-panel format.
% Each playback frame holds an actual solved state; no synthetic forces or
% interpolated states are reported as additional solver evaluations.
if nargin<3, options=struct; end
if ischar(loaded)||isstring(loaded), loaded=load(char(loaded)); end
truth=loaded.truth; estimate=loaded.estimate; scene=loaded.scene;
nt=size(truth.p,3); K=size(truth.contactForces,2);
frameRate=10; repeats=5;
if isfield(options,'frameRate'),frameRate=options.frameRate;end
if isfield(options,'repeatsPerState'),repeats=options.repeatsPerState;end
parent=fileparts(videoPath);
if ~isfolder(parent),mkdir(parent);end
writer=VideoWriter(videoPath,'MPEG-4'); writer.FrameRate=frameRate; open(writer);
fig=figure('Visible','off','Color','w','Position',[80 80 1180 620]);
cleanup=onCleanup(@()closeResources(writer,fig)); %#ok<NASGU>
allP=cat(2,truth.p,estimate.p);
xLim=limits(allP(1,:,:)); zLim=limits(allP(3,:,:));
maxF=max(vecnorm([reshape(truth.contactForces,3,[]),reshape(estimate.contactForces,3,[]),truth.tipForce,estimate.tipForce]));
forceScale=min(30,0.6*diff(xLim)/max(maxF,eps));
colors=lines(K); motion=scene.baseMotionMm;
forceTrue=squeeze(vecnorm(truth.contactForces,2,1));
forceEstimate=squeeze(vecnorm(estimate.contactForces,2,1));
for k=1:nt
    clf(fig); layout=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    title(layout,sprintf('%s | planar, frictionless | state %d/%d',scene.title,k,nt),'FontSize',13);
    ax=nexttile(layout,1); hold(ax,'on'); grid(ax,'on'); box(ax,'on'); axis(ax,'equal');
    p=truth.p(:,:,k); pe=estimate.p(:,:,k);
    h1=plot(ax,p(1,:),p(3,:),'-','Color',[0.35 0.7 0.9],'LineWidth',3);
    h2=plot(ax,pe(1,:),pe(3,:),'--','Color',[0.1 0.2 0.65],'LineWidth',1.6);
    for j=1:size(scene.planePointXZ,2)
        n=scene.planeNormalXZ(:,j); point=scene.planePointXZ(:,j); c=n'*point;
        if abs(n(1))>1e-8, z=zLim; x=(c-n(2)*z)/n(1); else, x=xLim;z=(c-n(1)*x)/n(2);end
        plot(ax,x,z,'k-','LineWidth',1.5,'HandleVisibility','off');
    end
    for j=1:K
        cp=truth.contactPoints(:,j,k); ce=estimate.contactPoints(:,j,k);
        f=truth.contactForces(:,j,k); fe=estimate.contactForces(:,j,k);
        plot(ax,cp(1),cp(3),'o','Color',colors(j,:),'MarkerFaceColor',colors(j,:),'HandleVisibility','off');
        quiver(ax,cp(1),cp(3),f(1)*forceScale,f(3)*forceScale,0,'Color',colors(j,:),'LineWidth',2,'HandleVisibility','off');
        quiver(ax,ce(1),ce(3),fe(1)*forceScale,fe(3)*forceScale,0,'Color',colors(j,:),'LineStyle','--','LineWidth',1.3,'HandleVisibility','off');
        text(ax,cp(1)+1,cp(3)+4,sprintf('C%d: %.2f / %.2f N',j,norm(f),norm(fe)),'Color',colors(j,:),'FontSize',9);
    end
    plot(ax,p(1,1),p(3,1),'ks','MarkerFaceColor','k','HandleVisibility','off');
    xlim(ax,xLim); ylim(ax,zLim); xlabel(ax,'x [mm]'); ylabel(ax,'z [mm]');
    delta=pe-p; shapeError=sqrt(mean(sum(delta.^2,1)));
    title(ax,{sprintf('24 curvature observations | true / estimated forces'), ...
        sprintf('Shape RMSE %.3g mm | review %d',shapeError,estimate.requiresReview(k))});
    legend(ax,[h1 h2],{'True rod','Estimated rod'},'Location','southoutside','Orientation','horizontal');
    ax2=nexttile(layout,2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
    handles=gobjects(0); labels={};
    for j=1:K
        handles(end+1)=plot(ax2,motion,forceTrue(j,:),'-','Color',colors(j,:),'LineWidth',1.8); %#ok<AGROW>
        handles(end+1)=plot(ax2,motion(1:k),forceEstimate(j,1:k),'--o','Color',colors(j,:),'LineWidth',1.2,'MarkerSize',3); %#ok<AGROW>
        labels=[labels {sprintf('C%d true',j),sprintf('C%d estimate',j)}]; %#ok<AGROW>
    end
    handles(end+1)=plot(ax2,motion,vecnorm(truth.totalForce),'k-','LineWidth',1.3);
    handles(end+1)=plot(ax2,motion(1:k),vecnorm(estimate.totalForce(:,1:k)),'k--','LineWidth',1.3);
    labels=[labels {'Resultant true','Resultant estimate'}];
    xline(ax2,motion(k),':','HandleVisibility','off');
    xlim(ax2,[motion(1) motion(end)]);
    xlabel(ax2,'Base lateral displacement [mm]'); ylabel(ax2,'Force [N]');
    title(ax2,{sprintf('Unknown tip force | true [%.3f, %.3f] N',truth.tipForce([1 3],k)), ...
        sprintf('Estimated tip [%.3f, %.3f] N',estimate.tipForce([1 3],k))});
    legend(ax2,handles,labels,'Location','southoutside','NumColumns',2);
    drawnow; frame=getframe(fig);
    for j=1:repeats,writeVideo(writer,frame);end
    if k==nt,exportgraphics(fig,fullfile(parent,'forces.png'),'Resolution',160);end
end
videoReport=struct('frameRate',frameRate,'frameCount',nt*repeats,'sourceStateCount',nt, ...
    'scope','Saved truth and inverse states; each state held for playback, no interpolation.');
end

function lim=limits(x)
x=x(:); pad=max(8,0.08*(max(x)-min(x))); lim=[min(x)-pad max(x)+pad];
end
function closeResources(writer,fig)
try,close(writer);catch,end
try,close(fig);catch,end
end
