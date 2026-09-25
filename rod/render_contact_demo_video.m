function report = render_contact_demo_video(resultsSource, videoPath, options)
%RENDER_CONTACT_DEMO_VIDEO Render a solved contact demo in the legacy format.
%
% This renderer deliberately follows the established MATLAB presentation used
% by run_rod_plane_force_sensing_experiment: a white 1180x620 figure, the
% rod/environment view on the left, and a force history on the right.  It
% consumes only saved states; it never invents a new force estimate.  When
% more movie frames than solved states are requested, positions are sampled
% between adjacent saved states only for display timing, while the title
% reports the source state and the source-state estimate remains unchanged.
%
% report = render_contact_demo_video(resultsMat, outputMp4, options)
% options.frameRate (default 10), options.durationSeconds (default 6), and
% options.title/options.paperTag are optional.  The MAT file must contain
% sensorInput, truth, output, scene, and report as written by
% run_contact_demo_suite.

if nargin < 3 || isempty(options), options = struct(); end
if ischar(resultsSource) || isstring(resultsSource)
    loaded = load(char(resultsSource));
else
    loaded = resultsSource;
end
required = {'truth','output','scene'};
assert(all(isfield(loaded, required)), ...
    'render_contact_demo_video:InvalidInput', ...
    'Expected saved truth, output, and scene fields.');
truth = loaded.truth;
out = loaded.output.ours;
scene = loaded.scene;
if ~isfield(loaded, 'report'), savedReport = struct(); else, savedReport = loaded.report; end

frameRate = getOption(options, 'frameRate', 10);
durationSeconds = getOption(options, 'durationSeconds', 6);
nFrames = max(size(out.p, 3), ceil(frameRate * durationSeconds));
samplePositions = linspace(1, size(out.p, 3), nFrames);
if isfield(options, 'title'), sceneTitle = char(options.title); else, sceneTitle = scene.title; end
paperTag = getOption(options, 'paperTag', 'EnFiRCE contact demo');

parent = fileparts(videoPath);
if ~isempty(parent) && ~isfolder(parent), mkdir(parent); end
writer = VideoWriter(videoPath, 'MPEG-4');
writer.FrameRate = frameRate;
open(writer);
fig = figure('Visible','off','Color','w','Position',[80 80 1180 620]);
cleanup = onCleanup(@()closeVideoResources(writer, fig)); %#ok<NASGU>

trueP = truth.p;
estimatedP = out.p;
[xLimits, zLimits] = axesLimits(trueP, estimatedP);
n = truth.planeNormal(:) / max(norm(truth.planeNormal), eps);
trueForce = truth.contactForce;
estimatedForce = out.contactForceResultant;
trueTip = truth.tipForce;
estimatedTip = out.tipForce;
trueNormalForce = n * (n' * trueForce);
estimatedNormalForce = n * (n' * estimatedForce);
trueFrictionForce = trueForce - trueNormalForce;
estimatedFrictionForce = estimatedForce - estimatedNormalForce;
trueOrigins = truth.contactPoint;
estimatedOrigins = out.contactPoint;
maxForce = max([vecnorm([trueForce, estimatedForce, trueTip, estimatedTip],2,1), 1]);
shapeSpan = max(diff(xLimits), diff(zLimits));
forceScale = min(35, 0.18 * shapeSpan / maxForce);
motion = getMotion(loaded, size(out.p,3));
trueNormal = vecnorm(trueNormalForce,2,1);
estimatedNormal = vecnorm(estimatedNormalForce,2,1);
trueTangent = vecnorm(trueFrictionForce,2,1);
estimatedTangent = vecnorm(estimatedFrictionForce,2,1);
trueTotal = vecnorm(truth.totalForce,2,1);
estimatedTotal = vecnorm(out.totalForceResultant,2,1);

for frameIdx = 1:nFrames
    q = samplePositions(frameIdx);
    pTrue = interpArray(trueP, q);
    pEstimated = interpArray(estimatedP, q);
    fcTrue = interpArray(trueForce, q);
    fcEstimated = interpArray(estimatedForce, q);
    tipTrue = interpArray(trueTip, q);
    tipEstimated = interpArray(estimatedTip, q);
    normalTrue = interpArray(trueNormalForce, q);
    normalEstimated = interpArray(estimatedNormalForce, q);
    frictionTrue = interpArray(trueFrictionForce, q);
    frictionEstimated = interpArray(estimatedFrictionForce, q);
    originTrue = interpArray(trueOrigins, q);
    originEstimated = interpArray(estimatedOrigins, q);
    if any(~isfinite(originEstimated)), originEstimated = pEstimated(:,end); end
    sourceState = max(1, min(size(out.p,3), round(q)));
    rmse = interpArray(out.shapeRmseMm, q);
    phase = stateText(out, sourceState);
    mode = modeText(out, sourceState);
    [motionHistory, estNormalHistory] = historyAt(motion, estimatedNormal, q);
    [~, estTangentHistory] = historyAt(motion, estimatedTangent, q);
    [~, estTotalHistory] = historyAt(motion, estimatedTotal, q);

    clf(fig);
    layout = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    title(layout, sprintf('%s  |  %s', sceneTitle, paperTag), ...
        'FontWeight','bold','FontSize',13);
    axShape = nexttile(layout,1);
    hold(axShape,'on'); grid(axShape,'on'); box(axShape,'on'); axis(axShape,'equal');
    hTrue = plot(axShape,pTrue(1,:),pTrue(3,:),'-','Color',[0.55 0.78 0.95],'LineWidth',2.6);
    hEst = plot(axShape,pEstimated(1,:),pEstimated(3,:),'-','Color',[0.05 0.28 0.72],'LineWidth',2.0);
    hPlane = plotPlaneSection(axShape, truth.planePoint, n, xLimits, zLimits);
    hTN = quiver(axShape,originTrue(1),originTrue(3),normalTrue(1)*forceScale,normalTrue(3)*forceScale,0,...
        'Color',[0.10 0.45 0.80],'LineWidth',2.0,'MaxHeadSize',0.8);
    hTF = quiver(axShape,originTrue(1),originTrue(3),frictionTrue(1)*forceScale,frictionTrue(3)*forceScale,0,...
        'Color',[0.10 0.65 0.25],'LineWidth',2.0,'MaxHeadSize',0.8);
    hEN = quiver(axShape,originEstimated(1),originEstimated(3),normalEstimated(1)*forceScale,normalEstimated(3)*forceScale,0,...
        'Color',[0.85 0.15 0.10],'LineWidth',1.8,'MaxHeadSize',0.8);
    hEF = quiver(axShape,originEstimated(1),originEstimated(3),frictionEstimated(1)*forceScale,frictionEstimated(3)*forceScale,0,...
        'Color',[0.70 0.20 0.65],'LineWidth',1.8,'MaxHeadSize',0.8);
    hTT = quiver(axShape,pTrue(1,end),pTrue(3,end),tipTrue(1)*forceScale,tipTrue(3)*forceScale,0,...
        'Color',[0.15 0.60 0.82],'LineWidth',1.6,'MaxHeadSize',0.8);
    hET = quiver(axShape,pEstimated(1,end),pEstimated(3,end),tipEstimated(1)*forceScale,tipEstimated(3)*forceScale,0,...
        'Color',[0.90 0.35 0.08],'LineWidth',1.6,'MaxHeadSize',0.8);
    xlim(axShape,xLimits); ylim(axShape,zLimits);
    xlabel(axShape,'x [mm]'); ylabel(axShape,'z [mm]');
    title(axShape,{sprintf('Frame %d/%d | source state %d/%d | %s | %s',frameIdx,nFrames,sourceState,size(out.p,3),phase,mode),...
        sprintf('Shape RMSE %.3f mm | true contact %.3f N | estimate %.3f N',rmse,norm(fcTrue),norm(fcEstimated))});
    legend(axShape,[hTrue hEst hPlane hTN hTF hEN hEF hTT hET],...
        {'True shape','Estimated shape','Plane','True normal','True friction','Estimated normal','Estimated friction','True tip','Estimated tip'},...
        'Location','southoutside','NumColumns',3);

    axForce = nexttile(layout,2);
    hold(axForce,'on'); grid(axForce,'on'); box(axForce,'on');
    hN = plot(axForce,motion,trueNormal,'k-','LineWidth',1.4);
    hNE = plot(axForce,motionHistory,estNormalHistory,'r--','LineWidth',1.8);
    hT = plot(axForce,motion,trueTangent,'-','Color',[0.10 0.45 0.80],'LineWidth',1.4);
    hTE = plot(axForce,motionHistory,estTangentHistory,'--','Color',[0.55 0.20 0.70],'LineWidth',1.8);
    hTot = plot(axForce,motion,trueTotal,'-','Color',[0.15 0.55 0.25],'LineWidth',1.4);
    hTotE = plot(axForce,motionHistory,estTotalHistory,':','Color',[0.85 0.35 0.05],'LineWidth',1.8);
    xline(axForce,interpArray(motion,q),':','Color',[0.35 0.35 0.35]);
    xlabel(axForce,'Base motion [mm]'); ylabel(axForce,'Force magnitude [N]');
    title(axForce,sprintf('true F_c=[%.2f %.2f %.2f], estimate=[%.2f %.2f %.2f] N',fcTrue,fcEstimated));
    legend(axForce,[hN hNE hT hTE hTot hTotE],...
        {'True normal','Estimated normal','True friction','Estimated friction','True total','Estimated total'},...
        'Location','best');

    drawnow;
    writeVideo(writer, captureFrame(fig));
    if frameIdx == nFrames
        exportgraphics(fig,fullfile(parent,'forces.png'),'Resolution',160);
    end
end

% onCleanup closes the writer and figure even if encoding fails.
report = struct('videoPath',videoPath,'frameRate',frameRate,'frameCount',nFrames,...
    'sourceStateCount',size(out.p,3),'sourceReport',savedReport);
end

function value = getOption(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), value=s.(name); else, value=default; end
end

function motion = getMotion(loaded,nt)
if isfield(loaded,'sensorInput') && isfield(loaded.sensorInput,'packet') && isfield(loaded.sensorInput.packet,'actuationMm')
    motion=loaded.sensorInput.packet.actuationMm(:)';
elseif isfield(loaded,'truth') && isfield(loaded.truth,'basePose')
    motion=squeeze(loaded.truth.basePose(3,4,:))';
else
    motion=1:nt;
end
if numel(motion)~=nt, motion=linspace(motion(1),motion(end),nt); end
end

function value = interpArray(data,q)
nt=size(data,ndims(data));
i0=max(1,min(nt,floor(q))); i1=max(1,min(nt,ceil(q))); a=q-i0;
subs0=repmat({':'},1,ndims(data)); subs1=subs0; subs0{end}=i0; subs1{end}=i1;
value=(1-a)*data(subs0{:})+a*data(subs1{:});
end

function [mh,vh]=historyAt(motion,values,q)
i=max(1,min(numel(motion),floor(q))); mh=motion(1:i); vh=values(1:i);
if q>i+1e-10 && i<numel(motion)
    mh(end+1)=interpArray(motion,q); vh(end+1)=interpArray(values,q);
end
end

function [xLim,zLim]=axesLimits(a,b)
x=[reshape(a(1,:,:),1,[]),reshape(b(1,:,:),1,[])]; z=[reshape(a(3,:,:),1,[]),reshape(b(3,:,:),1,[])];
xMargin=max(10,0.08*max(max(x)-min(x),1)); zMargin=max(10,0.08*max(max(z)-min(z),1));
xLim=[min(x)-xMargin,max(x)+xMargin]; zLim=[min(z)-zMargin,max(z)+zMargin];
end

function h=plotPlaneSection(ax,point,n,xLim,zLim)
nx=n(1); nz=n(3); c=n(1)*point(1)+n(3)*point(3);
if abs(nz)>1e-9
    x=linspace(xLim(1),xLim(2),2); z=(c-nx*x)/nz;
elseif abs(nx)>1e-9
    z=linspace(zLim(1),zLim(2),2); x=repmat(c/nx,1,2);
else
    x=[xLim(1),xLim(2)]; z=[zLim(2),zLim(2)];
end
h=plot(ax,x,z,'k--','LineWidth',1.1);
end

function frame=captureFrame(fig)
try
    frame=getframe(fig);
catch
    tmp=[tempname '.png']; exportgraphics(fig,tmp,'Resolution',120); frame=imread(tmp); delete(tmp);
end
end

function text=stateText(out,k)
text='contact';
if isfield(out,'complementarityMode') && numel(out.complementarityMode)>=k
    text=char(out.complementarityMode{k});
end
end

function text=modeText(out,k)
text='';
if isfield(out,'modeResolution') && numel(out.modeResolution)>=k
    value=out.modeResolution{k};
    if isstruct(value) && isfield(value,'name')
        text=char(value.name);
    elseif ischar(value) || isstring(value)
        text=char(value);
    end
end
if isempty(text), text='MPCC'; end
end

function closeVideoResources(writer,fig)
try, close(writer); catch, end
try, close(fig); catch, end
end
