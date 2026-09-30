function result = estimate_literature_curvature_baseline(input, method, contactCount, options)
%ESTIMATE_LITERATURE_CURVATURE_BASELINE Source-grounded, same-curvature baselines.
% 'point': unknown point forces/arcs, Xiao & Chen (2021) curvature LS idea,
% generalized to calibrated precurvature, all local force components and twist.
% 'gaussian': Aloi et al. (2022), (9)-(10), local transverse Gaussian series;
% the position likelihood (1) is adapted to the available two-channel strain.
% Both retain an independent unknown 3-D tip force, and exact Cosserat balance.
% These are disclosed adaptations, not the authors' official implementations.
if nargin<4,options=struct;end
if ~isfield(options,'numStarts'),options.numStarts=3;end
if ~isfield(options,'maxIterations'),options.maxIterations=80;end
if ~isfield(options,'showProgress'),options.showProgress=true;end
assert(ismember(method,{'point','gaussian'})&&isscalar(contactCount)&&contactCount>=0&& ...
    contactCount==round(contactCount),'rod:InvalidLiteratureBaseline','Invalid method/contact count.');
obs=formulation_window_observations(input); tube=input.tube; K=contactCount; T=obs.frameCount;
localAxes=1:3;if strcmp(method,'gaussian'),localAxes=1:2;end
d=numel(localAxes); L=tube.s(end)-tube.s(1); partitions=linspace(tube.s(1),tube.s(end),K+1);
lower=partitions(1:end-1)'+1e-3; upper=partitions(2:end)'-1e-3;
lb=[lower;-1000*ones(d*K,1);-10*ones(3,1)];
ub=[upper;1000*ones(d*K,1);10*ones(3,1)];
scale=[10*ones(K,1);100*ones(d*K,1);ones(3,1)];
if strcmp(method,'gaussian')
    lb=[lb;log(0.25)*ones(K,1)];ub=[ub;log(L/3)*ones(K,1)];scale=[scale;ones(K,1)];
end
solverOptions=optimoptions('lsqnonlin','Display','off','MaxIterations',options.maxIterations, ...
    'MaxFunctionEvaluations',12000,'FiniteDifferenceType','central', ...
    'FiniteDifferenceStepSize',1e-5,'FunctionTolerance',1e-9,'OptimalityTolerance',1e-7,'StepTolerance',1e-9);
force=zeros(3,K,T);tip=zeros(3,T);arcs=zeros(K,T);frames=cell(1,T);
flags=zeros(1,T);cost=zeros(1,T);seconds=zeros(1,T);counts=zeros(1,T);
parameters=zeros(numel(lb),T);trace=cell(1,T);previous=[];rejected=0;
timer=tic;
for k=1:T
    frameTimer=tic; tube.T_base=obs.basePose(:,:,k);
    elastic=zeros(3,numel(obs.arcs));
    for a=1:numel(obs.axes)
        ax=obs.axes(a);elastic(ax,:)=obs.u(a,:,k)-interp1(tube.s,tube.uhat(ax,:),obs.arcs,'previous');
    end
    dense=tube.uhat;
    for ax=obs.axes,dense(ax,:)=dense(ax,:)+interp1(obs.arcs,elastic(ax,:),tube.s,'pchip','extrap');end
    [seedR,seedP]=integrate_curvature_field(tube,dense,tube.T_base);
    starts=cell(1,options.numStarts);startCost=zeros(1,numel(starts));
    for z=1:options.numStarts
        fraction=0.5;if options.numStarts>1,fraction=0.25+0.5*(z-1)/(options.numStarts-1);end
        s0=lower+fraction*(upper-lower);A=zeros(numel(obs.axes)*numel(obs.arcs),d*K+3);
        for ii=1:numel(obs.arcs)
            node=find(tube.s<=obs.arcs(ii),1,'last');stiff=reshape(getTubeK(tube),3,[]);
            Rs=interp1(tube.s,reshape(seedR,9,[])',obs.arcs(ii))';Rs=reshape(Rs,3,3);
            p=interp1(tube.s,seedP',obs.arcs(ii))';rows=(ii-1)*numel(obs.axes)+(1:numel(obs.axes));
            project=Rs(:,obs.axes)'./stiff(obs.axes,node);
            for jj=1:K
                pc=interp1(tube.s,seedP',s0(jj))';Rc=reshape(interp1(tube.s,reshape(seedR,9,[])',s0(jj)),3,3);
                if obs.arcs(ii)<s0(jj),A(rows,(jj-1)*d+(1:d))=project*hat(pc-p)*Rc(:,localAxes);end
            end
            A(rows,end-2:end)=project*hat(seedP(:,end)-p)*seedR(:,:,end);
        end
        W=obs.curvatureWhitening(:,:,k);b=elastic(obs.axes,:);
        seed=[W*A;eye(size(A,2))*1e-3]\[W*b(:);zeros(size(A,2),1)];
        x=[s0;seed];if strcmp(method,'gaussian'),x=[x;log(0.5)*ones(K,1)];end
        x=max(lb,min(ub,x));starts{z}=x;startCost(z)=norm(safeResidual(x./scale))^2;
    end
    if ~isempty(previous),starts{end+1}=previous;startCost(end+1)=norm(safeResidual(previous./scale))^2;end
    [~,order]=sort(startCost);bestCost=inf;best=[];candidateTrace=cell(size(starts));
    for z=1:numel(order)
        x0=starts{order(z)};
        [y,value,~,flag,out]=lsqnonlin(@safeResidual,x0./scale,lb./scale,ub./scale,solverOptions);
        candidateTrace{z}=struct('startIndex',order(z),'exitflag',flag,'iterations',out.iterations, ...
            'objective',value,'functionCount',out.funcCount);
        if value<bestCost,bestCost=value;best=y.*scale;bestFlag=flag;end
        % A model-consistent, converged zero-residual branch needs no retries.
        if flag>0&&value<1e-6,break;end
    end
    assert(~isempty(best),'rod:LiteratureBaselineFailure','No finite optimization trial.');
    final=predict(best,true);arcs(:,k)=best(1:K);force(:,:,k)=final.contactForce;tip(:,k)=final.tipForce;
    parameters(:,k)=best;previous=best;flags(k)=bestFlag;cost(k)=bestCost;
    frames{k}=struct('shape',final,'predictedCurvature',final.predictedCurvature(obs.axes,:));
    seconds(k)=toc(frameTimer);trace{k}=candidateTrace(1:z);counts(k)=z;
    if options.showProgress
        fprintf('Literature %s K=%d frame %d/%d: exit=%d cost=%.5g %.1fs\n',method,K,k,T,bestFlag,bestCost,seconds(k));
    end
end
idx=obs.outputIndices;
result=struct('method',['literature-adapted-' method '-curvature-ls'],'contactForce',force(:,:,idx), ...
    'contactForceResultant',reshape(sum(force(:,:,idx),2),3,[]),'tipForce',tip(:,idx), ...
    'totalForceResultant',reshape(sum(force(:,:,idx),2),3,[])+tip(:,idx), ...
    'contactArcLength',arcs(:,idx),'contactPlaneIndex',nan(1,K), ...
    'activeContacts',reshape(vecnorm(force(:,:,idx))>1e-5,K,numel(idx)), ...
    'frames',{frames(idx)},'timeSeconds',obs.timeSeconds(idx),'parameters',parameters, ...
    'optimizationSeconds',toc(timer),'perFrameSeconds',seconds(idx),'objective',cost(idx), ...
    'quality',struct('requiresReview',flags(idx)<=0,'hasHistory',obs.predecessorIndex(idx)>0), ...
    'solver',struct('exitflag',flags(idx),'trace',{trace},'starts',counts,'rejectedTrials',rejected), ...
    'observations',obs,'options',options,'authorsOfficialImplementation',false, ...
    'scope','Same measured curvature/covariance/rod/base poses; independent frames; shared data-derived count only; own shape-only initial arcs; no environment/friction likelihood or truth.');
result.observationFit=audit_formulation_window_fit(result);
result.quality.requiresReview=result.quality.requiresReview|result.observationFit.requiresReview;
    function value=safeResidual(y)
        try
            x=y.*scale;sh=predict(x,false);e=sh.predictedCurvature(obs.axes,:)-obs.u(:,:,k);
            % Weak, isotropic zero-force regularization. No force/arc truth.
            amplitude=x(K+1:K+d*K+3);value=[obs.curvatureWhitening(:,:,k)*e(:);amplitude/1e6];
            if any(~isfinite(value)),error('rod:NonfiniteBaseline','Invalid prediction.');end
        catch
            rejected=rejected+1;value=ones(numel(obs.axes)*numel(obs.arcs)+d*K+3,1)*1e8;
        end
    end
    function sh=predict(x,computeShape)
        local=zeros(3,K);local(localAxes,:)=reshape(x(K+(1:d*K)),d,K);
        ft=x(K+d*K+(1:3));opt=struct('queryArcsMm',obs.arcs,'computeShape',computeShape);
        if strcmp(method,'gaussian'),opt.gaussianSigmaMm=exp(x(K+d*K+3+(1:K)));end
        sh=integrate_body_load_curvature(tube,x(1:K),local,ft,opt);
    end
end
function H=hat(v),H=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];end
