function estimate = estimate_planar_shape_only_point_loads(input)
% Fit ordered planar point loads from sparse curvature without environment geometry.
model=input.model;
assert(~isfield(model,'tipForceXZ'),'rod:TruthInShapeOnlyInput', ...
    'Tip force is unknown; do not pass it to the shape-only baseline.');
arc=input.fbgArcLengthMm(:)'; measured=input.curvaturePerMm(:)';
assert(numel(arc)==numel(measured) && all(diff(arc)>0));
contactCount=numel(model.contactPlaneIndex);
lengthMm=model.sMm(end);
intrinsic=interp1(model.sMm(1:end-1),model.intrinsicCurvaturePerMm,arc,'previous','extrap');
elastic=measured-intrinsic;
sampleArc=model.sMm(:)';
interpolated=interp1(arc,elastic,sampleArc,'pchip','extrap');
position=zeros(2,numel(sampleArc));
position(:,1)=model.baseXZ(:);
angle=model.baseAngleRad;
for index=1:numel(sampleArc)-1
    step=sampleArc(index+1)-sampleArc(index);
    turn=step*(model.intrinsicCurvaturePerMm(index)+ ...
        (interpolated(index)+interpolated(index+1))/2);
    scale=1;
    if abs(turn)>1e-8, scale=sin(turn/2)/(turn/2); end
    position(:,index+1)=position(:,index)+step*scale*[sin(angle+turn/2);cos(angle+turn/2)];
    angle=angle+turn;
end
sensorPosition=interp1(sampleArc,position',arc)';
tipPosition=position(:,end);
observedMoment=model.EINmm2*elastic(:);
partition=linspace(0,lengthMm,contactCount+1);
lower=partition(1:end-1)'+1e-3;
upper=partition(2:end)'-1e-3;
regularizationMm=0.5;
solverOptions=optimoptions('lsqnonlin','Display','off','MaxIterations',80, ...
    'MaxFunctionEvaluations',600,'FiniteDifferenceType','central');
timer=tic;
bestObjective=inf;
for fraction=[0.3 0.5 0.7]
    initial=lower+fraction*(upper-lower);
    [candidate,objective,~,flag]=lsqnonlin(@residual,initial,lower,upper,solverOptions);
    if objective<bestObjective
        bestArc=candidate; bestObjective=objective; bestFlag=flag;
    end
end
[matrix,forces]=fitForces(bestArc);
contactForceXZ=reshape(forces(1:2*contactCount),2,contactCount);
tipForceXZ=forces(2*contactCount+1:end);
estimate=struct('contactS',bestArc(:)','contactForceXZ',contactForceXZ, ...
    'tipForceXZ',tipForceXZ,'totalForceXZ',sum(contactForceXZ,2)+tipForceXZ, ...
    'seconds',toc(timer),'objective',bestObjective,'exitflag',bestFlag, ...
    'jacobianRank',rank(matrix),'unknownForceCount',size(matrix,2), ...
    'requiresReview',bestFlag<=0 || rank(matrix)<size(matrix,2) || ...
        rcond(matrix'*matrix)<1e-9, ...
    'scope','Ordered point-load fit to reconstructed planar shape; no environment point, normal, gap or tangency input.');
    function value=residual(contactArc)
        [design,fit]=fitForces(contactArc);
        value=[design*fit-observedMoment;regularizationMm*fit];
    end
    function [design,fit]=fitForces(contactArc)
        contactPosition=interp1(sampleArc,position',contactArc(:)')';
        design=zeros(numel(arc),2*contactCount+2);
        for contactIndex=1:contactCount
            arm=contactPosition(:,contactIndex)-sensorPosition;
            active=arc<contactArc(contactIndex);
            design(:,2*contactIndex-1)=(arm(2,:).*active)';
            design(:,2*contactIndex)=(-arm(1,:).*active)';
        end
        tipArm=tipPosition-sensorPosition;
        design(:,end-1)=tipArm(2,:)';
        design(:,end)=-tipArm(1,:)';
        systemMatrix=[design;regularizationMm*eye(size(design,2))];
        systemTarget=[observedMoment;zeros(size(design,2),1)];
        fit=systemMatrix\systemTarget;
    end
end
