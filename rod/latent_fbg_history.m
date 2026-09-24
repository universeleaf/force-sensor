function model = latent_fbg_history(tube,previous,cfg)
%LATENT_FBG_HISTORY Full sparse predecessor observation coordinates.
% q=(u_previous_latent-u_previous_measured)/sigma; its Gaussian negative
% log likelihood is q'*q/2. Every observed sensor/channel is retained.
% Unobserved curvature (e.g. torsion) remains a reconstruction assumption.
sigma=cfg.forceSensor.historyCurvatureStdPerMm;
axes=cfg.forceSensor.curvatureObservedAxes;
assert(sigma>0&&all(isfield(previous,{'sparseU','fbgIdx','T_base'})), ...
    'rod:MissingHistoryObservation','Latent history needs positive noise calibration and raw sparse predecessor observations.');
count=numel(axes)*numel(previous.fbgIdx);
model=struct('count',count,'shape',@shape,'sigmaPerMm',sigma,'observedAxes',axes);
% Exact local cache: many current-state perturbations share one predecessor.
keys=zeros(count,1);values={previous};
    function result=shape(q)
        q=q(:);
        assert(numel(q)==count&&isreal(q)&&all(isfinite(q)), ...
            'rod:InvalidHistoryState','Invalid standardized latent FBG vector.');
        match=find(all(keys==q,1),1);
        if ~isempty(match),result=values{match};return;end
        sparse=previous.sparseU;
        sparse(axes,:)=sparse(axes,:)+sigma*reshape(q,numel(axes),[]);
        result=reconstruct_sensor_curvature(tube,sparse,previous.fbgIdx,previous.T_base,cfg);
        if isfield(previous,'timeSeconds'),result.timeSeconds=previous.timeSeconds;end
        keys(:,end+1)=q;values{end+1}=result;
        if numel(values)>128,keys(:,1)=[];values(1)=[];end
    end
end
