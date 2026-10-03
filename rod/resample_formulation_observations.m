function input=resample_formulation_observations(clean,seed,sigma,sampleEnvironment)
% Simulation generator only: repeated independent noise on fixed equilibria.
assert(isstruct(clean)&&isfield(clean,'packet')&&clean.packet.schemaVersion==2&& ...
    isscalar(seed)&&isfinite(seed)&&seed==fix(seed)&&seed>=0&&isscalar(sigma)&&isfinite(sigma)&&sigma>=0, ...
    'rod:InvalidResampling','Use a clean schema-2 packet, integer seed and nonnegative noise SD.');
assert(isscalar(sampleEnvironment)&&ismember(sampleEnvironment,[false true]),'rod:InvalidResampling','Invalid environment switch.');
input=clean;stream=RandStream('mt19937ar','Seed',seed);p=clean.packet;
input.packet.curvaturePerMm=p.curvaturePerMm+sigma*randn(stream,size(p.curvaturePerMm));
input.packet.curvatureStdPerMm=max(sigma,1e-7);
if isfield(input.packet,'curvatureCovariance'),input.packet=rmfield(input.packet,'curvatureCovariance');end
if sampleEnvironment
    for k=1:numel(p.timeSeconds)
        for j=1:size(p.planePointMm,2)
            perturbation=chol(p.planeCovariance(:,:,j,k),'lower')*randn(stream,6,1);
            input.packet.planePointMm(:,j,k)=p.planePointMm(:,j,k)+perturbation(1:3);
            normal=p.planeNormal(:,j,k)+perturbation(4:6);
            assert(norm(normal)>1e-12,'rod:InvalidResampling','Degenerate sampled normal.');
            input.packet.planeNormal(:,j,k)=normal/norm(normal);
        end
    end
end
end
