function q=friction_direction_quality(o,kinematicsConsistent)
% Feasible friction equations do not establish an observable slip direction.
% Full MPCC currently provides a selected mode, not mode confidence. A noisy
% contact without an observation-resolution test must remain unassessed.
nt=numel(o.frictionMu);
q=struct('frictionDirectionResolutionAssessed',false(1,nt), ...
    'frictionDirectionResolved',false(1,nt),'hasFrictionObservationWarning',false(1,nt));
for k=1:nt
    applicable=o.frictionMu(k)>0 && o.state(7,k)>1e-4;
    if ~applicable,continue;end
    if o.historyCurvatureStdPerMm==0
        % Conditional on the explicitly declared deterministic-history model.
        q.frictionDirectionResolutionAssessed(k)=true;
        q.frictionDirectionResolved(k)=kinematicsConsistent(k)&& ...
            strcmp(o.complementarityMode{k},'sliding-contact');
    else
        mode=o.modeResolution{k};
        if all(isfield(mode,{'directionResolved','observedSlipMm','slipResolutionMm'}))
            q.frictionDirectionResolutionAssessed(k)=true;
            q.frictionDirectionResolved(k)=kinematicsConsistent(k)&&mode.directionResolved&& ...
                mode.observedSlipMm>mode.slipResolutionMm;
        end
        q.hasFrictionObservationWarning(k)=~q.frictionDirectionResolved(k);
    end
end
end
