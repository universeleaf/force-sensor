function test_friction_quality()
% An exact MPCC solution for noisy history is not direction certification.
o=struct('frictionMu',[.3 .3 0],'state',zeros(12,3), ...
    'historyCurvatureStdPerMm',5e-5, ...
    'complementarityMode',{{'sliding-contact','sticking-contact','frictionless-contact'}}, ...
    'modeResolution',{{struct('name','sliding-contact'),struct('name','sticking-contact'),struct}});
o.state(7,:)=1;
q=friction_direction_quality(o,[true true true]);
assert(~any(q.frictionDirectionResolved)&&~any(q.frictionDirectionResolutionAssessed)&& ...
    isequal(q.hasFrictionObservationWarning,[true true false]), ...
    'Numerical complementarity was mistaken for observable friction direction.');
o.modeResolution{1}=struct('directionResolved',true,'observedSlipMm',1,'slipResolutionMm',.01);
q=friction_direction_quality(o,[true true true]);
assert(q.frictionDirectionResolved(1)&&~q.hasFrictionObservationWarning(1));
q=friction_direction_quality(o,[false true true]);
assert(~q.frictionDirectionResolved(1)&&q.hasFrictionObservationWarning(1));
o.historyCurvatureStdPerMm=0;q=friction_direction_quality(o,[true true true]);
assert(q.frictionDirectionResolved(1)&&~q.frictionDirectionResolved(2)&&~any(q.hasFrictionObservationWarning));
o.historyCurvatureStdPerMm=5e-5;o.state(7,:)=0;
q=friction_direction_quality(o,[true true true]);assert(~any(q.hasFrictionObservationWarning));
disp('Friction feasibility, observation resolution and separation flags remain distinct.');
end
