function mpt_optInfControl_test1

% tests CITOC for linear systems
lti1d
probStruct.N = Inf;

% CITOC for 2-norms
probStruct.norm = 2;
ctrl=mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 7);

% CITOC for 2 norms must set details.regionHorizon field:
mbg_assertequal(ctrl.details.regionHorizon, [0 1 1 2 2 3 3]);


% CITOC for linear norms - uses mpt_optInfControlPWA()
probStruct.norm = Inf;
ctrl=mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 8);

% CITOC for linear norms must set details.incore:
mbg_assertequal(ctrl.details.incore, [1 1 1 1 1 1 1 1]);
