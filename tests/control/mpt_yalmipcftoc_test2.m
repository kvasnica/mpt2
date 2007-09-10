function mpt_yalmipcftoc_test2

% tests whether we properly switch to the one-shot formulation when
% Options.details=1 is given:
%

lti1d
probStruct.norm = 1;
probStruct.N = 3;

% the DP solution will be used here:
ctrl = mpt_yalmipcftoc(sysStruct, probStruct);
mbg_assertequal(size(ctrl.Fi{1}, 1), 1);

% here we should switch to the one-shot formulation
ctrl = mpt_yalmipcftoc(sysStruct, probStruct, struct('details', 1));
mbg_assertequal(size(ctrl.Fi{1}, 1), 3);
