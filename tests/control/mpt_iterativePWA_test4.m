function mpt_iterativePWA_test4

% tests Options.iterative=1 (reduced-switching strategy)

pwa1d
% set up a standard minimum-time problem
probStruct.subopt_lev = 1;
probStruct.N = Inf;

% use Options.iterative=1 (reduced-switching strategy) with 2-norm
probStruct.norm = 2;
ctrl = mpt_control(sysStruct, probStruct, struct('iterative', 1));
mbg_assertequal(length(ctrl.Pn), 6);

% use Options.iterative=1 (reduced-switching strategy) with 1-norm
probStruct.norm = 1;
ctrl = mpt_control(sysStruct, probStruct, struct('iterative', 1));
mbg_assertequal(length(ctrl.Pn), 6);
