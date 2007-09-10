function mpt_iterativePWA_test2

% test minimum-time control for 2-norm and 1-norm

pwa1d
% set up a standard minimum-time problem
probStruct.subopt_lev = 1;
probStruct.N = Inf;

% minimum-time controller for 2-norm
probStruct.norm = 2;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl.Pn), 6);

% minimum-time controller for 1-norm
probStruct.norm = 1;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl.Pn), 6);
