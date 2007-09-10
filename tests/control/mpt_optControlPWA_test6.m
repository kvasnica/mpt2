function mpt_optControlPWA_test6

% tests that we properly switch to different method when no QP solver is
% available:

% set up CFTOC problem
pwa1d
probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2;

ctrl = mpt_control(sysStruct, probStruct, struct('qpsolver', -1));
mbg_assertequal(length(ctrl), 6);
