function mpt_optControlPWA_test3

% test CFTOC for systems with multiple inputs

pwa1d
% use two inputs
sysStruct.B = {[1 1], [1 1]};
sysStruct.D = {[0 0], [0 0]};
sysStruct.umax = [1; 1];
sysStruct.umin = [-1; -1];

% set up CFTOC problem
probStruct.R = eye(2);
probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2;

ctrl=mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl.Pn), 4);
