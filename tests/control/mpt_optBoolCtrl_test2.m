function mpt_optBoolCtrl_test2

% test control of systems with multiple inputs
pwa1d
sysStruct.B = {[1 1], [1 1]};
sysStruct.D = {[0 0], [0 0]};
sysStruct.umax = [1; 1];
sysStruct.umin = [-1; -1];

% CFTOC problem
probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2;
probStruct.R = eye(2);

% two inputs, first from finite alphabet, second boolean
sysStruct.Uset{1} = [-0.5 0 0.5];
sysStruct.Uset{2} = [0 1];

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 4);
