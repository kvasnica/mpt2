function mpt_optMixedCtrl_test1

% test control of systems with mixed inputs
pwa1d

% system with 2 inputs
sysStruct.B = {[1 1], [1 1]};
sysStruct.D = {[0 0], [0 0]};
sysStruct.umax = [1; 1];
sysStruct.umin = [-1; -1];
probStruct.R = eye(2); 

probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2; probStruct.P_N = 1;

% first input is real
sysStruct.Uset{1} = [-Inf Inf];
% second input is from finite alphabet
sysStruct.Uset{2} = [-0.5 0 0.5];

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 6);
