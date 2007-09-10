function mpt_mixedMinTime_test1

% test minimum-timecontrol of systems with mixed inputs
pwa1d

% system with 2 inputs
sysStruct.B = {[1 1], [1 1]};
sysStruct.D = {[0 0], [0 0]};
sysStruct.umax = [1; 1];
sysStruct.umin = [-1; -1];
probStruct.R = eye(2);

% minimum-time controllers for systems with boolean inputs requires target set
% and P_N penalty to be defined
probStruct.Tset = unitbox(1,1);
probStruct.P_N = 1;

% first input is real
sysStruct.Uset{1} = [-Inf Inf];
% second input is from finite alphabet
sysStruct.Uset{2} = [-0.5 0 0.5];

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 2);
