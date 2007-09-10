function mpt_boolMinTime_test2

% test control of systems with multiple inputs
pwa1d
sysStruct.B = {[1 1], [1 1]};
sysStruct.D = {[0 0], [0 0]};
sysStruct.umax = [1; 1];
sysStruct.umin = [-1; -1];
probStruct.R = eye(2);

% minimum-time controllers for systems with boolean inputs requires target set
% to be given
probStruct.Tset = unitbox(1,1.5);

% two inputs, first from finite alphabet, second boolean
sysStruct.Uset{1} = [-1 0 1];
sysStruct.Uset{2} = [0 1];

ctrl = mpt_control(sysStruct, probStruct);

mbg_assertequal(length(ctrl), 6);
