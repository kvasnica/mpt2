function mpt_boolMinTime_test1

% test minimum-time control of systems with boolean/discrete inputs w/ noise
pwa1d_addU

% minimum-time controllers for systems with boolean inputs requires target set
% to be given
probStruct.Tset = unitbox(1,1);

% denote the input is from a finite alphabet
sysStruct.Uset{1} = [-1 0 1];
ctrl = mpt_control(sysStruct, probStruct);

mbg_assertequal(length(ctrl), 34);
