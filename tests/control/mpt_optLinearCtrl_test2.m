function mpt_optLinearCtrl_test2

% boolean inputs, output regulation

% 1-norm case
two_tanks
probStruct.N = 2;
ctrl=mpt_control(sysStruct,probStruct);
%mbg_assertequal(length(ctrl), 22);
mbg_asserttrue(length(ctrl)>20);
