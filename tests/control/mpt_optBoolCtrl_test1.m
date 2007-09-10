function mpt_optBoolCtrl_test1

% test control of systems with boolean/discrete inputs
pwa1d
probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2;

% denote the input as boolean
sysStruct.Uset{1} = [0 1];
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 2);

% denote the input is from a finite alphabet
sysStruct.Uset{1} = [-0.5 0 0.5];

% use N=1, test if we handle such case correctly
probStruct.N = 1;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 2);
