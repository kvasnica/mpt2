function mpt_getInput_test6

% verify that we return a proper full open-loop optimizer even if move blocking
% was used:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3374be93951f

x0 = [1;1];
Double_Integrator
probStruct.N = 3;
probStruct.Nc = 2;
probStruct.norm = 1;
ctrl = mpt_control(sysStruct, probStruct);
u = mpt_getInput(ctrl, x0, struct('openloop', 1));
mbg_assertequal(length(u), probStruct.N);
mbg_asserttolequal(u(2), u(3));

x0 = [1;1];
opt_sincos
probStruct.N = 3;
probStruct.Nc = 1;
probStruct.norm = 2;
probStruct.Tconstraint = 0;
ctrl = mpt_control(sysStruct, probStruct);
u = mpt_getInput(ctrl, x0, struct('openloop', 1));
mbg_assertequal(length(u), probStruct.N);
mbg_asserttolequal(u(1), u(3));
mbg_asserttolequal(u(2), u(3));

x0 = [1;1];
Double_Integrator
probStruct.N = 3;
probStruct.Nc = 2;
probStruct.norm = 1;
ctrl = mpt_control(sysStruct, probStruct, 'online');
u = mpt_getInput(ctrl, x0, struct('openloop', 1));
mbg_assertequal(length(u), probStruct.N);
mbg_asserttolequal(u(2), u(3));

x0 = [1;1];
opt_sincos
probStruct.N = 3;
probStruct.Nc = 1;
probStruct.norm = 2;
probStruct.Tconstraint = 0;
ctrl = mpt_control(sysStruct, probStruct, 'online');
u = mpt_getInput(ctrl, x0, struct('openloop', 1));
mbg_assertequal(length(u), probStruct.N);
mbg_asserttolequal(u(1), u(3));
mbg_asserttolequal(u(2), u(3));


x0 = [0.5; 0.1];
two_tanks
probStruct.N = 3;
probStruct.Nc = 1;
ctrl = mpt_control(sysStruct, probStruct, 'online');
u = mpt_getInput(ctrl, x0, struct('openloop', 1));
mbg_assertequal(length(u), probStruct.N*2);
mbg_asserttolequal(u(1:2), u(3:4));
mbg_asserttolequal(u(1:2), u(5:6));
