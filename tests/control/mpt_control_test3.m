function mpt_control_test3

% following changeset:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=cedc93418d40
% introduced verification of sysStruct/probStruct with Options.useyalmip=1,
% which breaks input blocking features if mpt_optControl is used. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=1559dcb39dea

x0 = [1;1];
Double_Integrator
probStruct.Tconstraint = 0;
probStruct.Nc = 2;
probStruct.N = 3;

ctrl = mpt_control(sysStruct, probStruct);
u = mpt_getInput(ctrl, x0, struct('openloop', 1));
mbg_asserttolequal(u(2), u(3));
