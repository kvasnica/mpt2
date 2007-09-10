function mpt_getInput_test5

% tests that we inform the user that initial condition violates state
% constraints:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=1d9c35868c0a

Double_Integrator
sysStruct.xmax = [1;1];
sysStruct.xmin = [-1;-1];
ctrl = mpt_control(sysStruct, probStruct);
u = mpt_getInput(ctrl, [2;2]);
