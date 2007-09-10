function info_test1

% tests the info() method for explicit controllers:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=6fe57323a98e

fprintf('Controller with a search tree:\n');
load ctrl_1d_searchtree
info(ctrl_st);

fprintf('Controller without a search tree:\n');
load ctrl1
info(ctrl);

fprintf('An on-line controller:\n');
load ctrl1
probStruct = ctrl.probStruct;
probStruct.Tconstraint = 0;
ctrl = set(ctrl, 'probStruct', probStruct);
ctrl = mpt_control(ctrl.sysStruct, ctrl.probStruct, 'online');
info(ctrl);
