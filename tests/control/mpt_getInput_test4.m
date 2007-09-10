function mpt_getInput_test4

% tests whether we correctly return number of operations needed to identify an
% optimal control law:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=b5541222c605

load ctrl_1d_searchtree

[U,feasible,region,cost,details]=mpt_getInput(ctrl_st,1);

mbg_asserttrue(isfield(details, 'nops'));
mbg_assertequal(details.nops.multiplications, 3);
mbg_assertequal(details.nops.summations, 6);
mbg_assertequal(details.nops.comparisons, 2);
