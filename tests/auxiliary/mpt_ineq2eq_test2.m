function mpt_ineq2eq_test2

% this example was returning wrong equalities in Matlab R12 due to some
% numerical issues. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=eed2f8dbdff4

load mpt_ineq2eq_R12_prob
[aa,bb,aeq,beq,indeq]=mpt_ineq2eq(A, b);

mbg_assertequal(indeq, [33 35; 34 36; 71 73; 72 74]);
