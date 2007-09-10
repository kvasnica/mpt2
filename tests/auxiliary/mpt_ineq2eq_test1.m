function mpt_ineq2eq_test1

% tests whether we properly return 5th output argument:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=0779d505b163

A=[1 1; -1 -1; eye(2)]; B=[1; -1; 2; 3];
[a,b,ae,be,i]=mpt_ineq2eq([A; A], [B; B]);

mbg_assertequal(a, [eye(2); eye(2)]);
mbg_assertequal(b, [2; 3; 2; 3]);
mbg_assertequal(ae, [1 1; -1 -1; 1 1]);
mbg_assertequal(be, [1; -1; 1]);
mbg_assertequal(i, [1 2; 2 5; 5 6]);
