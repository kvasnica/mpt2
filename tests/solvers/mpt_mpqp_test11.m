function mpt_mpqp_test11

% empty hardA,hardb matrices caused a break in mpt_mpqp. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/96da9fea84ba

load mpt_mpqp_test11
[Pn, Fi, Gi, AC, Phard] = mpt_mpqp(Matrices, struct('verbose', 2));
mbg_asserttrue(Pn == Phard);
mbg_asserttrue(isfulldim(Pn));
