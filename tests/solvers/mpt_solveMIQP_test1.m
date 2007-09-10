function mpt_solveMIQP_test1

% tests that the branch&bound method gives correct solution
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=42fe8446872e

load miqp_prob
[xopt, fval, how, ef] = mpt_solveMIQP(H, f, A, B, [], [], [], [], vartype, [], [], 1);

% without the above patch we have been getting fval=0 where the true minimum is
% -1.6142 (verified with cplex)
mbg_asserttolequal(fval, -1.6142, 1e-3);
