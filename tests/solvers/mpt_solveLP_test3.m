function mpt_solveLP_test3

% tests that we handle infinity terms in constraints:
%

A=[eye(2); eye(2); -eye(2)];
Binfeasible=[repmat(1, 4, 1); repmat(-Inf, 2, 1)];
Bfeasible=[repmat(1, 4, 1); repmat(Inf, 2, 1)];

[xopt, d1, d2, ef, how] = mpt_solveLP([1 1], A, Bfeasible, [], [], [], 3);
mbg_asserttrue(isequal(how, 'ok'));
mbg_assertequal(ef, 1);

[xopt, d1, d2, ef, how] = mpt_solveLP([1 1], A, Binfeasible, [], [], [], 3);
mbg_asserttrue(isequal(how, 'infeasible'));
mbg_assertequal(ef, -1);
