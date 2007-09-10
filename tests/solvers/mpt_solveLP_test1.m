function mpt_solveLP_test1

% we check if inequality constraints are satisfied at the end of mpt_solveLP().
% that check, however, should only be performed if some inequality constraints
% are given. verify that here...

% no inequality constraints
A = [];
B = [];

% just equality constraints
Aeq = eye(2);
Beq = ones(2,1);

xopt = mpt_solveLP([1 1], A, B, Aeq, Beq, [], 3);
mbg_assertequal(xopt, inv(Aeq)*Beq);
