function mpt_solveMILP_test1

% Bug 70: Need to check more statuses when calling glpkmex

A = [1; -1];
B = [1; 1];
f = 1;
vartype = 'C';
[a, b, c, d] = mpt_solveMILP(f, A, B, [], [], [], [], vartype, [], [], 2);
mbg_asserttrue(isequal(c, 'ok'));
mbg_assertequal(d, 1);
