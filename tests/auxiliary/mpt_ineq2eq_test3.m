function mpt_ineq2eq_test3

% test that we handle tolerances correctly. hard-coded tolerance in
% mpt_ineq2eq is 1e-12


% tolerance-equal
A=[1 0; -1 0+1e-13]; B=[0;0];
[ain,bin,aeq,beq]=mpt_ineq2eq(A, B);
mbg_asserttrue(isempty(ain));
mbg_asserttrue(isempty(bin));
mbg_assertequal(aeq, [1 0]);
mbg_assertequal(beq, 0);

% not tolerance-equal
A=[1 0; -1 0+1e-11]; B=[0;0];
[ain,bin,aeq,beq]=mpt_ineq2eq(A, B);
mbg_assertequal(ain, A);
mbg_assertequal(bin, B);
mbg_asserttrue(isempty(aeq));
mbg_asserttrue(isempty(beq));
