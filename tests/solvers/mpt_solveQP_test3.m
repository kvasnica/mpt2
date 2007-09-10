function mpt_solveQP_test3

% tests that we properly deal with empty inequality constraints:
%

f = 1;
H = 1;
Aeq = 1;
Beq = 1;
A = [];
B = [];

% test quadprog (it worked also before)
xopt = mpt_solveQP(H, f, A, B, Aeq, Beq, [], 1);
mbg_asserttolequal(xopt, 1);


opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

% test NAG, it was giving an error:
%    ??? RHS argument (x) is undefined
xopt = mpt_solveQP(H, f, A, B, Aeq, Beq, [], 0);
mbg_asserttolequal(xopt, 1);

