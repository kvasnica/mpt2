function mpt_solveLP_test2

% NAG has problems when there are no inequality constraints defined. we must
% handle that case correctly in mpt_solveLPi():
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=894c960763bf

opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

f = randn(10, 1);
A = []; B = [];     % no inequality constraints
Aeq = eye(10);
Beq = rand(10, 1);

xopt = mpt_solveLPi(f, A, B, Aeq, Beq, [], 0);
mbg_asserttolequal(xopt, Beq);

xopt = mpt_solveLPi(f, A, B, Aeq, Beq, [], 9);
mbg_asserttolequal(xopt, Beq);
