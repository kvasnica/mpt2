function mpt_solveQP_test1

% tests if we are dealing correctly with equality constraints for NAG:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=e95314c45a4f

opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

% use NAG
xopt = mpt_solveQP(1,1,[1;-1],[1;1],2,1,[],0);

% equality constraint 2*xopt = 1 => xopt=0.5
mbg_assertequal(xopt, 0.5);


% use quadprog
xopt = mpt_solveQP(1,1,[1;-1],[1;1],2,1,[],1);

% equality constraint 2*xopt = 1 => xopt=0.5
mbg_assertequal(xopt, 0.5);
