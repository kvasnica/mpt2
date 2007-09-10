function mpt_solveQP_test2

% tests that lagrange multipliers are correct
%
% as of Nov-18-2005 all QP solvers return wrong lagrange multipliers. if we
% compute the slacks manually by plugging the optimizer back into constraints,
% we get different set of active constraints!
load qp_lambda_prob 

% following randomization fixes the problem:
%B = B+rand(size(B))*1e-11;

[xopt,lambda,how,exitflag,objqp]=mpt_solveQP(H,f,A,B,Aeq,Beq,[],1);

slacks = (B-A*xopt);

indact = find(abs(slacks) < 1e-12);
indact2 = find(lambda > 1e-12);
isequal(indact, indact2)
