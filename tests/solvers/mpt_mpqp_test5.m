function mpt_mpqp_test5

% tests that the solution of mpt_mpqp is correct by comparing it to a solution
% of a QP for a given point

Double_Integrator
M = mpt_constructMatrices(sysStruct, probStruct);
[Pn, Fi, Gi, AC, Ph, D] = mpt_mpqp(M);
mbg_assertequal(length(Pn), 25);

% solve a QP for chebycenter of each region
for ii = 1:length(Pn),
    x0 = chebyball(Pn(ii));
    [U_qp, cost_qp, feasible] = sub_qpsol(M, x0);
    
    % the QP must be feasible
    mbg_asserttrue(feasible);
    
    U_mp = Fi{ii}*x0 + Gi{ii};
    cost_mp = x0'*D.Ai{ii}*x0 + D.Bi{ii}*x0 + D.Ci{ii};
    
    % optimizer and cost must be identical
    mbg_asserttolequal(cost_qp, cost_mp, 1e-10);
    mbg_asserttolequal(U_qp, U_mp, 1e-10);
end


%------------------------------------------------------------
function [U, cost, feasible] = sub_qpsol(M, x0)
% solve the QP for x0

A = M.G;
B = M.W + M.E*x0;

H = M.H;
f = x0'*M.F + M.Cf;

[xopt, lam, how, ef, fval] = mpt_solveQP(H, f, A, B);
feasible = strcmpi(how, 'ok');
U = xopt;
cost = fval + x0'*M.Y*x0 + M.Cx*x0 + M.Cc;
