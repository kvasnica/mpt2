function mpt_mplp_test13

% tests that the solution of mpt_mplp is correct by comparing it to a solution
% of an LP for a given point

Double_Integrator
probStruct.N = 2; probStruct.norm = 1;
M = mpt_constructMatrices(sysStruct, probStruct);
[Pn, Fi, Gi, AC, Ph, D] = mpt_mplp(M);
mbg_assertequal(length(Pn), 28);

% solve a QP for chebycenter of each region
for ii = 1:length(Pn),
    x0 = chebyball(Pn(ii));
    [U_lp, cost_lp, feasible] = sub_lpsol(M, x0);
    
    % the QP must be feasible
    mbg_asserttrue(feasible);
    
    U_mp = Fi{ii}*x0 + Gi{ii};
    cost_mp = D.Bi{ii}*x0 + D.Ci{ii};

    % optimizer and cost must be identical
    mbg_asserttolequal(cost_lp, cost_mp, 1e-10);
    mbg_asserttolequal(U_lp, U_mp, 1e-10);
end


%------------------------------------------------------------
function [U, cost, feasible] = sub_lpsol(M, x0)
% solve the QP for x0

A = M.G;
B = M.W + M.E*x0;

f = M.H;

[xopt, fval, lam, ef, how] = mpt_solveLP(f, A, B);
feasible = strcmpi(how, 'ok');
U = xopt;
cost = fval + M.F*x0;
