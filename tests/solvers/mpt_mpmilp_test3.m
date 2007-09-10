function mpt_mpmilp_test3

% tests that the solution of mpt_mpmilp is correct by comparing it to a solution
% of a MILP for a given point

Double_Integrator
probStruct.N = 1; probStruct.norm = 1;
M = mpt_constructMatrices(sysStruct, probStruct);
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1) = 'B';

S = mpt_mpmilp(M);
mbg_assertequal(length(S.Pn), 16);

% solve an MILP for chebycenter of each region
for ii = 1:length(S.Pn),
    x0 = chebyball(S.Pn(ii));
    [U_lp, cost_lp, feasible] = sub_milpsol(M, x0);
    
    % the MILP must be feasible
    mbg_asserttrue(feasible);
    
    % solution of mpMILP could be overlapping, identify region with minimal cost
    [isin, inwhich] = isinside(S.Pn, x0);
    cost_mp = [];
    for jj = 1:length(inwhich),
        reg = inwhich(jj);
        cost_mp = [cost_mp; S.Bi{reg}*x0 + S.Ci{reg}];
    end
    [a, b] = min(cost_mp);
    cost_mp = a;
    minreg = inwhich(b);  % region in which cost is minimal
    U_mp = S.Fi{minreg}*x0 + S.Gi{minreg};

    % optimizer and cost must be identical
    mbg_asserttolequal(cost_lp, cost_mp, 1e-9);
    mbg_asserttolequal(U_lp, U_mp, 1e-9);
end


%------------------------------------------------------------
function [U, cost, feasible] = sub_milpsol(M, x0)
% solve the QP for x0

A = M.G;
B = M.W + M.E*x0;

f = M.H;

[xopt, fval, how, ef] = mpt_solveMILP(f, A, B, [], [], [], [], M.vartype);
feasible = strcmpi(how, 'ok');
U = xopt;
cost = fval + M.F*x0;
