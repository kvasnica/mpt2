function mpt_mpmiqp_test3

% tests that the solution of mpt_mpmiqp is correct by comparing it to a solution
% of a MIQP for a given point

Double_Integrator
probStruct.N = 2;
M = mpt_constructMatrices(sysStruct, probStruct);
M.vartype(1) = 'C';
M.vartype(2) = 'B';
S = mpt_mpmiqp(M);
mbg_assertequal(length(S.Pn), 14);

%% we can also test the 1D example:
% lti1d
% probStruct.N = 3; probStruct.norm = 2;
% M = mpt_constructMatrices(sysStruct, probStruct);
% M.vartype = repmat('B', size(M.G, 2), 1);
% S = mpt_mpmiqp(M);
% mbg_assertequal(length(S.Pn), 8);


% solve a MIQP for chebycenter of each region
for ii = 1:length(S.Pn),
    x0 = chebyball(S.Pn(ii));
    [U_qp, cost_qp, feasible] = sub_miqpsol(M, x0);
    
    % the MIQP must be feasible
    mbg_asserttrue(feasible);
    
    % solution of mpMIQP is overlapping, identify region with minimal cost
    [isin, inwhich] = isinside(S.Pn, x0);
    cost_mp = [];
    for jj = 1:length(inwhich),
        reg = inwhich(jj);
        cost_mp = [cost_mp; x0'*S.Ai{reg}*x0 + S.Bi{reg}*x0 + S.Ci{reg}];
    end
    [a, b] = min(cost_mp);
    cost_mp = a;
    minreg = inwhich(b);  % region in which cost is minimal
    U_mp = S.Fi{minreg}*x0 + S.Gi{minreg};

    % optimizer and cost must be identical. we use gigher tolerances here
    % because the Branch&Bound method often produces slightly different results
    mbg_asserttolequal(cost_qp, cost_mp, 1e-6);
    mbg_asserttolequal(U_qp, U_mp, 1e-3);
end


%------------------------------------------------------------
function [U, cost, feasible] = sub_miqpsol(M, x0)
% solve the QP for x0

A = M.G;
B = M.W + M.E*x0;

H = M.H;
f = x0'*M.F + M.Cf;

[xopt, fval, how, ef] = mpt_solveMIQP(H, f, A, B, [], [], [], [], M.vartype);
feasible = strcmpi(how, 'ok');
U = xopt;
cost = fval + x0'*M.Y*x0 + M.Cx*x0 + M.Cc;
