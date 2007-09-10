function mpt_optMixedCtrl_test1

% tests that solution of mpt_optMixedCtrl is identical to solution of an on-line
% MPC controller (i.e. MILP) for a given state

% system with one real and one boolean input
two_tanks
probStruct.N = 2;
ctrl_exp = mpt_control(sysStruct, probStruct);
ctrl_onl = mpt_control(sysStruct, probStruct, 'online');

%mbg_assertequal(length(ctrl_exp), 22);
mbg_asserttrue(length(ctrl_exp) > 20);

for ii = 1:length(ctrl_exp),
    % assume x0 as the chebycenter of each region
    x0 = chebyball(ctrl_exp.Pn(ii));
    
    [U_exp, r, cost_exp] = mpt_getInput(ctrl_exp, x0);
    [U_onl, r, cost_onl] = mpt_getInput(ctrl_onl, x0);
    
    % cost must be identical
    mbg_asserttolequal(cost_exp, cost_onl, 1e-7);
    
    % control law does not have to be the same, becuase of non-uniqueness
end
