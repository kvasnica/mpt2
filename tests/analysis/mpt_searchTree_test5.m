function mpt_searchTree_test5

% mpt_searchTree would give wrong results if Pfinal is wrong

Double_Integrator
probStruct.N = 5;
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
ctrl = mpt_control(sysStruct, probStruct, struct('verbose', 0));

% shrink the feasible set considerably
ctrl_badPfinal = struct(ctrl);
ctrl_badPfinal.Pfinal = 0.5*ctrl_badPfinal.Pfinal;
ctrl_badPfinal = mptctrl(ctrl_badPfinal);

good_st = mpt_searchTree(ctrl);
bad_st = mpt_searchTree(ctrl_badPfinal);

% check that both search trees give the same result when evaluated for a
% corner initial condition
x0 = [-5; 3];
u_orig = ctrl(x0);
u_good = good_st(x0);
u_bad = bad_st(x0);

mbg_asserttolequal(u_orig, u_good);
mbg_asserttolequal(u_orig, u_bad);
mbg_assertequal(size(good_st.details.searchTree), ...
    size(bad_st.details.searchTree));
