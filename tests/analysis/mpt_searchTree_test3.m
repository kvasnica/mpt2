function mpt_searchTree_test3

% test mpt_searchTree with non-covnex ctrl.Pfinal

opt_sincos
sysStruct.guardX{2} = [-1 0; 0 1];
sysStruct.guardC{2} = [0; 3];
sysStruct.xmax = [20; 20];
sysStruct.xmin = [-20; -20];
sysStruct.ymax = sysStruct.xmax;
sysStruct.ymin = sysStruct.xmin;
probStruct.N = 2;
ctrl = mpt_control(sysStruct, probStruct);
st = mpt_searchTree(ctrl);

X0 = mpt_feasibleStates(ctrl, 20);

% test that the search tree evaluation gives the same result as brute-force
% search through all regions
for i = 1:size(X0, 1)
    x0 = X0(i, :)';
    [u_ctrl,f_ctrl,r_ctrl,c_ctrl,d_ctrl] = mpt_getInput(ctrl, x0);
    [u_st,f_st,r_st,c_st,d_st] = mpt_getInput(st, x0);
    mbg_asserttolequal(u_ctrl, u_st);  % control move is identical
%     mbg_assertequal(r_ctrl, r_st);     % active region is identical
%     mbg_assertequal(d_ctrl.inwhich, d_st.inwhich);   % active region is identical
    mbg_asserttrue(d_st.nops.multiplications > 0); % search tree was really traversed
end
