function mpt_searchTree_test1

% tests mpt_searchTree()

load lti1d_ctrl_N1_norm1

st = mpt_searchTree(ctrl);
mbg_assertequal(st.details.searchTree, [-1 0 2 3;-1 3 -4 -1;1 3 -2 -1]);
