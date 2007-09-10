function mpt_searchTree_issue519

% issue519: mpt_searchTree failed in certain cases

load mpt_searchTree_issue519
st = mpt_searchTree(ctrl);
mbg_assertequal(size(st.details.searchTree), [230 11]);
