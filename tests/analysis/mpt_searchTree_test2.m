function mpt_searchTree_test2

% tests that mpt_searchTree() only assigns modified object in the caller's
% workspace if nargout==0

load lti1d_ctrl_N1_norm1

% search tree should only be stored into the "st" object
st = mpt_searchTree(ctrl);
mbg_asserttrue(isfield(st.details, 'searchTree'));
mbg_assertfalse(isfield(ctrl.details, 'searchTree'));

% search tree should be stored into the "ctrl" object
mpt_searchTree(ctrl);
mbg_asserttrue(isfield(ctrl.details, 'searchTree'));
