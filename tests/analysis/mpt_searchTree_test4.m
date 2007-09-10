function mpt_searchTree_test4

% test mpt_searchTree() on controllers with only one region
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/e846869a4787

Double_Integrator
probStruct.N = 1;
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
ctrl = mpt_control(sysStruct, probStruct, struct('verbose', 0));
mbg_assertequal(length(ctrl.Pn), 1);

% without a patch, mpt_searchTree would break with an error
st = mpt_searchTree(ctrl);
mbg_assertequal(st.details.searchTree, [0 0 -1 -1]);
