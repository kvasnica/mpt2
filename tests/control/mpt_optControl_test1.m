function mpt_optControl_test1

% CFTOC for 2 norm for 1D systems was failing due to an error in facetcircle
lti1d
probStruct.norm = 2;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 5);

% CFTOC for 1-norm for 1D systems
lti1d
probStruct.norm = 1;
probStruct.P_N = 0;
ctrl_mpt = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_mpt), 4);

% test also linear norms with the old method - not with PWA cost-to-go
lti1d
probStruct.norm = 1;
%ctrl1old = mpt_control(sysStruct, probStruct, struct('mplpver',3));
ctrl_yalmip = mpt_control(sysStruct, probStruct, struct('prefered','yalmip'));
% if the DP approach is used in mpt_yalmipcftoc (default), then the solution has
% 2 regions. one-shot solution (Options.dp=0) leads 4 regions
mbg_assertequal(length(ctrl_yalmip), 2);

% cost must be identical
b = mpt_isPWAbigger(ctrl_mpt, ctrl_yalmip);
mbg_assertequal(b, 0);
