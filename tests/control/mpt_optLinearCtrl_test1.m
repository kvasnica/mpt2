function mpt_optLinearCtrl_test1

% all inputs continuous

% 1-norm case
opt_sincos
probStruct.N = 2;
probStruct.norm = 1;
ctrl_mpt=mpt_control(sysStruct, probStruct, struct('prefred', 'mpt'));  % miroslav's DP
ctrl_yalmip=mpt_yalmipcftoc(sysStruct, probStruct); % your DP
b=mpt_isPWAbigger(ctrl_mpt, ctrl_yalmip);
mbg_assertequal(b, 0);   % costs must be identical

opt_sincos
probStruct.N = 2;
probStruct.norm = Inf;
ctrl_mpt=mpt_control(sysStruct,probStruct,struct('prefered', 'mpt'));  % miroslav's DP
ctrl_yalmip=mpt_yalmipcftoc(sysStruct,probStruct); % your DP
b=mpt_isPWAbigger(ctrl_mpt, ctrl_yalmip);
mbg_assertequal(b, 0);   % costs must be identical
