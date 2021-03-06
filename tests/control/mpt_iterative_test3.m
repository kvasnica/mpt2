function mpt_iterative_test3

% test minimum-time w/ noise

lti1d_addU
% set up minimum-time problem w/ 2-norm
probStruct.subopt_lev = 1;
probStruct.N = Inf;
probStruct.norm = 2;

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 7);

% test also linear norms
probStruct.norm = 1;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 11);
